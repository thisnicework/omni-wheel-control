/*
 * app.js
 * Omni-wheel Robot Controller Engine
 * 
 * Hold-To-Drive Mode:
 *   - Robot drives ONLY while button/key is pressed or held down!
 *   - Releases button or key -> Dispatches instant STOP 'x' command.
 */

// ==========================================
// 1. WEB SERIAL API CONTROLLER (USB DIRECT)
// ==========================================
class SerialController {
    constructor(onData, onStatus) {
        this.onData = onData;
        this.onStatus = onStatus;
        this.port = null;
        this.reader = null;
        this.writer = null;
        this.keepReading = false;
    }

    async connect() {
        if (!('serial' in navigator)) {
            alert('Web Serial API is not supported in this browser. Please use Google Chrome or Edge.');
            return false;
        }

        try {
            this.port = await navigator.serial.requestPort();
            await this.port.open({ baudRate: 115200 });

            const textEncoder = new TextEncoderStream();
            this.writableStreamClosed = textEncoder.readable.pipeTo(this.port.writable);
            this.writer = textEncoder.writable.getWriter();

            this.keepReading = true;
            this.readLoop();

            if (this.onStatus) this.onStatus(true, 'USB SERIAL CONNECTED');
            return true;
        } catch (error) {
            console.error('[Web Serial Error]', error);
            if (this.onStatus) this.onStatus(false, 'SERIAL ERROR');
            this.disconnect();
            return false;
        }
    }

    async readLoop() {
        const textDecoder = new TextDecoderStream();
        this.readableStreamClosed = this.port.readable.pipeTo(textDecoder.writable);
        const reader = textDecoder.readable.getReader();
        this.reader = reader;

        let buffer = '';
        try {
            while (this.keepReading) {
                const { value, done } = await reader.read();
                if (done) break;
                if (value) {
                    buffer += value;
                    const lines = buffer.split('\n');
                    buffer = lines.pop();
                }
            }
        } catch (error) {
            console.warn('[Web Serial Read Error]', error);
        } finally {
            reader.releaseLock();
        }
    }

    async send(data) {
        if (this.writer) {
            try {
                await this.writer.write(data);
            } catch (err) {
                console.error('[Web Serial Send Error]', err);
            }
        }
    }

    async disconnect() {
        this.keepReading = false;
        if (this.reader) await this.reader.cancel().catch(() => {});
        if (this.writer) await this.writer.close().catch(() => {});
        if (this.port) await this.port.close().catch(() => {});
        this.port = null;
        this.reader = null;
        this.writer = null;
        if (this.onStatus) this.onStatus(false, 'DISCONNECTED');
    }
}


// ==========================================
// 2. CLOUD MQTT WSS CONTROLLER (RENDER HTTPS)
// ==========================================
class CloudMqttWssController {
    constructor(onStatusChange) {
        this.onStatusChange = onStatusChange;
        this.client = null;
        this.isConnected = false;
        this.connect();
    }

    connect() {
        console.log('[Cloud MQTT WSS] Connecting to WSS Cloud Broker (broker.hivemq.com:8884)...');
        try {
            if (typeof mqtt === 'undefined') {
                console.warn('MQTT.js library not loaded yet');
                return;
            }
            this.client = mqtt.connect('wss://broker.hivemq.com:8884/mqtt', {
                clientId: 'OmniWeb_' + Math.random().toString(16).substr(2, 8),
                clean: true,
                connectTimeout: 5000,
                reconnectPeriod: 2500
            });

            this.client.on('connect', () => {
                console.log('✅ [Cloud MQTT WSS] Connected to WSS Cloud Broker!');
                this.isConnected = true;
                if (this.onStatusChange) this.onStatusChange(true, 'CLOUD MQTT ACTIVE');
            });

            this.client.on('offline', () => {
                this.isConnected = false;
                if (this.onStatusChange) this.onStatusChange(false, 'CLOUD OFFLINE');
            });

            this.client.on('error', (err) => {
                console.warn('[Cloud MQTT Error]', err);
            });
        } catch (e) {
            console.error('[Cloud MQTT Exception]', e);
        }
    }

    send(cmd) {
        if (this.client && this.isConnected) {
            this.client.publish('omniwheel/cmd', cmd);
            console.log(`⚡ [Cloud MQTT Published] Sent '${cmd}' to omniwheel/cmd`);
        }
    }
}


// ==========================================
// 3. MULTI-CHANNEL CONTROL DISPATCHER
// ==========================================
class RobotControlDispatcher {
    constructor(serialController, cloudMqttController, statusBadge, statusText) {
        this.serialController = serialController;
        this.cloudMqttController = cloudMqttController;
        this.statusBadge = statusBadge;
        this.statusText = statusText;
        this.lastCmd = null;
    }

    updateStatus(isConnected, text) {
        if (this.statusBadge && this.statusText) {
            if (isConnected) {
                this.statusBadge.classList.add('connected');
                this.statusText.textContent = text;
            } else {
                this.statusBadge.classList.remove('connected');
                this.statusText.textContent = text;
            }
        }
    }

    send(cmd) {
        this.lastCmd = cmd;

        // Path 1: Cloud MQTT WSS (Render HTTPS)
        if (this.cloudMqttController) {
            this.cloudMqttController.send(cmd);
        }

        // Path 2: USB Web Serial (Local Cable Backup)
        if (this.serialController && this.serialController.port) {
            this.serialController.send(cmd);
        }

        // Path 3: Render REST API Backup (/api/cmd?set=w)
        fetch('/api/cmd?set=' + cmd).catch(() => {});
    }
}


// ==========================================
// 4. MAIN APPLICATION & HOLD-TO-DRIVE BINDINGS
// ==========================================
document.addEventListener('DOMContentLoaded', () => {
    // DOM Elements
    const btnConnect = document.getElementById('btn-connect');
    const statusBadge = document.getElementById('status-badge');
    const statusText = document.getElementById('status-text');

    const btnUp = document.getElementById('btn-up');
    const btnDown = document.getElementById('btn-down');
    const btnLeft = document.getElementById('btn-left');
    const btnRight = document.getElementById('btn-right');
    const btnStop = document.getElementById('btn-stop');
    const btnAuto = document.getElementById('btn-auto');
    const btnStopAll = document.getElementById('btn-stop-all');
    const speedPills = document.querySelectorAll('.speed-pill');

    // Instantiate Controllers & Dispatcher
    const serialController = new SerialController(null, (isConnected, label) => {
        dispatcher.updateStatus(isConnected, label);
    });

    const cloudMqttController = new CloudMqttWssController((isConnected, label) => {
        dispatcher.updateStatus(isConnected, label);
    });

    const dispatcher = new RobotControlDispatcher(serialController, cloudMqttController, statusBadge, statusText);

    // Speed Pill Click Handler
    speedPills.forEach(pill => {
        pill.addEventListener('click', (e) => {
            e.preventDefault();
            speedPills.forEach(p => p.classList.remove('active'));
            pill.classList.add('active');
            const level = pill.getAttribute('data-speed');
            dispatcher.send(level);
        });
    });

    // USB Serial Connect Button
    if (btnConnect) {
        btnConnect.addEventListener('click', async () => {
            if (serialController.port) {
                await serialController.disconnect();
            } else {
                await serialController.connect();
            }
        });
    }

    // Helper: Hold-To-Drive Binding Function
    function bindHoldToDrive(element, moveCmd) {
        if (!element) return;

        const startMove = (e) => {
            e.preventDefault();
            element.classList.add('active');
            dispatcher.send(moveCmd);
        };

        const stopMove = (e) => {
            e.preventDefault();
            element.classList.remove('active');
            dispatcher.send('x');
        };

        // Mouse Events
        element.addEventListener('mousedown', startMove);
        element.addEventListener('mouseup', stopMove);
        element.addEventListener('mouseleave', stopMove);

        // Touch Events (Mobile Touchscreen)
        element.addEventListener('touchstart', startMove, { passive: false });
        element.addEventListener('touchend', stopMove, { passive: false });
        element.addEventListener('touchcancel', stopMove, { passive: false });
    }

    // Bind D-Pad Direction Buttons
    bindHoldToDrive(btnUp, 'w');
    bindHoldToDrive(btnDown, 's');
    bindHoldToDrive(btnLeft, 'a');
    bindHoldToDrive(btnRight, 'd');

    // Tap Action Buttons (Stop & Auto Roam)
    if (btnStop) {
        btnStop.addEventListener('click', (e) => {
            e.preventDefault();
            dispatcher.send('x');
        });
    }

    if (btnAuto) {
        btnAuto.addEventListener('click', (e) => {
            e.preventDefault();
            dispatcher.send('i');
        });
    }

    if (btnStopAll) {
        btnStopAll.addEventListener('click', (e) => {
            e.preventDefault();
            dispatcher.send('o');
        });
    }

    // Keyboard WASD & Arrow Key Hold-To-Drive Handler
    const keyMap = {
        'w': { cmd: 'w', btn: btnUp },
        'arrowup': { cmd: 'w', btn: btnUp },
        's': { cmd: 's', btn: btnDown },
        'arrowdown': { cmd: 's', btn: btnDown },
        'a': { cmd: 'a', btn: btnLeft },
        'arrowleft': { cmd: 'a', btn: btnLeft },
        'd': { cmd: 'd', btn: btnRight },
        'arrowright': { cmd: 'd', btn: btnRight }
    };

    const activeKeys = new Set();

    window.addEventListener('keydown', (e) => {
        const k = e.key.toLowerCase();
        if (keyMap[k] && !activeKeys.has(k)) {
            e.preventDefault();
            activeKeys.add(k);
            if (keyMap[k].btn) keyMap[k].btn.classList.add('active');
            dispatcher.send(keyMap[k].cmd);
        } else if (k === 'x' || k === ' ') {
            e.preventDefault();
            dispatcher.send('x');
        } else if (k === 'i') {
            e.preventDefault();
            dispatcher.send('i');
        } else if (k >= '1' && k <= '9') {
            e.preventDefault();
            dispatcher.send(k);
        }
    });

    window.addEventListener('keyup', (e) => {
        const k = e.key.toLowerCase();
        if (keyMap[k]) {
            e.preventDefault();
            activeKeys.delete(k);
            if (keyMap[k].btn) keyMap[k].btn.classList.remove('active');
            
            // If all direction keys released, stop robot!
            if (activeKeys.size === 0) {
                dispatcher.send('x');
            }
        }
    });
});

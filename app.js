/*
 * app.js
 * Omni-wheel Robot Controller Engine
 * 
 * Multi-Channel Control Dispatcher:
 *   1. Cloud MQTT WSS (wss://broker.hivemq.com:8884/mqtt) -> Works 100% on Render HTTPS!
 *   2. USB Web Serial (Local Cable Backup)
 *   3. Render REST API Backup (/api/cmd?set=w)
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
        this.readableStreamClosed = null;
        this.writableStreamClosed = null;
        this.keepReading = false;
        
        this.packetCount = 0;
        this.lastFpsCalc = performance.now();
        this.currentHz = 0;
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
                    buffer = lines.pop(); // Keep partial frame in buffer

                    for (const line of lines) {
                        this.parseLine(line.trim());
                    }
                }
            }
        } catch (error) {
            console.warn('[Web Serial Read Error]', error);
        } finally {
            reader.releaseLock();
        }
    }

    parseLine(line) {
        if (!line) return;
        const tokens = line.split(',').map(Number);
        if (tokens.length === 4 && !tokens.some(isNaN)) {
            this.packetCount++;
            const now = performance.now();
            if (now - this.lastFpsCalc >= 1000) {
                this.currentHz = Math.round((this.packetCount * 1000) / (now - this.lastFpsCalc));
                this.packetCount = 0;
                this.lastFpsCalc = now;
            }

            if (this.onData) {
                this.onData(tokens, this.currentHz);
            }
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
        if (this.reader) {
            await this.reader.cancel().catch(() => {});
        }
        if (this.writer) {
            await this.writer.close().catch(() => {});
        }
        if (this.port) {
            await this.port.close().catch(() => {});
        }
        this.port = null;
        this.reader = null;
        this.writer = null;
        if (this.onStatus) this.onStatus(false, 'DISCONNECTED');
    }
}


// ==========================================
// 2. CLOUD MQTT WSS CONTROLLER (RENDER HTTPS COMPATIBLE)
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
        // Path 1: Cloud MQTT WSS (Works over Render HTTPS!)
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
// 4. MAIN APPLICATION BOOTSTRAP
// ==========================================
document.addEventListener('DOMContentLoaded', () => {
    // DOM Handles
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

    // Instantiate Controllers
    const serialController = new SerialController(
        (encoders, hz) => {},
        (isConnected, label) => {
            dispatcher.updateStatus(isConnected, label);
        }
    );

    const cloudMqttController = new CloudMqttWssController((isConnected, label) => {
        dispatcher.updateStatus(isConnected, label);
    });

    const dispatcher = new RobotControlDispatcher(serialController, cloudMqttController, statusBadge, statusText);

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

    // Command Sender Function
    const triggerCmd = (cmd, btnElement = null) => {
        dispatcher.send(cmd);
        if (btnElement) {
            btnElement.classList.add('active');
            setTimeout(() => btnElement.classList.remove('active'), 180);
        }
    };

    // Button Click Listeners
    if (btnUp) btnUp.addEventListener('click', () => triggerCmd('w', btnUp));
    if (btnDown) btnDown.addEventListener('click', () => triggerCmd('s', btnDown));
    if (btnLeft) btnLeft.addEventListener('click', () => triggerCmd('a', btnLeft));
    if (btnRight) btnRight.addEventListener('click', () => triggerCmd('d', btnRight));
    if (btnStop) btnStop.addEventListener('click', () => triggerCmd('x', btnStop));
    if (btnAuto) btnAuto.addEventListener('click', () => triggerCmd('i', btnAuto));
    if (btnStopAll) btnStopAll.addEventListener('click', () => triggerCmd('o', btnStopAll));

    // Keyboard WASD & Arrow Keys Event Listeners
    const keyMap = {
        'w': { cmd: 'w', btn: btnUp },
        'arrowup': { cmd: 'w', btn: btnUp },
        's': { cmd: 's', btn: btnDown },
        'arrowdown': { cmd: 's', btn: btnDown },
        'a': { cmd: 'a', btn: btnLeft },
        'arrowleft': { cmd: 'a', btn: btnLeft },
        'd': { cmd: 'd', btn: btnRight },
        'arrowright': { cmd: 'd', btn: btnRight },
        'x': { cmd: 'x', btn: btnStop },
        ' ': { cmd: 'x', btn: btnStop },
        'i': { cmd: 'i', btn: btnAuto },
        'o': { cmd: 'o', btn: btnStopAll }
    };

    window.addEventListener('keydown', (e) => {
        const k = e.key.toLowerCase();
        if (keyMap[k]) {
            e.preventDefault();
            triggerCmd(keyMap[k].cmd, keyMap[k].btn);
        }
    });
});

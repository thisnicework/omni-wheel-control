/*
 * app.js
 * Omni-wheel Robot Controller Engine (Ultra-Low Latency Local HTTP)
 * 
 * Architecture:
 *   - PRIMARY: Mac Local HTTP Server (http://192.168.10.140:8000/api/cmd)
 *   - BACKUP: USB Web Serial API (Chrome only, cable connection)
 *   - NO MQTT, NO Cloud. Pure local network for instant response.
 * 
 * Continuous Hold-To-Drive Pulse System (100ms):
 *   - While button or key is held down, sends heartbeat pulse every 100ms.
 *   - Release -> Clears interval & sends instant STOP 'x'.
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
// 2. LOCAL HTTP API CONTROLLER (PRIMARY)
// ==========================================
class LocalHttpController {
    constructor(onStatusChange) {
        this.onStatusChange = onStatusChange;
        this.isConnected = false;
        this.checkConnection();
    }

    checkConnection() {
        fetch('/api/status')
            .then(resp => resp.json())
            .then(data => {
                this.isConnected = true;
                if (this.onStatusChange) {
                    this.onStatusChange(true, 'LOCAL HTTP ACTIVE');
                }
            })
            .catch(err => {
                this.isConnected = false;
                if (this.onStatusChange) {
                    this.onStatusChange(false, 'SERVER OFFLINE');
                }
                // Retry in 3 seconds
                setTimeout(() => this.checkConnection(), 3000);
            });
    }

    send(cmd) {
        fetch('/api/cmd?set=' + cmd).catch(() => {
            this.isConnected = false;
            if (this.onStatusChange) {
                this.onStatusChange(false, 'SERVER OFFLINE');
            }
        });
    }
}


// ==========================================
// 3. MULTI-CHANNEL CONTROL DISPATCHER
// ==========================================
class RobotControlDispatcher {
    constructor(serialController, localHttpController, statusBadge, statusText) {
        this.serialController = serialController;
        this.localHttpController = localHttpController;
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

        // Path 1: Local HTTP API (Primary, <5ms)
        if (this.localHttpController) {
            this.localHttpController.send(cmd);
        }

        // Path 2: USB Web Serial (Backup, cable only)
        if (this.serialController && this.serialController.port) {
            this.serialController.send(cmd);
        }
    }
}


// ==========================================
// 4. MAIN APPLICATION & CONTINUOUS HOLD PULSE BINDINGS
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

    const localHttpController = new LocalHttpController((isConnected, label) => {
        dispatcher.updateStatus(isConnected, label);
    });

    const dispatcher = new RobotControlDispatcher(serialController, localHttpController, statusBadge, statusText);

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

    // Active Hold Pulse Timer Storage
    let holdPulseTimer = null;
    let currentHoldingCmd = null;

    function startHoldPulse(cmd, element) {
        if (currentHoldingCmd === cmd) return;
        currentHoldingCmd = cmd;
        if (element) element.classList.add('active');

        // Immediate first pulse
        dispatcher.send(cmd);

        // Continuous 100ms heartbeat pulses while held down
        if (holdPulseTimer) clearInterval(holdPulseTimer);
        holdPulseTimer = setInterval(() => {
            if (currentHoldingCmd) {
                dispatcher.send(currentHoldingCmd);
            }
        }, 100);
    }

    function stopHoldPulse(element) {
        currentHoldingCmd = null;
        if (holdPulseTimer) {
            clearInterval(holdPulseTimer);
            holdPulseTimer = null;
        }
        if (element) element.classList.remove('active');
        dispatcher.send('x');
    }

    function bindHoldToDrive(element, moveCmd) {
        if (!element) return;

        // Mouse Events
        element.addEventListener('mousedown', (e) => {
            e.preventDefault();
            startHoldPulse(moveCmd, element);
        });

        element.addEventListener('mouseup', (e) => {
            e.preventDefault();
            stopHoldPulse(element);
        });

        element.addEventListener('mouseleave', (e) => {
            e.preventDefault();
            if (currentHoldingCmd === moveCmd) {
                stopHoldPulse(element);
            }
        });

        // Touch Events (Mobile Touchscreen)
        element.addEventListener('touchstart', (e) => {
            e.preventDefault();
            startHoldPulse(moveCmd, element);
        }, { passive: false });

        element.addEventListener('touchend', (e) => {
            e.preventDefault();
            stopHoldPulse(element);
        }, { passive: false });

        element.addEventListener('touchcancel', (e) => {
            e.preventDefault();
            stopHoldPulse(element);
        }, { passive: false });
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
            stopHoldPulse(null);
        });
    }

    if (btnAuto) {
        btnAuto.addEventListener('click', (e) => {
            e.preventDefault();
            stopHoldPulse(null);
            dispatcher.send('i');
        });
    }

    if (btnStopAll) {
        btnStopAll.addEventListener('click', (e) => {
            e.preventDefault();
            stopHoldPulse(null);
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
            startHoldPulse(keyMap[k].cmd, keyMap[k].btn);
        } else if (k === 'x' || k === ' ') {
            e.preventDefault();
            stopHoldPulse(null);
        } else if (k === 'i') {
            e.preventDefault();
            stopHoldPulse(null);
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
            
            if (activeKeys.size === 0) {
                stopHoldPulse(keyMap[k].btn);
            }
        }
    });
});

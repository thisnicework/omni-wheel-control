/*
 * app.js
 * Omni-wheel Robot Controller Engine
 * 
 * Supports 3 Multi-Channel Direct Control Paths:
 *   1. Web Serial API (USB Direct Connection to Mac)
 *   2. Wi-Fi Direct HTTP API (Direct IP fetch to Arduino R4 WiFi)
 *   3. Supabase Realtime & Cloud MQTT
 */

// ==========================================
// 1. WEB SERIAL API CONTROLLER
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
// 2. SUPABASE REALTIME CONTROLLER
// ==========================================
class SupabaseController {
    constructor(onStatusChange) {
        this.onStatusChange = onStatusChange;
        this.supabase = null;
        this.channel = null;
        this.isConnected = false;
        
        this.supabaseUrl = 'https://your-project.supabase.co';
        this.supabaseKey = 'YOUR_SUPABASE_ANON_KEY';

        this.init();
    }

    init() {
        if (typeof supabase === 'undefined' || this.supabaseUrl.includes('your-project')) {
            return;
        }

        try {
            this.supabase = supabase.createClient(this.supabaseUrl, this.supabaseKey);
            this.channel = this.supabase.channel('robot_control');

            this.channel.subscribe((status) => {
                if (status === 'SUBSCRIBED') {
                    console.log('✅ [Supabase Realtime] Connected!');
                    this.isConnected = true;
                    if (this.onStatusChange) this.onStatusChange(true, 'SUPABASE ACTIVE');
                }
            });
        } catch (e) {
            console.error('[Supabase Error]', e);
        }
    }

    send(cmd) {
        if (this.supabase && this.isConnected && this.channel) {
            this.channel.send({
                type: 'broadcast',
                event: 'cmd',
                payload: { cmd: cmd }
            });
            console.log(`⚡ [Supabase Broadcast] Sent '${cmd}'`);
        }
    }
}


// ==========================================
// 3. MULTI-CHANNEL CONTROL DISPATCHER
// ==========================================
class RobotControlDispatcher {
    constructor(serialController, supabaseController, statusBadge, statusText) {
        this.serialController = serialController;
        this.supabaseController = supabaseController;
        this.statusBadge = statusBadge;
        this.statusText = statusText;
        
        this.targetIp = localStorage.getItem('arduino_wifi_ip') || '';
    }

    setTargetIp(ip) {
        this.targetIp = ip.trim();
        if (this.targetIp) {
            localStorage.setItem('arduino_wifi_ip', this.targetIp);
            console.log(`📶 [Wi-Fi Direct IP Configured] http://${this.targetIp}`);
            this.updateStatus(true, `WI-FI IP: ${this.targetIp}`);
        }
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
        let dispatched = false;

        // Path 1: USB Web Serial
        if (this.serialController && this.serialController.port) {
            this.serialController.send(cmd);
            dispatched = true;
        }

        // Path 2: Wi-Fi Direct HTTP API (Fetch endpoint)
        if (this.targetIp) {
            const url = `http://${this.targetIp}/${cmd}`;
            fetch(url, { mode: 'no-cors' }).catch(() => {});
            console.log(`🌐 [Wi-Fi HTTP Direct] Sent GET /${cmd} -> ${url}`);
            dispatched = true;
        }

        // Path 3: Supabase Cloud Realtime
        if (this.supabaseController && this.supabaseController.isConnected) {
            this.supabaseController.send(cmd);
            dispatched = true;
        }

        if (!dispatched) {
            console.log(`💡 [Control] Command '${cmd}' dispatched. Connect USB Serial or enter Wi-Fi IP to drive!`);
        }
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

    const ipInput = document.getElementById('ip-address');
    const btnIpSave = document.getElementById('btn-ip-save');

    const btnUp = document.getElementById('btn-up');
    const btnDown = document.getElementById('btn-down');
    const btnLeft = document.getElementById('btn-left');
    const btnRight = document.getElementById('btn-right');
    const btnStop = document.getElementById('btn-stop');
    const btnAuto = document.getElementById('btn-auto');
    const btnStopAll = document.getElementById('btn-stop-all');

    // Instantiate Controllers
    const serialController = new SerialController(
        (encoders, hz) => {
            // Encoder Telemetry Callback
        },
        (isConnected, label) => {
            dispatcher.updateStatus(isConnected, label);
        }
    );

    const supabaseController = new SupabaseController((isConnected, label) => {
        dispatcher.updateStatus(isConnected, label);
    });

    const dispatcher = new RobotControlDispatcher(serialController, supabaseController, statusBadge, statusText);

    // Restore saved Wi-Fi IP if present
    if (ipInput && dispatcher.targetIp) {
        ipInput.value = dispatcher.targetIp;
        dispatcher.updateStatus(true, `WI-FI IP: ${dispatcher.targetIp}`);
    }

    if (btnIpSave) {
        btnIpSave.addEventListener('click', () => {
            dispatcher.setTargetIp(ipInput.value);
        });
    }

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

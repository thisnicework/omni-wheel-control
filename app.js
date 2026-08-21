/**
 * app.js
 * 
 * Production-level Interactive Media Art Architecture:
 * 1. SerialController: Web Serial API stream parsing (Arduino Uno R4 -> Mac USB CDC)
 * 2. OmniKinematics: 4-Wheel Omni-wheel Inverse Kinematics & Global Odometry
 * 3. SwampRenderer: Three.js WebGL Top-Down Orthographic Shader-driven Swamp Scene
 */

// ==========================================
// 1. OMNI-WHEEL INVERSE KINEMATICS ENGINE
// ==========================================
class OmniKinematics {
    constructor() {
        // Physical Robot Kinematic Parameters (Meters & Radians)
        this.wheelRadius = 0.05;      // r = 50mm wheel radius
        this.robotRadius = 0.20;      // R = 200mm distance from center to wheels
        this.countsPerTurn = 1.0;     // ODrive pos_estimate is directly in float turns

        // Global Absolute Pose (X, Y, Theta)
        this.x = 0.0;                 // Global X (meters)
        this.y = 0.0;                 // Global Y (meters)
        this.theta = 0.0;             // Global Orientation (radians)

        // 20% Scale Centered Workspace Boundary Limits (0.4m N, S, E, W around Origin (0,0))
        // Range: X [-0.4m, +0.4m], Y [-0.4m, +0.4m] (Total 0.8m x 0.8m area)
        this.bounds = {
            minX: -0.4,
            maxX: 0.4,
            minY: -0.4,
            maxY: 0.4
        };
        this.enforceBounds = true;    // Hard/Soft constraint toggle

        // Previous Encoder Values
        this.prevEncoders = null;

        // Raw Deltas
        this.lastDeltas = [0, 0, 0, 0];
    }

    /**
     * Checks if current pose is inside Quadrant 3 bounds
     */
    getBoundaryStatus() {
        const isInsideX = this.x >= this.bounds.minX && this.x <= this.bounds.maxX;
        const isInsideY = this.y >= this.bounds.minY && this.y <= this.bounds.maxY;
        const isNearEdge = (this.x < this.bounds.minX + 0.3 || this.x > this.bounds.maxX - 0.3 ||
                            this.y < this.bounds.minY + 0.3 || this.y > this.bounds.maxY - 0.3);

        return {
            isInside: isInsideX && isInsideY,
            isNearEdge: isNearEdge,
            pctX: ((this.x - this.bounds.minX) / (this.bounds.maxX - this.bounds.minX)) * 100,
            pctY: ((this.y - this.bounds.minY) / (this.bounds.maxY - this.bounds.minY)) * 100
        };
    }

    /**
     * Resets global odometry pose to origin (0, 0, 0)
     */
    resetPose() {
        this.x = 0.0;
        this.y = 0.0;
        this.theta = 0.0;
        this.prevEncoders = null;
        this.lastDeltas = [0, 0, 0, 0];
    }

    /**
     * Updates odometry using 4-wheel Omni-Wheel Inverse Kinematics.
     * Hardware Wheel & Motor Mapping:
     *   - Enc 0 (FL / Q2): Left Front  (Front ODrive axis0)
     *   - Enc 1 (FR / Q1): Right Front (Front ODrive axis1) -> Inverted rotation
     *   - Enc 2 (RL / Q3): Left Rear   (Rear ODrive axis1)
     *   - Enc 3 (RR / Q4): Right Rear  (Rear ODrive axis0) -> Inverted rotation
     * 
     * @param {Array<number>} currentEncoders - [Enc0, Enc1, Enc2, Enc3] positions in turns
     */
    update(currentEncoders) {
        if (!this.prevEncoders) {
            this.prevEncoders = [...currentEncoders];
            return;
        }

        // 1. Calculate wheel linear travel deltas in meters with motor sign alignment
        // Right side motors (FR, RR) are mirrored, so invert their encoder deltas
        const rawDeltas = new Array(4);
        const ds = new Array(4);
        const signCorrection = [1.0, -1.0, 1.0, -1.0]; // [FL, FR, RL, RR]

        for (let i = 0; i < 4; i++) {
            const deltaTurn = currentEncoders[i] - this.prevEncoders[i];
            rawDeltas[i] = deltaTurn;
            // Linear travel per wheel (m) = deltaTurn * (2 * PI * r) * signCorrection
            ds[i] = deltaTurn * (2 * Math.PI * this.wheelRadius) * signCorrection[i];
            this.lastDeltas[i] = deltaTurn;
        }
        this.prevEncoders = [...currentEncoders];

        // 2. Inverse Kinematics Computation for 4-Wheel Omni-wheel layout
        // ds0 = FL, ds1 = FR, ds2 = RL, ds3 = RR
        const invSqrt2 = 1.0 / Math.SQRT2;

        const dxLocal = (invSqrt2 / 2.0) * ( ds[0] - ds[1] - ds[2] + ds[3]); // Lateral (X)
        const dyLocal = (invSqrt2 / 2.0) * ( ds[0] + ds[1] + ds[2] + ds[3]); // Forward (Y)
        const dTheta  = (1.0 / (4.0 * this.robotRadius)) * (-ds[0] + ds[1] - ds[2] + ds[3]); // Rotation (Theta)

        // 3. Transform local displacement to global absolute frame using current theta
        const cosT = Math.cos(this.theta);
        const sinT = Math.sin(this.theta);

        const dxGlobal = dxLocal * cosT - dyLocal * sinT;
        const dyGlobal = dxLocal * sinT + dyLocal * cosT;

        // 4. Integrate pose
        this.x += dxGlobal;
        this.y += dyGlobal;
        this.theta += dTheta;

        // Normalize Theta to range [-PI, PI]
        while (this.theta > Math.PI) this.theta -= 2 * Math.PI;
        while (this.theta < -Math.PI) this.theta += 2 * Math.PI;
    }
}


// ==========================================
// 2. WEB SERIAL COMMUNICATION CONTROLLER
// ==========================================
class SerialController {
    constructor(onDataCallback, onStatusCallback) {
        this.port = null;
        this.reader = null;
        this.keepReading = false;
        this.onDataCallback = onDataCallback;
        this.onStatusCallback = onStatusCallback;

        // Hz Tracker
        this.packetCount = 0;
        this.currentHz = 0;
        this.lastHzCheck = performance.now();

        setInterval(() => this.calculateHz(), 1000);
    }

    calculateHz() {
        const now = performance.now();
        const elapsed = (now - this.lastHzCheck) / 1000.0;
        this.currentHz = Math.round(this.packetCount / elapsed);
        this.packetCount = 0;
        this.lastHzCheck = now;
    }

    async connect() {
        if (!('serial' in navigator)) {
            alert('Web Serial API is not supported in this browser. Please use Chrome on Mac.');
            return;
        }

        try {
            // Request user to select Arduino Serial Port
            this.port = await navigator.serial.requestPort();
            await this.port.open({ baudRate: 115200 });

            this.keepReading = true;
            this.onStatusCallback(true);
            this.readLoop();
        } catch (error) {
            console.error('Serial Connection Error:', error);
            this.onStatusCallback(false, error.message);
        }
    }

    async disconnect() {
        this.keepReading = false;
        if (this.reader) {
            await this.reader.cancel();
        }
        if (this.port) {
            await this.port.close();
        }
        this.port = null;
        this.onStatusCallback(false);
    }

    async send(data) {
        if (this.port && this.port.writable) {
            try {
                const encoder = new TextEncoder();
                const writer = this.port.writable.getWriter();
                await writer.write(encoder.encode(data));
                writer.releaseLock();
            } catch (err) {
                console.error('Serial Send Error:', err);
            }
        }
    }

    async readLoop() {
        const textDecoder = new TextDecoderStream();
        const readableStreamClosed = this.port.readable.pipeTo(textDecoder.writable);
        this.reader = textDecoder.readable.getReader();

        let lineBuffer = '';

        try {
            while (this.keepReading) {
                const { value, done } = await this.reader.read();
                if (done) break;
                if (value) {
                    lineBuffer += value;
                    const lines = lineBuffer.split('\n');
                    // Retain incomplete trailing fragment
                    lineBuffer = lines.pop();

                    for (const line of lines) {
                        const trimmed = line.trim();
                        if (trimmed.length > 0) {
                            this.parseCSVLine(trimmed);
                        }
                    }
                }
            }
        } catch (err) {
            console.error('Serial Read Error:', err);
        } finally {
            this.reader.releaseLock();
        }
    }

    parseCSVLine(line) {
        // Parse CSV format: "Enc0,Enc1,Enc2,Enc3"
        const parts = line.split(',');
        if (parts.length >= 4) {
            const encoders = parts.map(p => parseFloat(p));
            if (encoders.every(val => !isNaN(val))) {
                this.packetCount++;
                this.onDataCallback(encoders, this.currentHz);
            }
        }
    }
}


// ==========================================
// 3. THREE.JS TOP-DOWN SWAMP VISUALIZER
// ==========================================
class SwampRenderer {
    constructor(canvas) {
        this.canvas = canvas;
        this.clock = new THREE.Clock();

        // 1. Scene setup
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x060f0d);

        // 2. Orthographic Camera Setup (Top-Down parallel to ground plane)
        this.frustumSize = 8.0; // 8 meters view field
        const aspect = window.innerWidth / window.innerHeight;
        this.camera = new THREE.OrthographicCamera(
            -this.frustumSize * aspect / 2,
             this.frustumSize * aspect / 2,
             this.frustumSize / 2,
            -this.frustumSize / 2,
            0.1,
            100
        );
        // Position camera directly above origin looking down -Z onto XY ground plane
        this.camera.position.set(0, 0, 10);
        this.camera.lookAt(0, 0, 0);

        // 3. WebGL Renderer
        this.renderer = new THREE.WebGLRenderer({
            canvas: this.canvas,
            antialias: true,
            powerPreference: "high-performance"
        });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

        // 4. Build Procedural Realistic Swamp Visuals
        this.initSwampMaterials();
        this.buildSwampWorld();

        // Window Resize Listener
        window.addEventListener('resize', () => this.onWindowResize());
    }

    initSwampMaterials() {
        // Create dynamic offscreen canvas for procedural organic swamp ground texture
        const procCanvas = document.createElement('canvas');
        procCanvas.width = 1024;
        procCanvas.height = 1024;
        const ctx = procCanvas.getContext('2d');

        // Dark murky swamp water base
        ctx.fillStyle = '#061614';
        ctx.fillRect(0, 0, 1024, 1024);

        // Generate organic moss and mud noise patches
        for (let i = 0; i < 400; i++) {
            const x = Math.random() * 1024;
            const y = Math.random() * 1024;
            const r = 20 + Math.random() * 80;
            const grad = ctx.createRadialGradient(x, y, 0, x, y, r);
            const isMoss = Math.random() > 0.4;
            if (isMoss) {
                grad.addColorStop(0, 'rgba(16, 60, 42, 0.7)');
                grad.addColorStop(1, 'rgba(6, 22, 20, 0)');
            } else {
                grad.addColorStop(0, 'rgba(40, 30, 18, 0.6)');
                grad.addColorStop(1, 'rgba(6, 22, 20, 0)');
            }
            ctx.fillStyle = grad;
            ctx.beginPath();
            ctx.arc(x, y, r, 0, Math.PI * 2);
            ctx.fill();
        }

        const swampTex = new THREE.CanvasTexture(procCanvas);
        swampTex.wrapS = THREE.RepeatWrapping;
        swampTex.wrapT = THREE.RepeatWrapping;
        swampTex.repeat.set(4, 4);

        // Custom GLSL Shader for Animated Murky Swamp Water & Caustics
        this.swampUniforms = {
            uTime: { value: 0 },
            uTexture: { value: swampTex },
            uResolution: { value: new THREE.Vector2(window.innerWidth, window.innerHeight) }
        };

        const vertexShader = `
            varying vec2 vUv;
            varying vec3 vWorldPosition;
            void main() {
                vUv = uv;
                vec4 worldPos = modelMatrix * vec4(position, 1.0);
                vWorldPosition = worldPos.xyz;
                gl_Position = projectionMatrix * viewMatrix * worldPos;
            }
        `;

        const fragmentShader = `
            uniform float uTime;
            uniform sampler2D uTexture;
            varying vec2 vUv;
            varying vec3 vWorldPosition;

            // Simplex-style pseudo noise for fluid caustics
            float hash(vec2 p) {
                p = fract(p * vec2(123.34, 456.21));
                p += dot(p, p + 45.32);
                return fract(p.x * p.y);
            }

            float noise(vec2 p) {
                vec2 i = floor(p);
                vec2 f = fract(p);
                f = f * f * (3.0 - 2.0 * f);
                float a = hash(i);
                float b = hash(i + vec2(1.0, 0.0));
                float c = hash(i + vec2(0.0, 1.0));
                float d = hash(i + vec2(1.0, 1.0));
                return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
            }

            void main() {
                // UV offset driven by absolute world position for seamless infinite scrolling
                vec2 worldUv = vWorldPosition.xy * 0.25;

                // Animated fluid ripples
                float wave1 = sin(worldUv.x * 4.0 + worldUv.y * 3.0 + uTime * 1.2);
                float wave2 = cos(worldUv.x * 3.0 - worldUv.y * 5.0 + uTime * 0.9);
                float n = noise(worldUv * 6.0 + vec2(uTime * 0.2, uTime * 0.15));

                vec2 distortedUv = worldUv + vec2(wave1, wave2) * 0.015 + vec2(n) * 0.01;

                // Base swamp texture sample
                vec4 baseColor = texture2D(uTexture, distortedUv);

                // Deep murky water color palette
                vec3 waterDeep = vec3(0.02, 0.08, 0.07);
                vec3 emeraldShine = vec3(0.06, 0.35, 0.22);
                vec3 specularHighlight = vec3(0.4, 0.7, 0.6);

                // Specular reflection sheen
                float sheen = pow(n, 3.0) * 0.35 + wave1 * 0.05;
                vec3 finalColor = mix(baseColor.rgb, waterDeep, 0.3) + emeraldShine * sheen;
                finalColor += specularHighlight * pow(clamp(wave1 + wave2, 0.0, 1.0), 4.0) * 0.2;

                gl_FragColor = vec4(finalColor, 1.0);
            }
        `;

        this.swampMaterial = new THREE.ShaderMaterial({
            uniforms: this.swampUniforms,
            vertexShader: vertexShader,
            fragmentShader: fragmentShader,
            side: THREE.DoubleSide
        });
    }

    buildSwampWorld() {
        // Ground Plane
        const planeGeo = new THREE.PlaneGeometry(100, 100);
        this.groundMesh = new THREE.Mesh(planeGeo, this.swampMaterial);
        this.scene.add(this.groundMesh);

        // --- 20% SCALE WORKSPACE BOUNDARY FRAME (0.4m N, S, E, W around (0,0)) ---
        // Bounds: X [-0.4, +0.4], Y [-0.4, +0.4]
        const boundaryGeo = new THREE.BufferGeometry();
        const vertices = new Float32Array([
             0.4,  0.4, 0.02,   -0.4,  0.4, 0.02,  // Top edge (Y=+0.4)
            -0.4,  0.4, 0.02,   -0.4, -0.4, 0.02,  // Left edge (X=-0.4)
            -0.4, -0.4, 0.02,    0.4, -0.4, 0.02,  // Bottom edge (Y=-0.4)
             0.4, -0.4, 0.02,    0.4,  0.4, 0.02   // Right edge (X=+0.4)
        ]);
        boundaryGeo.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
        const boundaryMat = new THREE.LineBasicMaterial({ color: 0x10b981, linewidth: 3 });
        const boundaryLines = new THREE.LineSegments(boundaryGeo, boundaryMat);
        this.scene.add(boundaryLines);

        // Center Origin Marker (0,0)
        const originGeo = new THREE.RingGeometry(0.06, 0.09, 32);
        const originMat = new THREE.MeshBasicMaterial({ color: 0x06b6d4, side: THREE.DoubleSide });
        const originMarker = new THREE.Mesh(originGeo, originMat);
        originMarker.position.set(0, 0, 0.03);
        this.scene.add(originMarker);

        // Corner Markers at (0.4, 0.4), (-0.4, 0.4), (-0.4, -0.4), (0.4, -0.4)
        const cornerGeo = new THREE.CircleGeometry(0.04, 16);
        const cornerMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b, side: THREE.DoubleSide });
        [[0.4, 0.4], [-0.4, 0.4], [-0.4, -0.4], [0.4, -0.4]].forEach(([cx, cy]) => {
            const corner = new THREE.Mesh(cornerGeo, cornerMat);
            corner.position.set(cx, cy, 0.03);
            this.scene.add(corner);
        });

        // Add procedural floating swamp elements centered inside [-2.0, +2.0] area
        this.decorGroup = new THREE.Group();
        this.scene.add(this.decorGroup);

        const lilypadGeo = new THREE.CircleGeometry(0.25, 16, 0, Math.PI * 1.75);
        const lilypadMat = new THREE.MeshBasicMaterial({ color: 0x15803d, side: THREE.DoubleSide });

        // Scatter 50 procedural lilypads inside [-2.0, +2.0] x [-2.0, +2.0]
        for (let i = 0; i < 50; i++) {
            const mesh = new THREE.Mesh(lilypadGeo, lilypadMat);
            mesh.position.set(
                (Math.random() - 0.5) * 4.0,
                (Math.random() - 0.5) * 4.0,
                0.01
            );
            mesh.rotation.z = Math.random() * Math.PI * 2;
            const scale = 0.5 + Math.random() * 0.7;
            mesh.scale.set(scale, scale, 1);
            this.decorGroup.add(mesh);
        }

        // Submerged log elements inside [-2.0, +2.0]
        const logGeo = new THREE.CylinderGeometry(0.08, 0.1, 1.8, 8);
        const logMat = new THREE.MeshBasicMaterial({ color: 0x271c14 });
        for (let i = 0; i < 12; i++) {
            const log = new THREE.Mesh(logGeo, logMat);
            log.position.set(
                (Math.random() - 0.5) * 4.0,
                (Math.random() - 0.5) * 4.0,
                0.005
            );
            log.rotation.z = Math.random() * Math.PI;
            this.decorGroup.add(log);
        }

        // --- BIOLUMINESCENT SWAMP ORGANISMS (FLOATING FIREFLIES / PLANKTON) ---
        const particleCount = 80;
        const fireflyGeo = new THREE.BufferGeometry();
        const fireflyPositions = new Float32Array(particleCount * 3);
        const fireflySpeeds = new Float32Array(particleCount);

        for (let i = 0; i < particleCount; i++) {
            fireflyPositions[i * 3 + 0] = (Math.random() - 0.5) * 4.0;
            fireflyPositions[i * 3 + 1] = (Math.random() - 0.5) * 4.0;
            fireflyPositions[i * 3 + 2] = 0.05 + Math.random() * 0.4;
            fireflySpeeds[i] = 0.5 + Math.random() * 1.5;
        }

        fireflyGeo.setAttribute('position', new THREE.BufferAttribute(fireflyPositions, 3));
        const fireflyMat = new THREE.PointsMaterial({
            color: 0x34d399,
            size: 0.08,
            transparent: true,
            opacity: 0.85,
            blending: THREE.AdditiveBlending
        });
        this.fireflies = new THREE.Points(fireflyGeo, fireflyMat);
        this.fireflySpeeds = fireflySpeeds;
        this.scene.add(this.fireflies);
    }

    /**
     * Synchronizes Top-Down Orthographic Camera with Robot Odometry.
     * @param {number} x - Global X position (meters)
     * @param {number} y - Global Y position (meters)
     * @param {number} theta - Global Heading angle (radians)
     */
    updatePose(x, y, theta) {
        // 1. Move camera to match robot's absolute global position
        this.camera.position.x = x;
        this.camera.position.y = y;

        // 2. Rotate Orthographic Camera Z-axis to match robot orientation Theta
        this.camera.rotation.z = theta;
    }

    render() {
        const delta = this.clock.getDelta();
        this.swampUniforms.uTime.value += delta;

        this.renderer.render(this.scene, this.camera);
    }

    onWindowResize() {
        const aspect = window.innerWidth / window.innerHeight;
        this.camera.left = -this.frustumSize * aspect / 2;
        this.camera.right = this.frustumSize * aspect / 2;
        this.camera.top = this.frustumSize / 2;
        this.camera.bottom = -this.frustumSize / 2;
        this.camera.updateProjectionMatrix();

        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.swampUniforms.uResolution.value.set(window.innerWidth, window.innerHeight);
    }
}


// ==========================================
// 3.5 SUPABASE CLOUD IOT CONTROLLER
// ==========================================
class SupabaseController {
    constructor(kinematics, onStatusChange) {
        this.kinematics = kinematics;
        this.onStatusChange = onStatusChange;
        this.supabase = null;
        this.channel = null;
        this.isConnected = false;
        
        // Enter your Supabase Project URL & Anon Key
        this.supabaseUrl = 'https://your-project.supabase.co';
        this.supabaseKey = 'YOUR_SUPABASE_ANON_KEY';

        this.init();
    }

    init() {
        if (typeof supabase === 'undefined') {
            console.warn('Supabase JS library not loaded');
            return;
        }
        if (this.supabaseUrl.includes('your-project')) {
            console.log('⚡ [Supabase] Ready for configuration (Enter URL & Anon Key in app.js / ino)');
            return;
        }

        try {
            this.supabase = supabase.createClient(this.supabaseUrl, this.supabaseKey);
            this.channel = this.supabase.channel('robot_control');

            this.channel.on('broadcast', { event: 'telemetry' }, (payload) => {
                if (payload && payload.encoders) {
                    this.kinematics.updateFromEncoders(payload.encoders);
                }
            });

            this.channel.subscribe((status) => {
                if (status === 'SUBSCRIBED') {
                    console.log('✅ [Supabase Realtime] Connected!');
                    this.isConnected = true;
                    if (this.onStatusChange) this.onStatusChange(true, 'SUPABASE ACTIVE');
                }
            });
        } catch (e) {
            console.error('[Supabase Exception]', e);
        }
    }

    send(cmd) {
        if (this.supabase && this.isConnected && this.channel) {
            this.channel.send({
                type: 'broadcast',
                event: 'cmd',
                payload: { cmd: cmd }
            });
            console.log(`⚡ [Supabase Realtime] Broadcasted cmd: '${cmd}'`);
        }
    }
}


// ==========================================
// 4. MAIN APPLICATION BOOTSTRAP
// ==========================================
document.addEventListener('DOMContentLoaded', () => {
    // 1. DOM Element Handles
    const canvas = document.getElementById('webgl-canvas');
    const btnConnect = document.getElementById('btn-connect');
    const btnReset = document.getElementById('btn-reset');
    const statusBadge = document.getElementById('status-badge');
    const statusText = document.getElementById('status-text');

    const valX = document.getElementById('val-x');
    const valY = document.getElementById('val-y');
    const valTheta = document.getElementById('val-theta');
    const valFps = document.getElementById('val-fps');

    const encElems = [
        document.getElementById('enc-0'),
        document.getElementById('enc-1'),
        document.getElementById('enc-2'),
        document.getElementById('enc-3')
    ];

    const boundStatus = document.getElementById('bound-status');

    // 2. Instantiate Core Systems
    const kinematics = new OmniKinematics();
    const swampRenderer = new SwampRenderer(canvas);

    // 3. Web Serial Controller Setup
    const serialController = new SerialController(
        // On Data Callback (runs on incoming encoder packets)
        (encoders, hz) => {
            // Update kinematics
            kinematics.update(encoders);

            // Update HUD
            valX.textContent = kinematics.x.toFixed(2) + 'm';
            valY.textContent = kinematics.y.toFixed(2) + 'm';
            const thetaDeg = (kinematics.theta * (180.0 / Math.PI)).toFixed(1);
            valTheta.textContent = thetaDeg + '°';

            // Boundary tracking
            const bStatus = kinematics.getBoundaryStatus();
            if (boundStatus) {
                if (!bStatus.isInside) {
                    boundStatus.textContent = 'OUT OF BOUNDS';
                    boundStatus.style.color = '#ef4444';
                } else if (bStatus.isNearEdge) {
                    boundStatus.textContent = 'NEAR EDGE';
                    boundStatus.style.color = '#f59e0b';
                } else {
                    boundStatus.textContent = 'INSIDE';
                    boundStatus.style.color = '#06b6d4';
                }
            }

            for (let i = 0; i < 4; i++) {
                if (encElems[i]) {
                    encElems[i].textContent = encoders[i].toFixed(2);
                }
            }
            valFps.textContent = `${hz} Hz`;
        },
        // On Status Callback
        (isConnected, errorMsg) => {
            if (isConnected) {
                statusBadge.classList.add('connected');
                statusText.textContent = 'CONNECTED (50Hz)';
                btnConnect.innerHTML = `
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                        <path d="M18 6L6 18M6 6l12 12"></path>
                    </svg>
                    Disconnect
                `;
            } else {
                statusBadge.classList.remove('connected');
                statusText.textContent = errorMsg ? 'ERROR' : 'DISCONNECTED';
                btnConnect.innerHTML = `
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                        <path d="M5 12h14M12 5l7 7-7 7"></path>
                    </svg>
                    Connect Serial
                `;
            }
        }
    );

    // 4. UI Button Event Listeners
    btnConnect.addEventListener('click', async () => {
        if (serialController.port) {
            await serialController.disconnect();
        } else {
            await serialController.connect();
        }
    });

    btnReset.addEventListener('click', () => {
        kinematics.resetPose();
        valX.textContent = '0.00m';
        valY.textContent = '0.00m';
        valTheta.textContent = '0.0°';
    });

    // 3.5 Supabase Cloud Controller Setup
    const supabaseCtrl = new SupabaseController(kinematics, (isConnected, label) => {
        if (isConnected) {
            statusBadge.classList.add('connected');
            statusText.textContent = label;
        }
    });

    // 5. Interactive D-Pad On-Screen Control Buttons
    const btnUp = document.getElementById('btn-up');
    const btnDown = document.getElementById('btn-down');
    const btnLeft = document.getElementById('btn-left');
    const btnRight = document.getElementById('btn-right');
    const btnStop = document.getElementById('btn-stop');
    const btnAuto = document.getElementById('btn-auto');
    const btnStopAll = document.getElementById('btn-stop-all');

    const sendCmd = (cmd, btn = null) => {
        serialController.send(cmd);
        supabaseCtrl.send(cmd);
        console.log(`[Web Control] Sent '${cmd}' via Web Serial & Supabase Cloud`);
        if (btn) {
            btn.classList.add('active');
            setTimeout(() => btn.classList.remove('active'), 150);
        }
    };

    if (btnUp) btnUp.addEventListener('click', () => sendCmd('w', btnUp));
    if (btnDown) btnDown.addEventListener('click', () => sendCmd('s', btnDown));
    if (btnLeft) btnLeft.addEventListener('click', () => sendCmd('a', btnLeft));
    if (btnRight) btnRight.addEventListener('click', () => sendCmd('d', btnRight));
    if (btnStop) btnStop.addEventListener('click', () => sendCmd('x', btnStop));
    if (btnAuto) btnAuto.addEventListener('click', () => sendCmd('i', btnAuto));
    if (btnStopAll) btnStopAll.addEventListener('click', () => sendCmd('o', btnStopAll));

    // 6. Interactive Keyboard Controller (WASD, Arrow Keys, I, O, X, Space)
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
            sendCmd(keyMap[k].cmd, keyMap[k].btn);
        }
    });

    // 7. Main WebGL Render Loop (60 FPS)
    function animate() {
        requestAnimationFrame(animate);

        // Synchronize Orthographic Camera with Kinematics
        swampRenderer.updatePose(kinematics.x, kinematics.y, kinematics.theta);

        // Render WebGL Swamp Scene
        swampRenderer.render();
    }

    animate();
});

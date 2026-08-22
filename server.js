/*
 * server.js
 * Mac Local Web & API Relay Server (Simple & Reliable HTTP Only)
 * 
 * Target IP: 192.168.10.140:8000
 * Purpose: Relays WASD control commands from Web UI to Arduino via HTTP poll.
 * 
 * How it works:
 *   1. Browser sends button press via /api/cmd?set=w
 *   2. Arduino polls /api/poll every ~100ms and receives the latest command
 *   3. No UDP, no MQTT, no cloud. Pure local HTTP. Simple and reliable.
 */

const http = require('http');
const fs = require('fs');
const path = require('path');
const url = require('url');

const PORT = 8000;
let latestCommand = 'x';
let lastCommandTime = Date.now();
let arduinoIp = '(unknown)';
let arduinoLastSeen = 0;

// Auto-reset command to 'x' if no new command received in 500ms (Hold-to-drive safety)
setInterval(() => {
    if (latestCommand !== 'x' && (Date.now() - lastCommandTime > 500)) {
        latestCommand = 'x';
    }
}, 50);

// Status log every 10 seconds
setInterval(() => {
    const ago = arduinoLastSeen ? Math.round((Date.now() - arduinoLastSeen) / 1000) : '?';
    console.log(`📊 [Status] cmd='${latestCommand}' | Arduino IP=${arduinoIp} (last seen ${ago}s ago)`);
}, 10000);

const server = http.createServer((req, res) => {
    const parsedUrl = url.parse(req.url, true);
    const pathname = parsedUrl.pathname;

    // Enable CORS
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type');

    if (req.method === 'OPTIONS') {
        res.writeHead(200);
        res.end();
        return;
    }

    // 1. Arduino Polling API Endpoint: GET /api/poll
    if (pathname === '/api/poll') {
        // Auto-detect Arduino IP from incoming connection
        const rawIp = req.socket.remoteAddress || '';
        const clientIp = rawIp.replace(/^.*:/, ''); // Strip IPv6 prefix
        if (clientIp && clientIp !== '127.0.0.1' && clientIp !== '192.168.10.140') {
            if (arduinoIp !== clientIp) {
                console.log(`🔍 [Auto-Detect] Arduino IP updated: ${arduinoIp} -> ${clientIp}`);
            }
            arduinoIp = clientIp;
            arduinoLastSeen = Date.now();
        }
        
        res.writeHead(200, { 'Content-Type': 'text/plain', 'Connection': 'close' });
        res.end(latestCommand);
        return;
    }

    // 2. Web UI Command Endpoint: GET/POST /api/cmd?set=w
    if (pathname === '/api/cmd') {
        const cmd = parsedUrl.query.set || 'x';
        latestCommand = cmd.toLowerCase().charAt(0);
        lastCommandTime = Date.now();
        console.log(`⚡ [Web UI] Command: '${latestCommand}'`);
        res.writeHead(200, { 'Content-Type': 'text/plain', 'Connection': 'close' });
        res.end('OK');
        return;
    }

    // 3. Status endpoint
    if (pathname === '/api/status') {
        const status = JSON.stringify({
            command: latestCommand,
            arduinoIp: arduinoIp,
            arduinoLastSeen: arduinoLastSeen ? new Date(arduinoLastSeen).toISOString() : null,
            uptime: process.uptime()
        });
        res.writeHead(200, { 'Content-Type': 'application/json', 'Connection': 'close' });
        res.end(status);
        return;
    }

    // 4. Static File Server (index.html, app.js, css)
    let filePath = path.join(__dirname, pathname === '/' ? 'index.html' : pathname);
    const extname = path.extname(filePath);

    let contentType = 'text/html';
    if (extname === '.js') contentType = 'text/javascript';
    if (extname === '.css') contentType = 'text/css';
    if (extname === '.json') contentType = 'application/json';

    fs.readFile(filePath, (err, content) => {
        if (err) {
            if (err.code === 'ENOENT') {
                res.writeHead(404, { 'Content-Type': 'text/plain' });
                res.end('404 Not Found');
            } else {
                res.writeHead(500, { 'Content-Type': 'text/plain' });
                res.end('500 Server Error');
            }
        } else {
            res.writeHead(200, { 'Content-Type': contentType });
            res.end(content, 'utf-8');
        }
    });
});

server.listen(PORT, '0.0.0.0', () => {
    console.log(`==================================================`);
    console.log(`🚀 Mac Local Relay Server on port ${PORT}`);
    console.log(`🌐 Web UI: http://192.168.10.140:${PORT}`);
    console.log(`📡 Arduino Poll: http://192.168.10.140:${PORT}/api/poll`);
    console.log(`📊 Status: http://192.168.10.140:${PORT}/api/status`);
    console.log(`==================================================`);
});

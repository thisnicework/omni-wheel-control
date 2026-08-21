/*
 * server.js
 * Mac Local Web & API Relay Server
 * 
 * Target IP: 192.168.10.140:8000
 * Purpose: Relays WASD control commands from Web UI to Arduino Uno R4 WiFi over local Wi-Fi.
 */

const http = require('http');
const fs = require('fs');
const path = require('path');
const url = require('url');

const PORT = 8000;
let latestCommand = 'x';
let lastCommandTime = Date.now();

// Auto-reset command to 'x' if no new command received in 500ms (Hold-to-drive safety)
setInterval(() => {
    if (latestCommand !== 'x' && (Date.now() - lastCommandTime > 500)) {
        latestCommand = 'x';
    }
}, 50);

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
        res.writeHead(200, { 'Content-Type': 'text/plain', 'Connection': 'close' });
        res.end(latestCommand);
        return;
    }

    // 2. Web UI Command Endpoint: GET/POST /api/cmd?set=w
    if (pathname === '/api/cmd') {
        const cmd = parsedUrl.query.set || 'x';
        latestCommand = cmd.toLowerCase().charAt(0);
        lastCommandTime = Date.now();
        console.log(`⚡ [Mac Server] Command updated: '${latestCommand}'`);
        res.writeHead(200, { 'Content-Type': 'text/plain', 'Connection': 'close' });
        res.end('OK');
        return;
    }

    // 3. Static File Server (index.html, app.js, css)
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
    console.log(`🚀 Mac Local Relay Server running on port ${PORT}`);
    console.log(`🌐 Local Web UI URL : http://192.168.10.140:${PORT}`);
    console.log(`📡 Arduino Poll URL : http://192.168.10.140:${PORT}/api/poll`);
    console.log(`==================================================`);
});

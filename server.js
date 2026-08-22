/*
 * server.js
 * Mac Local Web & Blazing-Fast UDP Relay Server
 * 
 * Target IP: 192.168.10.140:8000 (Web) / UDP Port 8888 (1ms Low-Latency Direct UDP)
 * Purpose: Relays WASD control commands from Web UI to Arduino Uno R4 WiFi over local Wi-Fi with <2ms latency.
 */

const http = require('http');
const fs = require('fs');
const path = require('path');
const url = require('url');
const dgram = require('dgram');

const PORT = 8000;
const UDP_PORT = 8888;

let latestCommand = 'x';
let lastCommandTime = Date.now();
let arduinoIp = '192.168.10.156'; // Default/Target Arduino IP

// Create UDP Socket for 1ms Ultra Low Latency Control
const udpSocket = dgram.createSocket('udp4');
udpSocket.bind(UDP_PORT, () => {
    console.log(`⚡ UDP Direct Socket listening on port ${UDP_PORT}`);
});

udpSocket.on('message', (msg, rinfo) => {
    // Dynamically update Arduino IP from heartbeat/UDP packet
    arduinoIp = rinfo.address;
});

// Direct UDP Sender Function (<2ms latency)
function sendUdpCommand(cmd) {
    if (!arduinoIp) return;
    const buf = Buffer.from(cmd);
    udpSocket.send(buf, 0, buf.length, UDP_PORT, arduinoIp, (err) => {
        if (err) console.error('⚠️ UDP Send Error:', err.message);
    });
}

// Auto-reset command to 'x' if no new command received in 500ms (Hold-to-drive safety)
setInterval(() => {
    if (latestCommand !== 'x' && (Date.now() - lastCommandTime > 500)) {
        latestCommand = 'x';
        sendUdpCommand('x');
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
        const clientIp = req.socket.remoteAddress ? req.socket.remoteAddress.replace(/^.*:/, '') : null;
        if (clientIp && clientIp !== '127.0.0.1') {
            arduinoIp = clientIp;
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

        // Send INSTANT UDP Packet (<2ms Latency) to Arduino
        sendUdpCommand(latestCommand);

        console.log(`⚡ [Mac UDP Relay] Command sent -> '${latestCommand}' to ${arduinoIp}:${UDP_PORT}`);
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
    console.log(`🚀 Mac Local Ultra Low Latency Server on port ${PORT}`);
    console.log(`🌐 Local Web UI URL : http://192.168.10.140:${PORT}`);
    console.log(`⚡ Direct UDP Port   : ${UDP_PORT} (<2ms Latency)`);
    console.log(`==================================================`);
});

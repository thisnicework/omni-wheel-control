const express = require('express');
const path = require('path');
const cors = require('cors');

const app = express();
const PORT = process.env.PORT || 3000;

app.use(cors());
app.use(express.json());
app.use(express.static(path.join(__dirname, './')));

// In-Memory Command & Telemetry State
let currentCommand = 'x'; // Default stop 'x'
let telemetry = [0, 0, 0, 0];
let lastCmdTime = Date.now();

// --- RENDER CLOUD REST API ENDPOINTS ---

// GET /api/cmd (Returns current command JSON)
// Or GET /api/cmd?set=w (Sets new command from website)
app.get('/api/cmd', (req, res) => {
    if (req.query.set) {
        currentCommand = req.query.set.toLowerCase();
        lastCmdTime = Date.now();
        console.log(`⚡ [Render Cloud API] Command Set -> '${currentCommand}'`);
    }
    res.json({ cmd: currentCommand, time: lastCmdTime });
});

// POST /api/cmd
app.post('/api/cmd', (req, res) => {
    if (req.body && req.body.cmd) {
        currentCommand = req.body.cmd.toLowerCase();
        lastCmdTime = Date.now();
        console.log(`⚡ [Render Cloud API] Command Set via POST -> '${currentCommand}'`);
    }
    res.json({ status: 'ok', cmd: currentCommand });
});

// Telemetry Endpoints
app.post('/api/telemetry', (req, res) => {
    if (req.body && req.body.encoders) {
        telemetry = req.body.encoders;
    }
    res.json({ status: 'ok' });
});

app.get('/api/telemetry', (req, res) => {
    res.json({ encoders: telemetry });
});

// Serve index.html for all other routes
app.get('*', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

app.listen(PORT, () => {
    console.log(`🌐 Omni-Wheel Control Server active on port ${PORT}`);
});

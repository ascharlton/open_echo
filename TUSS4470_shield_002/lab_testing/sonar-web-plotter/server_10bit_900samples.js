const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const { DelimiterParser } = require('@serialport/parser-delimiter');

// --- Configuration ---
const PORT = 3000;
const SERIAL_PORT = '/dev/ttyACM1'; // Your hardcoded port
const BAUD_RATE = 250000;
const NUM_SAMPLES = 900;

// PACKET_SIZE: 1 (Header) + 6 (Metadata) + 2 * NUM_SAMPLES (Samples) + 1 (Checksum) = 1808 bytes
const PACKET_SIZE = 1 + 6 + 2 * NUM_SAMPLES + 1; 

// --- Web Server Setup ---
const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

// Serve static files (index.html, client-side JS)
app.use(express.static('public'));

server.listen(PORT, () => {
    console.log(`\n🚀 Server listening on http://<Pi-IP-Address>:${PORT}`);
    console.log(`WebSocket server started on port ${PORT}.`);
});

// --- Serial Port Setup ---
try {
    const port = new SerialPort({ 
        path: SERIAL_PORT, 
        baudRate: BAUD_RATE,
    });

    let buffer = Buffer.alloc(0); // Initialize an empty buffer to accumulate fragments

    port.on('data', (data) => {
        // 1. Concatenate the new data chunk to the accumulated buffer
        buffer = Buffer.concat([buffer, data]);

        while (buffer.length >= PACKET_SIZE) {
            
            const headerIndex = buffer.indexOf(0xAA);
            
            if (headerIndex === -1) {
                // No header found in buffer, discard it all
                buffer = Buffer.alloc(0);
                console.warn(`[WARNING] Dropping`);
                break;
            }

            if (headerIndex > 0) {
                console.warn(`[WARNING] Discarding ${headerIndex} junk bytes before header.`);
                // Slice off junk bytes up to the header
                buffer = buffer.slice(headerIndex);
                if (buffer.length < PACKET_SIZE) break; 
            }

            // We know the buffer now starts with 0xAA and is at least PACKET_SIZE long
            const packet = buffer.slice(0, PACKET_SIZE);
            
            // --- Data Processing ---
            // The payload is everything between the start byte (1) and the checksum (1)
            const payload = packet.slice(1, PACKET_SIZE - 1);
            
            // 1. Unpack Metadata (6 bytes) using BIG ENDIAN (BE)
            const depthIndex = payload.readUInt16BE(0); 
            const tempScaled = payload.readInt16BE(2);   
            const vDrvScaled = payload.readUInt16BE(4);  
            
            // 2. Unpack Samples (3600 bytes)
            const samplesBuffer = payload.slice(6);
            const samples = [];
            for (let i = 0; i < NUM_SAMPLES; i++) {
                // Read 16-bit (2-byte) sample data for 10-bit resolution
                samples.push(samplesBuffer.readUInt16BE(i * 2)); 
            }

            // 3. Calculate Real Values and Emit
            const SPEED_OF_SOUND = 330;
            const SAMPLE_TIME = 13.2e-6;
            const SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;

            const depthCm = depthIndex * SAMPLE_RESOLUTION;
            const temperature = tempScaled / 100.0;
            const driveVoltage = vDrvScaled / 100.0;
            
            const dataToSend = {
                samples: samples,
                depth: depthCm.toFixed(1),
                temperature: temperature.toFixed(1),
                driveVoltage: driveVoltage.toFixed(1),
                resolution: 10
            };
            
            // 4. Push data to all connected web clients via WebSocket
            wss.clients.forEach(client => {
                if (client.readyState === WebSocket.OPEN) {
                    client.send(JSON.stringify(dataToSend));
                }
            });

            // 5. Slice buffer: Move past the packet we just processed
            buffer = buffer.slice(PACKET_SIZE);                    
        }
    });


} catch (e) {
    console.error(`❌ Fatal Error: Could not initialize serial port: ${e.message}`);
    console.log(`Please check if port ${SERIAL_PORT} is correct.`);
}

// server.js

// tracks raw signal only coming in on serial port and identify higest consostent signal to track and send to grapg front end

const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const { DelimiterParser } = require('@serialport/parser-delimiter');

// --- Configuration ---
const PORT = 3000;
const SERIAL_PORT = '/dev/ttyACM1'; // Your hardcoded port
const BAUD_RATE = 250000;
const NUM_SAMPLES = 1800;
const PACKET_SIZE = 1 + 6 + 2 * NUM_SAMPLES + 1; // Expected total size: 3608 bytes
console.log("expecting packet size:", PACKET_SIZE, "bytes");

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
        // The serialport library handles the data stream inherently non-blocking
    });

    // The DelimiterParser isn't needed here as we rely on the header and fixed size,
    // but using a stream handler is cleaner than managing raw buffer reads with timeouts.
    let buffer = Buffer.alloc(0); // Initialize an empty buffer to accumulate fragments

    port.on('data', (data) => {
        // 1. Concatenate the new data chunk to the accumulated buffer until buffer.length > 3608
        buffer = Buffer.concat([buffer, data]);

        while (buffer.length >= PACKET_SIZE) {
            
            // 2. Search for the start marker (0xAA or 170)
            const headerIndex = buffer.indexOf(0xAA);
            
            if (headerIndex === -1) {
                // No header found in the buffer. Assume all data is junk (or a massive tail fragment) 
                // and discard it to prevent memory overflow. Break the loop.
                console.warn(`[WARNING] Discarding ${buffer.length} junk bytes without any header.`);
                buffer = Buffer.alloc(0);
                break;
            }

            if (headerIndex > 0) {
                // Junk found before the header. Discard everything up to the header.
                // Console log this, as it indicates corruption/missed data.
                console.warn(`[WARNING] Discarding ${headerIndex} junk bytes before header.`);
                buffer = buffer.slice(headerIndex);
                
                // Check again if we have a full packet starting at the new index 0
                if (buffer.length < PACKET_SIZE) {
                    console.warn(`[WARNING] Header found but not enough bytes collected.`);
                    break; // Not enough data for a full packet after cleanup, wait for next chunk.
                }
            }

            // 3. At this point, header is at index 0 and buffer.length >= PACKET_SIZE
            const packet = buffer.slice(0, PACKET_SIZE);
            
            // --- Packet Processing Logic (Your existing logic) ---
            const payload = packet.slice(1, PACKET_SIZE - 1);
            const receivedChecksum = packet[PACKET_SIZE - 1];
            //console.log("received checksum: ", receivedChecksum);
            let calculatedChecksum = 0;
            for (const byte of payload) {
                calculatedChecksum ^= byte;
            }
            if(calculatedChecksum!=receivedChecksum){
                console.warn(`Received/calculated checksum: ", ${receivedChecksum}:${calculatedChecksum}`);
            }

            try {
                // 4. Unpack Payload and Samples (using Big Endian - BE, matching Python's '>')
                // Assuming you reverted to BE as discussed
                
                // The first 6 bytes of the payload:
                const depthIndex = payload.readUInt16BE(0);
                const tempScaled = payload.readInt16BE(2);
                const vDrvScaled = payload.readUInt16BE(4);
                
                //console.log(`DEBUG: Header Data: Depth Index=${depthIndex}, Temp Scaled=${tempScaled}`);

                const samplesBuffer = payload.slice(6);
                const samples = [];
                
                // Check array length before the loop
                if (samplesBuffer.length !== (NUM_SAMPLES * 2)) {
                    throw new Error(`Sample buffer size mismatch: Expected ${NUM_SAMPLES * 2}, got ${samplesBuffer.length}`);
                }

                for (let i = 0; i < NUM_SAMPLES; i++) {
                    // Read 2-byte unsigned int (Big Endian) for each sample
                    // If this line crashes, it confirms an Endianness/buffer corruption error.
                    samples.push(samplesBuffer.readUInt16BE(i * 2)); 
                }
                
                //console.log(`DEBUG: Samples Unpacked. First 30: ${samples.slice(0, 30)}`);

                // 5. Calculate Real Values and Emit
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
                };
                
                // 6. Push data to all connected web clients via WebSocket
                wss.clients.forEach(client => {
                    if (client.readyState === WebSocket.OPEN) {
                        client.send(JSON.stringify(dataToSend));
                    }
                });
                
                console.log(`DEBUG: WebSocket Sent.`);

            } catch (e) {
                // If ANY line inside the try block fails, it will print here
                console.error(`\n❌ CRITICAL UNPACKING CRASH: ${e.message}`);
                // DO NOT break here, still slice the buffer to keep the stream flowing
            }
            buffer = buffer.slice(PACKET_SIZE); 
        }
    });


} catch (e) {
    console.error(`❌ Fatal Error: Could not initialize serial port: ${e.message}`);
    console.log(`Please check if port ${SERIAL_PORT} is correct.`);
}

// server.js

// tracks raw signal only coming in on serial port and identify higest consostent signal to track and send to grapg front end

const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const { DelimiterParser } = require('@serialport/parser-delimiter');

// --- Configuration ---
const COM_PORT_PATH = '/dev/ttyACM1'; // e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux
const BAUD_RATE = 250000;
const EMA_ALPHA = 0.1;
const PORT = 3000;
const NUM_SAMPLES = 1800;
const PACKET_SIZE = 1 + 6 + 2 * NUM_SAMPLES + 1; // Expected total size: 3608 bytes
console.log("expecting packet size:", PACKET_SIZE, "bytes");

// Set to true to print the entire raw sample array for each frame.
const PRINT_RAW_SAMPLES = true;
// Defines the range (from the end of the 1800 samples) used to calculate average noise floor
const NOISE_FLOOR_RANGE = 100; 
// (Speed of Sound * Sample Time * 100 cm/m) / 2 = (330 * 13.2e-6 * 100) / 2 = 0.2178 cm/sample
const SAMPLE_RESOLUTION = 0.2178; // 2.1mm This is the smallest distance change that can be measured or resolved
let lastSmoothedDistance = null; 

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

function sonarPacketHandler() {
    try {
        const port = new SerialPort({ 
            path: COM_PORT_PATH, 
            baudRate: BAUD_RATE 
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
                    const samplesBuffer = payload.slice(6);
                    const rawSamples = [];
                    
                    // Check array length before the loop
                    if (samplesBuffer.length !== (NUM_SAMPLES * 2)) {
                        throw new Error(`Sample buffer size mismatch: Expected ${NUM_SAMPLES * 2}, got ${samplesBuffer.length}`);
                    }

                    for (let i = 0; i < NUM_SAMPLES; i++) {
                        // Read 2-byte unsigned int (Big Endian) for each sample
                        // If this line crashes, it confirms an Endianness/buffer corruption error.
                        rawSamples.push(samplesBuffer.readUInt16BE(i * 2)); 
                    }

                    const graphSamples = [];
                    if (PRINT_RAW_SAMPLES) {
                        // Truncate the output to the first 100 samples and the last 10 samples
                        //const partialSamples = rawSamples.slice(0, 360).concat(['...'], rawSamples.slice(-10));
                        const partialSamples = rawSamples.slice(0, 360);

                        // TEST FOR PRIMARY SIGNAL
                        for (let i = 0; i < partialSamples.length; i++) {
                            if (partialSamples[i] > 55) {
                                graphSamples.push(partialSamples[i]);
                            }else{
                                graphSamples.push('.');
                            }
                        }
                        console.dir(graphSamples, {'maxArrayLength': null});
                    }

                    const reflection = findPrimaryReflection(rawSamples);
    
                    // 2. Apply smoothing filter to the raw distance
                    const smoothedDistance = applyExponentialSmoothing(reflection.distance);
                    console.log(smoothedDistance);
                    
// new
                    const distMM = Math.round(smoothedDistance * 10);
                    // Peak Value: Clamp to 8 bits (max 255)
                    const peakVal = Math.min(reflection.value, 255); 
                    // Peak Index: Clamp to 16 bits (max 65535)
                    const peakIdx = reflection.index;
                    
                    // 2. Create the 5-byte binary buffer
                    const bufferToSend = Buffer.alloc(5);
                    
                    // Byte 0-1: Distance in mm (UInt16 Big Endian)
                    bufferToSend.writeUInt16BE(distMM, 0); 
                    
                    // Byte 2: Peak Value (UInt8)
                    bufferToSend.writeUInt8(peakVal, 2); 
                    
                    // Byte 3-4: Peak Index (UInt16 Big Endian)
                    bufferToSend.writeUInt16BE(peakIdx, 3);
// end new


                    // Check if the detected peak is strong enough to be considered a real reflection
                    if (reflection.value < 50) {
                        // Skip sending the frame if the reflection is weak
                        console.log(`[LOG WARN] Skipping log: Peak value ${reflection.value.toFixed(0)} is below threshold (50).`);
                        buffer = buffer.slice(PACKET_SIZE); 
                        continue; // IMPORTANT: Use continue to skip to the next packet
                    }
                    // 6. SIMPLIFIED: Only send the smoothed distance value and the peak value
                    // const dataToSend = {
                    //     distance: smoothedDistance, 
                    //     peakValue: reflection.value,
                    //     peakIndex: reflection.index
                    // };
                    // 7. Push consolidated data to all connected web clients via WebSocket
                    // wss.clients.forEach(client => {
                    //     if (client.readyState === WebSocket.OPEN) {
                    //         client.send(JSON.stringify(dataToSend));
                    //     }
                    // });

                    // 3. Push the binary buffer to all connected web clients
                    wss.clients.forEach(client => {
                        if (client.readyState === WebSocket.OPEN) {
                            // !!! IMPORTANT: Send the raw Buffer instead of a JSON string
                            client.send(bufferToSend); 
                        }
                    });
                    
                    console.log(`Frame Sent. Smoothed Distance: ${smoothedDistance.toFixed(2)} cm`);

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
}

/**
 * Calculates the average noise floor from the tail end of the samples (where only noise is expected).
 * @param {number[]} samples - Array of 1800 ADC values.
 * @returns {number} - The average noise floor value.
 */
function calculateNoiseFloor(samples) {
    // Use the last NOISE_FLOOR_RANGE samples
    const start = samples.length - NOISE_FLOOR_RANGE;
    if (start < 0) return 0;

    const tailSamples = samples.slice(start);
    const sum = tailSamples.reduce((acc, val) => acc + val, 0);
    
    return sum / NOISE_FLOOR_RANGE;
}

/**
 * Dynamically finds the end of the primary pulse/blind zone, defined as the first index
 * where the signal permanently drops to or below the calculated noise floor.
 * @param {number[]} samples - Array of 1800 ADC values.
 * @param {number} noiseFloor - The calculated average noise floor.
 * @returns {number} - The index where the signal is considered to have ended.
 */
function findBlindZoneEnd(samples, noiseFloor) {
    // We only need to check the first few hundred samples for the blind zone
    const searchLimit = 500; 
    const threshold = noiseFloor * 1.2; // Use a slight buffer above the pure average

    for (let i = 0; i < Math.min(samples.length, searchLimit); i++) {
        // Find the point where the signal drops close to the noise level
        if (samples[i] <= threshold) {
            // Return this index as the start of the usable measurement area
            return i;
        }
    }
    // Fallback: If no drop is detected, use a fixed safe limit
    return 150; 
}


/**
 * Finds the strongest reflection (max value and its index) in the raw sample array,
 * AFTER dynamically determining the end of the blind zone.
 * @param {number[]} samples - Array of 1800 ADC values.
 * @returns {object} - { value, index, distance, blindZoneEndIndex }
 */
function findPrimaryReflection(samples) {
    if (samples.length === 0) {
        return { value: 0, index: -1, distance: 0, blindZoneEndIndex: 0 };
    }

    // 1. Calculate the dynamic noise floor and start index
    const noiseFloor = calculateNoiseFloor(samples); 
    const startIndex = findBlindZoneEnd(samples, noiseFloor);
    
    let maxValue = 0;
    let maxIndex = -1;

    // 2. Start the search for the strongest peak AFTER the dynamically determined blind zone
    for (let i = startIndex; i < samples.length; i++) {
        if (samples[i] > maxValue) {
            maxValue = samples[i];
            maxIndex = i;
        }
    }
    
    // The calculated distance uses the index of the max value and the new calibrated resolution
    const rawDistanceCm = maxIndex * SAMPLE_RESOLUTION;

    return { 
        value: maxValue, 
        index: maxIndex, 
        distance: rawDistanceCm, // Return the raw, unfiltered distance
        blindZoneEndIndex: startIndex
    };
}

/**
 * Applies Exponential Moving Average (EMA) to the distance for smoothing.
 * This filters out short noise spikes while still tracking movement.
 * @param {number} currentDistance - The raw distance value from the latest sample.
 * @returns {number} - The smoothed distance value.
 */
function applyExponentialSmoothing(currentDistance) {
    if (lastSmoothedDistance === null) {
        // Initialize with the first reading
        lastSmoothedDistance = currentDistance;
        return currentDistance;
    }

    // EMA Formula: Smoothed = (Alpha * Current) + ((1 - Alpha) * Previous Smoothed)
    const smoothedDistance = 
        (EMA_ALPHA * currentDistance) + 
        ((1 - EMA_ALPHA) * lastSmoothedDistance);

    lastSmoothedDistance = smoothedDistance;
    return smoothedDistance;
}

/**
 * Main execution block
 */
async function main() { 
    sonarPacketHandler(); // Start the sonar data listener (now event-driven)
    
    console.log(`\nLogger started. Distance is now smoothed (EMA Alpha: ${EMA_ALPHA}).`);
    if (PRINT_RAW_SAMPLES) {
        console.log("!!! VERBOSE MODE ACTIVE: Partial raw sample array will be printed for each frame. !!!");
        console.log("!!! NOTE: Output is truncated to the first 100 and last 10 samples for readability. !!!");
    }
}

main().catch(err => {
    console.error('An unrecoverable error occurred in main:', err);
    process.exit(1);
});
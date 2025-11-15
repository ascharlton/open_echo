import sys
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
import socket # Kept socket import for utility functions

# --- Configuration ---
BAUD_RATE = 250000
NUM_SAMPLES = 1800  # Number of frequency/amplitude bins (X-axis)

# --- NEW CONFIGURATION: THRESHOLD ---
HIGHLIGHT_THRESHOLD = 50  # Highlight any sample value above this (scale is 0-65535)

# Constants used for data conversion (kept for accurate calculation)
SPEED_OF_SOUND = 330
SAMPLE_TIME = 13.2e-6
SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2


# --- Core Packet Reading Function ---

def read_packet(ser):
    """
    Reads a single, verified packet from the serial port.
    Returns unpacked sensor data and sample array, or None if reading fails.
    """
    payload_len = 6 + 2 * NUM_SAMPLES
    
    while True:
        # 1. Wait for Header
        header = ser.read(1)
        if header != b'\xAA':
            continue

        # 2. Read Payload and Checksum
        payload = ser.read(payload_len)
        checksum = ser.read(1)

        if len(payload) != payload_len or len(checksum) != 1:
            # print("⚠️ Incomplete packet read. Retrying...") # Suppress for continuous reading
            continue

        # 3. Verify Checksum
        calc_checksum = 0
        for byte in payload:
            calc_checksum ^= byte

        if calc_checksum != checksum[0]:
            print(f"⚠️ Checksum mismatch! Calculated: {calc_checksum:02X}, Received: {checksum[0]:02X}")
            continue

        # 4. Unpack Payload and Samples
        depth_index, temp_scaled, vDrv_scaled = struct.unpack(">HhH", payload[:6])
        samples = struct.unpack(f">{NUM_SAMPLES}H", payload[6:])

        # 5. Scale and Calculate Real Values
        depth_cm = depth_index * SAMPLE_RESOLUTION
        temperature = temp_scaled / 100.0
        drive_voltage = vDrv_scaled / 100.0
        values = np.array(samples, dtype=np.uint16) # Ensure correct NumPy type

        return values, depth_cm, temperature, drive_voltage


# --- Main Execution Loop ---

def run_serial_console(port, baud_rate):
    """Initializes serial connection, reads data, and prints highlighted samples."""
    print(f"Attempting to connect to: {port} @ {baud_rate} baud...")
    print(f"Highlight Threshold Set To: {HIGHLIGHT_THRESHOLD}")
    
    # Set numpy to print the full 1800-item array without truncation
    np.set_printoptions(threshold=NUM_SAMPLES + 10, linewidth=200)

    try:
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print(f"✅ Connected to {port}. Reading data...")
            
            while True:
                result = read_packet(ser)
                
                if result:
                    values, depth_cm, temperature, drive_voltage = result
                    
                    # --- Create Highlighted String ---
                    highlighted_samples = []
                    for value in values:
                        if value >= HIGHLIGHT_THRESHOLD:
                            # Highlight values over the threshold
                            #highlighted_samples.append(f"{value}")
                            highlighted_samples.append(f"*")
                        else:
                            highlighted_samples.append(str("."))

                    # Join the list into a space-separated string
                    samples_output = " ".join(highlighted_samples)
                    
                    # --- CONSOLE OUTPUT ---
                    print("\n" + "=" * 50)
                    print(f"Time: {time.strftime('%H:%M:%S')}")
                    print(f"Depth: {depth_cm:.1f} cm | Index: {int(depth_cm / SAMPLE_RESOLUTION)}")
                    print(f"Temp: {temperature:.1f} °C | Vdrv: {drive_voltage:.1f} V")
                    print("=" * 50)
                    print(f"Raw Samples (Highlighted [X] >= {HIGHLIGHT_THRESHOLD}):")
                    print(samples_output)
                
    except serial.SerialException as e:
        print(f"\n❌ Serial Port Error: {e}")
        print(f"Ensure device is connected and port '{port}' is correct.")
    except KeyboardInterrupt:
        print("\n🛑 Program terminated by user.")
    except Exception as e:
        print(f"\n❌ An unexpected error occurred: {e}")


if __name__ == "__main__":
    # --- Port Detection/Selection Logic ---
    ports = [p.device for p in serial.tools.list_ports.comports()]
    
    if len(ports) == 0:
        DEFAULT_PORT = "/dev/ttyACM1" 
        print(f"⚠️ No serial ports found. Defaulting to '{DEFAULT_PORT}'.")
    else:
        DEFAULT_PORT = ports[0]
        print(f"Found available serial ports: {ports}")
        
    selected_port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    
    run_serial_console(selected_port, BAUD_RATE)

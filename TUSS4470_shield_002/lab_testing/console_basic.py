import sys
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
import socket # Kept socket import for utility functions, though not strictly needed for core logic

# --- Configuration (Kept from Original) ---
BAUD_RATE = 250000
NUM_SAMPLES = 1800  # Number of frequency/amplitude bins (X-axis)

# Constants used for data conversion (kept for accurate calculation of MAX_DEPTH/RESOLUTION)
SPEED_OF_SOUND = 330  # meters per second in water
SAMPLE_TIME = 13.2e-6  # 13.2 microseconds in seconds Atmega328 sample speed
SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2  # cm per row
PACKET_SIZE = 1 + 6 + 2 * NUM_SAMPLES + 1 # header + payload + checksum


# --- Core Packet Reading Function (Simplified) ---

def read_packet(ser):
    """
    Reads a single, verified packet from the serial port.
    Returns unpacked sensor data and sample array, or None if reading fails.
    """
    while True:
        # 1. Wait for Header
        header = ser.read(1)
        if header != b'\xAA':
            continue

        # 2. Read Payload and Checksum
        payload_len = 6 + 2 * NUM_SAMPLES
        payload = ser.read(payload_len)
        checksum = ser.read(1)

        if len(payload) != payload_len or len(checksum) != 1:
            print("⚠️ Incomplete packet read. Retrying...")
            continue

        # 3. Verify Checksum
        calc_checksum = 0
        for byte in payload:
            calc_checksum ^= byte

        if calc_checksum != checksum[0]:
            print(f"⚠️ Checksum mismatch! Calculated: {calc_checksum:02X}, Received: {checksum[0]:02X}")
            continue

        # 4. Unpack Payload
        # >HhH: Big-Endian Unsigned Short (2B), Signed Short (2B), Unsigned Short (2B)
        depth_index, temp_scaled, vDrv_scaled = struct.unpack(">HhH", payload[:6])
        
        # Clamp depth (NUM_SAMPLES constant doesn't strictly exist here, but we use depth_index for clarity)
        # Note: We skip the clamping line for simplicity as 'NUM_SAMPLES' is the effective max.

        # 5. Unpack Samples (The main data array)
        # >{NUM_SAMPLES}H: Big-Endian Unsigned Short for the rest of the payload
        samples = struct.unpack(f">{NUM_SAMPLES}H", payload[6:])

        # 6. Scale and Calculate Real Values
        depth_cm = depth_index * SAMPLE_RESOLUTION
        temperature = temp_scaled / 100.0
        drive_voltage = vDrv_scaled / 100.0
        values = np.array(samples)

        return values, depth_cm, temperature, drive_voltage


# --- Main Execution Loop ---

def run_serial_console(port, baud_rate):
    """Initializes serial connection and starts continuous reading."""
    print(f"Attempting to connect to: {port} @ {baud_rate} baud...")
    
    try:
        # Use 'with' to ensure the serial port is closed properly
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print(f"✅ Connected to {port}. Reading data...")
            print("-" * 50)

            while True:
                result = read_packet(ser)
                
                if result:
                    values, depth_cm, temperature, drive_voltage = result
                    
                    # --- CONSOLE OUTPUT ---
                    print(f"Time: {time.strftime('%H:%M:%S')}")
                    print(f"Depth: {depth_cm:.1f} cm | Index: {int(depth_cm / SAMPLE_RESOLUTION)}")
                    print(f"Temp: {temperature:.1f} °C | Vdrv: {drive_voltage:.1f} V")
                    
                    # Print a slice of the raw data (e.g., the first 10 values)
                    # Use 'threshold=np.inf' to print the full array if desired
                    #np.set_printoptions(threshold=100) # Print only the first/last few items
                    #print(f"Raw Samples (First 10): {values[:150]}")
                    #values = np.array(samples)
                    np.set_printoptions(threshold=np.inf)
                    print(f"Raw Samples: {values[:550]}")
                    print("-" * 50)
                
                # Optional: Add a small delay if the console is flooding too fast
                # time.sleep(0.01)

    except serial.SerialException as e:
        print(f"\n❌ Serial Port Error: {e}")
        print(f"Ensure device is connected and port '{port}' is correct.")
    except KeyboardInterrupt:
        print("\n🛑 Program terminated by user.")
    except Exception as e:
        print(f"\n❌ An unexpected error occurred: {e}")


if __name__ == "__main__":
    # --- Auto-detect Port or use default ---
    ports = [p.device for p in serial.tools.list_ports.comports()]
    
    if len(ports) == 0:
        # Default fallback for Linux/Pi
        DEFAULT_PORT = "/dev/ttyACM1" 
        print(f"⚠️ No serial ports found. Defaulting to '{DEFAULT_PORT}'.")
    else:
        # Use the first detected port
        DEFAULT_PORT = ports[0]
        print(f"Found available serial ports: {ports}")
        
    
    # --- Execution ---
    # The default port can be overridden by passing it as a command-line argument:
    # python serial_console.py /dev/ttyACM1
    
    selected_port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    
    run_serial_console(selected_port, BAUD_RATE)
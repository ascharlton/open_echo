import sys
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time

# --- Configuration ---
BAUD_RATE = 250000
#NUM_SAMPLES = 1800  # Number of frequency/amplitude bins (X-axis)
NUM_SAMPLES = 900

# --- TRACKING CONFIGURATION ---
THRESHOLD = 50                  # 50 Minimum value a point must have to be considered a peak
CONSISTENCY_SAMPLES = 3         # 3 - Minimum number of samples a peak must appear in to be 'consistent'
POSITION_TOLERANCE = 5          # 5- Range (+/-) within which a peak position is considered the same

# Constants used for data conversion
SPEED_OF_SOUND = 330
SAMPLE_TIME = 13.2e-6
SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2


# --- Signal Tracker Class for Time-Series Analysis ---

class SignalTracker:
    """
    Manages a buffer of past sample arrays and identifies consistent signal indices.
    """
    def __init__(self, buffer_size, threshold, tolerance):
        self.buffer = []
        self.buffer_size = buffer_size
        self.threshold = threshold
        self.tolerance = tolerance
        # Holds the indices that were consistent in the last analysis
        self.consistent_indices = set() 

    def update_and_get_consistent_indices(self, current_values):
        """
        Adds current samples to the buffer, analyzes consistency, and returns 
        the indices of consistently appearing peaks.
        """
        # 1. Identify current 'peak' indices (above threshold)
        current_peak_indices = np.where(current_values >= self.threshold)[0]
        
        # 2. Update buffer
        self.buffer.append(current_peak_indices)
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)

        # Skip analysis until the buffer is full
        if len(self.buffer) < self.buffer_size:
            self.consistent_indices = set()
            return self.consistent_indices
        
        # 3. Analyze Consistency across all buffered samples
        # Use a dictionary to count how many times an index is part of a cluster
        index_counts = {}

        # The core idea: Iterate through the latest peaks and check for neighbours in the history.
        for index in current_peak_indices:
            
            is_consistent = True
            
            # Check this index against ALL previous buffers
            for past_peak_indices in self.buffer[:-1]:
                
                # Check if any past peak falls within the tolerance range [index-tol, index+tol]
                min_idx = max(0, index - self.tolerance)
                max_idx = min(NUM_SAMPLES - 1, index + self.tolerance)
                
                # Check if the set of past peaks intersects the allowed range
                is_match = False
                for p_idx in past_peak_indices:
                    if min_idx <= p_idx <= max_idx:
                        is_match = True
                        break
                
                if not is_match:
                    is_consistent = False
                    break # Break out of past_peak_indices loop

            if is_consistent:
                # If consistent across all past frames, this index is significant.
                # We use the current index as the canonical consistent position.
                index_counts[index] = index_counts.get(index, 0) + 1
        
        # 4. Final selection: indices that passed the check
        # Since we broke the inner loop on failure, the result set is already 'consistent'.
        self.consistent_indices = set(index_counts.keys())
        return self.consistent_indices


# --- Core Packet Reading Function (Unchanged) ---

def read_packet(ser):
    """
    Reads a single, verified packet from the serial port.
    """
    payload_len = 6 + 2 * NUM_SAMPLES
    
    # ... [Same read_packet implementation as before, returning values, depth_cm, temperature, drive_voltage] ...
    while True:
        # 1. Wait for Header
        header = ser.read(1)
        if header != b'\xAA':
            continue

        # 2. Read Payload and Checksum
        payload = ser.read(payload_len)
        checksum = ser.read(1)

        if len(payload) != payload_len or len(checksum) != 1:
            continue

        # 3. Verify Checksum
        calc_checksum = 0
        for byte in payload:
            calc_checksum ^= byte

        if calc_checksum != checksum[0]:
            # print(f"⚠️ Checksum mismatch! Calculated: {calc_checksum:02X}, Received: {checksum[0]:02X}")
            continue

        # 4. Unpack Payload and Samples
        depth_index, temp_scaled, vDrv_scaled = struct.unpack(">HhH", payload[:6])
        samples = struct.unpack(f">{NUM_SAMPLES}H", payload[6:])

        # 5. Scale and Calculate Real Values
        depth_cm = depth_index * SAMPLE_RESOLUTION
        temperature = temp_scaled / 100.0
        drive_voltage = vDrv_scaled / 100.0
        values = np.array(samples, dtype=np.uint16)

        return values, depth_cm, temperature, drive_voltage


# --- Main Execution Loop ---

def run_serial_console(port, baud_rate):
    """Initializes serial connection, reads data, and prints highlighted samples."""
    print(f"Attempting to connect to: {port} @ {baud_rate} baud...")
    print(f"Tracking: Threshold={THRESHOLD}, Consistency={CONSISTENCY_SAMPLES}, Tolerance={POSITION_TOLERANCE}")

    # Set numpy to print the full array without truncation
    np.set_printoptions(threshold=NUM_SAMPLES + 10, linewidth=200)
    
    # Initialize the Signal Tracker
    tracker = SignalTracker(CONSISTENCY_SAMPLES, THRESHOLD, POSITION_TOLERANCE)

    try:
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print(f"✅ Connected to {port}. Reading data...")
            
            while True:
                result = read_packet(ser)
                
                if result:
                    values, depth_cm, temperature, drive_voltage = result
                    
                    # 1. Get Consistent Indices
                    consistent_indices = tracker.update_and_get_consistent_indices(values)
                    
                    # 2. Create Highlighted String
                    highlighted_samples = []
                    for i, value in enumerate(values):
                        if i in consistent_indices and value >= THRESHOLD:
                            # Highlight: Show value for consistent, high-value data
                            highlighted_samples.append(f"{value:4d}") 
                        elif value >= THRESHOLD:
                            # Show a marker for high-value but inconsistent data
                            highlighted_samples.append(" .  ") 
                        else:
                            # Show a dot for low-value data (below threshold)
                            highlighted_samples.append("    ") 

                    samples_output = "".join(highlighted_samples)
                    
                    # --- CONSOLE OUTPUT ---
                    print("\n" + "=" * 100)
                    print(f"Time: {time.strftime('%H:%M:%S')} | Depth: {depth_cm:.1f} cm | Temp: {temperature:.1f} °C | Vdrv: {drive_voltage:.1f} V")
                    print("-" * 100)
                    print(f"Consistent Peaks (Value shown) | Inconsistent Peaks (*) | Background (.)")
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

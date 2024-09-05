import can
import time
import os
import serial.tools.list_ports

def parse_can_message(line):
    # Assuming each line in the file represents a CAN message in the format "ID DATA"
    parts = line.split(' ')
    arbitration_id = int(parts[0][:2], 16)
    data = [int(parts[0][i:i+2], 16) for i in range(2, len(parts[0]), 2)] + [int(byte, 16) for byte in parts[1:]]
    return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)

def can_send_message(bus, msg):
    bus.send(msg)
    data_bytes = ', '.join([f'0x{byte:02X}' for byte in msg.data])
    print(f"msg1 = can.Message(arbitration_id=0x{msg.arbitration_id:X}, data=[{data_bytes}], is_extended_id=False)")
    time.sleep(0.5)  # Introduce a 500ms delay between messages

def find_can_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "CAN" in port.description or "USB2CAN" in port.description:
            return port.device
    return None

def main():
    script_directory = os.path.dirname(os.path.abspath(__file__))
    txt_files = [file for file in os.listdir(script_directory) if file.endswith(".txt")]
    if not txt_files:
        print("No .txt files found in the script directory.")
        return

    selected_file = txt_files[0]  # Assuming the first .txt file is the one to be used
    file_path = os.path.join(script_directory, selected_file)

    can_port = find_can_port()
    if can_port is None:
        print("No CAN interface found. Please check your connection.")
        return

    print(f"Using CAN interface on port: {can_port}")
    bus = can.interface.Bus(bustype='slcan', channel=can_port, bitrate=500000)

    with open(file_path, 'r') as file:
        lines = file.readlines()

    for line in lines:
        msg = parse_can_message(line.strip())
        can_send_message(bus, msg)

    # Close the bus after sending all messages
    bus.shutdown()

if __name__ == "__main__":
    main()

import os
import re

# Parameters Section
# -------------------

# Gearbox ratios for each motor
gear_ratios = [1, 1, 1, 1, 1, 1]  # Replace with your actual gearbox ratios

# Direction inversion for each motor (True/False)
invert_direction = [True, True, False, False, False, False]  # Set True for motors where direction should be inverted

# Initialize zero positions and last positions
initial_positions = [0] * 6
last_positions = [0] * 6

# -------------------

def calculate_crc(data):
    crc = sum(data) & 0xFF
    return crc





def convert_to_can_message(axis_id, speed, position, gear_ratio, invert_direction=False):
    can_id = format(axis_id, '02X')
    speed_hex = format(speed, '04X')

    # Calculate relative position based on the initial position
    rel_position = int((position * gear_ratio - initial_positions[axis_id - 1]) * 100)

    # Handle signed 24-bit integer using two's complement representation
    rel_position_hex = format(rel_position & 0xFFFFFF, '06X')

    # Update last_position for the axis
    last_positions[axis_id - 1] = position * gear_ratio

    return can_id + 'F4' + speed_hex + '02' + rel_position_hex



def console():
    # Get user input from text input field
    user_input = console_input.get().strip()
    
    # Parse user input to extract the entire message
    try:
        # Parse the entire message
        msg_data = bytes.fromhex(user_input)
        
        # Create a CAN message
        msg = can.Message(data=msg_data, is_extended_id=False)  # Assuming standard ID
        
        # Send the CAN message
        try:
            with can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000) as bus:
                bus.send(msg)
                print("Message sent successfully.")
        except Exception as e:
            print(f"Error sending message: {e}")
    except ValueError:
        print("Invalid input format. Please enter a valid message in hexadecimal format.")






def process_tap_files():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_filename = os.path.join(script_dir, "jog.tap")
    output_filename = os.path.join(script_dir, "jog.txt")

    with open(input_filename, 'r') as input_file, open(output_filename, 'w') as output_file:
        speed = 0

        for line in input_file:
            speed_match = re.search(r'F(\d+)', line)
            if speed_match:
                try:
                    speed = int(speed_match.group(1))
                except ValueError:
                    continue

            if line.startswith("G90"):
                values = [float(value) if '.' in value else int(value) for value in re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', line)]

                if len(values) >= 7:
                    for axis_id, position in enumerate(values[1:7], start=1):
                        # Use the corresponding gear ratio for each motor
                        gear_ratio = gear_ratios[axis_id - 1]
                        invert_dir = invert_direction[axis_id - 1]
                        can_message = convert_to_can_message(axis_id, speed, position, gear_ratio, invert_dir)
                        crc = calculate_crc([int(can_message[i:i+2], 16) for i in range(0, len(can_message), 2)])
                        can_message_with_crc = can_message + format(crc, '02X')
                        output_file.write(can_message_with_crc + '\n')
                        print(f"Converted g-code line to CAN message: {can_message_with_crc}")

if __name__ == "__main__":
    process_tap_files()


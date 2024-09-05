import os

import serial

import time

import serial.tools.list_ports



def find_serial_port():

    """

    Finds the port for the serial device.



    Returns:

        The device name of the serial port, or None if not found.

    """

    ports = list(serial.tools.list_ports.comports())

    for port in ports:

        # You might need to adjust this condition based on your specific device

        if "USB" in port.description or "ACM" in port.description:

            return port.device

    return None



def send_gcode(serial_port: str, baudrate: int, file_path: str) -> None:

    """

    Sends G-code commands from a file to a device over a serial connection.



    Args:

        serial_port: The serial port to which the device is connected (e.g., '/dev/ttyUSB0').

        baudrate: The baud rate for the serial connection (e.g., 115200).

        file_path: The path to the file containing G-code commands.

    """

    try:

        with serial.Serial(serial_port, baudrate, timeout=1) as ser:

            with open(file_path, 'r') as file:

                lines = file.readlines()

            for line in lines:

                command = line.strip()

                if command:

                    ser.write((command + '\n').encode())

                    print(f"Sent: {command}")

                    

                    # Optionally wait for the device to acknowledge the command

                    response = ser.readline().decode().strip()

                    if response:

                        print(f"Received: {response}")

                    time.sleep(0.1)  # Add delay if necessary

    except serial.SerialException as e:

        print(f"Serial error: {e}")

    except FileNotFoundError:

        print("G-code file not found.")



def main() -> None:

    """

    Main function to send G-code commands from a .tap file to a device over a serial connection.

    """

    script_directory = os.path.dirname(os.path.abspath(__file__))

    file_path = os.path.join(script_directory, "jog.tap")

    baudrate = 115200  # Change to your device's baud rate



    if not os.path.exists(file_path):

        print("jog.tap file not found in the script directory.")

        return



    serial_port = find_serial_port()

    if serial_port is None:

        print("No suitable serial port found. Please check your connection.")

        return



    print(f"Using serial port: {serial_port}")

    send_gcode(serial_port, baudrate, file_path)



if __name__ == "__main__":

    main()

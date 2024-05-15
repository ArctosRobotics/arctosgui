import os
import time
from typing import List

import can


def parse_can_message(line: str) -> can.Message:
    """
    Parses a single line from a file representing a CAN message.

    Args:
        line: A string representing a CAN message in the format "ID DATA".

    Returns:
        A `can.Message` object with the parsed arbitration ID and data bytes.

    Note:
        The arbitration ID is assumed to be the first two characters of the ID,
        converted from hexadecimal. The data is assumed to be hexadecimal bytes,
        where the first part of the data is directly following the ID in the
        same string, and additional data parts are separated by spaces.
    """
    parts = line.split(" ")
    arbitration_id = int(parts[0][:2], 16)
    data = [int(parts[0][i : i + 2], 16) for i in range(2, len(parts[0]), 2)] + [
        int(byte, 16) for byte in parts[1:]
    ]
    return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)


def calculate_crc(arbitration_id: int, status: int) -> int:
    """
    Calculates a simple CRC value based on the arbitration ID and status.

    Args:
        arbitration_id: The arbitration ID of the CAN message.
        status: An arbitrary status value.

    Returns:
        An integer representing the calculated CRC value.
    """
    return (arbitration_id + 0xF4 + status) & 0xFF


def adjust_speeds_within_packet(messages: List[can.Message]) -> None:
    """
    Adjusts the speeds within a packet of CAN messages based on the average speed.

    Args:
        messages: A list of `can.Message` objects, each representing a CAN message.

    Note:
        This function directly modifies the `data` attribute of each `can.Message`
        object in the list to adjust the speed values.
    """
    speeds = [(msg.data[3] << 8) + msg.data[4] for msg in messages]
    reference_speed = sum(speeds) // len(speeds)
    if reference_speed == 0:
        return
    for msg in messages:
        speed = (msg.data[3] << 8) + msg.data[4]
        adjusted_speed = int((speed / reference_speed) * reference_speed)
        msg.data[3] = (adjusted_speed >> 8) & 0xFF
        msg.data[4] = adjusted_speed & 0xFF


def can_send_messages(bus: can.interface.Bus, messages: List[can.Message]) -> None:
    """
    Sends a list of CAN messages through a specified CAN bus and waits for responses.

    Args:
        bus: The `can.interface.Bus` instance representing the CAN bus to send messages on.
        messages: A list of `can.Message` objects to be sent.

    Note:
        This function waits for responses from expected motors after sending messages
        and prints out the status of the sent and received messages.
    """
    expected_responses = {1, 2}
    received_responses = set()
    for msg in messages:
        bus.send(msg)
        data_bytes = ", ".join([f"0x{byte:02X}" for byte in msg.data])
        print(
            f"Sent: arbitration_id=0x{msg.arbitration_id:X}, data=[{data_bytes}], is_extended_id=False"
        )
    timeout = 0.5
    start_time = time.time()
    while True:
        received_msg = bus.recv(timeout=3)
        if received_msg is not None:
            received_data_bytes = ", ".join(
                [f"0x{byte:02X}" for byte in received_msg.data]
            )
            print(
                f"Received: arbitration_id=0x{received_msg.arbitration_id:X}, data=[{received_data_bytes}], is_extended_id=False"
            )
            if received_msg.arbitration_id in expected_responses:
                received_responses.add(received_msg.arbitration_id)
        if received_responses == expected_responses:
            if all(
                received_msg.data[0] == 2 if received_msg is not None else False
                for received_msg in [bus.recv(timeout=0.1)] * len(expected_responses)
            ):
                print(
                    "Responses received for all expected motors with status 2. Moving to the next set of messages."
                )
                break
        if time.time() - start_time > timeout:
            print("Timeout waiting for responses from expected motors with status 2.")
            break


def main() -> None:
    """
    Main function to read CAN messages from a .txt file, send them through a CAN bus, and adjust speeds within packets.
    """
    script_directory = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_directory, "gcode.txt")

    if not os.path.exists(file_path):
        print("gcode.txt file not found in the script directory.")
        return

    bus = can.interface.Bus(bustype="slcan", channel="/dev/ttyACM0", bitrate=500000)

    with open(file_path, "r") as file:
        lines = file.readlines()

    message_sets = [lines[i : i + 6] for i in range(0, len(lines), 6)]

    for message_set in message_sets:
        messages = [parse_can_message(line.strip()) for line in message_set]
        adjust_speeds_within_packet(messages)
        can_send_messages(bus, messages)

    bus.shutdown()




if __name__ == "__main__":
    main()

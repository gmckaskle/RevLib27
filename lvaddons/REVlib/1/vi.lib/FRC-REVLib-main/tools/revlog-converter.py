# Note: steps to add new devices are ordered by number in comments in the code (4 steps)

# Bundle command to exe (in venv): pyinstaller --onefile --name revlog-converter --add-data "spark.public.dbc;." --add-data "servo_hub.public.dbc;." revlog-converter.py
# Add more --add-data flags for future device DBCs as needed

import argparse
import os
import struct
import sys
from abc import ABC, abstractmethod

import cantools
from tqdm import tqdm

RECORDS = {}
NEXT_RECORD_ID = 1

HEADER = "WPILOG"
VERSION_MAJOR = 1
VERSION_MINOR = 0
EXTRA_HEADER = ""

# 1. Add new DBC_FILE_PATH, PREFIX, and device type ID from FRC CAN Device specs for new device
SPARK_DBC_FILE_PATH = "spark.public.dbc"
SPARK_PREFIX = "REV/Spark-"
MOTOR_CONTROLLER = 2

SERVO_HUB_DBC_FILE_PATH = "servo_hub.public.dbc"
SERVO_HUB_PREFIX = "REV/ServoHub-"
SERVO_CONTROLLER = 12


class Device(ABC):
    def __init__(self, dbc_file_path, prefix, device_type_id):
        self.prefix = prefix
        self.device_type_id = device_type_id
        self.firmware_frame_spec = None
        self.periodic_frame_specs = []
        self.dbc = None
        try:
            if getattr(sys, "frozen", False) and hasattr(sys, "_MEIPASS"):
                path = sys._MEIPASS
            else:
                path = os.path.dirname(os.path.abspath(__file__))
            filepath = os.path.join(path, dbc_file_path)
            self.dbc = cantools.database.load_file(filepath)
        except FileNotFoundError:
            print(f"Error: DBC file not found at '{filepath}'")

        self.load_frame_specs()

    @abstractmethod
    def load_frame_specs(self):
        pass

    @abstractmethod
    def parse_firmware_version(self, decoded_message):
        pass


# 2. Create class that inherits Device to define the load_frame_specs and parse_firmware_version methods based on specific device DBC
class Spark(Device):
    def load_frame_specs(self):
        for message in self.dbc.messages:
            if message.name == "GET_FIRMWARE_VERSION":
                self.firmware_frame_spec = message.frame_id
            if message.name.startswith("STATUS_"):
                self.periodic_frame_specs.append(message.frame_id)

    def parse_firmware_version(self, decoded_message):
        firmware_version = {}
        firmware_version_str = ""
        for signal_name, signal_value in decoded_message.items():
            if (
                signal_name.upper() == "MAJOR"
                or signal_name.upper() == "MINOR"
                or signal_name.upper() == "BUILD"
            ):
                firmware_version[signal_name.upper()] = str(signal_value)
        firmware_version_str += (
            firmware_version["MAJOR"]
            + "."
            + firmware_version["MINOR"]
            + "."
            + firmware_version["BUILD"]
        )
        return firmware_version_str


class ServoHub(Device):
    def load_frame_specs(self):
        for message in self.dbc.messages:
            if message.name == "GET_VERSION":
                self.firmware_frame_spec = message.frame_id
            if message.name.startswith("STATUS_"):
                self.periodic_frame_specs.append(message.frame_id)

    def parse_firmware_version(self, decoded_message):
        firmware_version = {}
        firmware_version_str = ""
        for signal_name, signal_value in decoded_message.items():
            if (
                signal_name.upper() == "FIRMWARE_YEAR"
                or signal_name.upper() == "FIRMWARE_MINOR"
                or signal_name.upper() == "FIRMWARE_FIX"
            ):
                firmware_version[signal_name.upper()] = str(signal_value)
        firmware_version_str += (
            firmware_version["FIRMWARE_YEAR"]
            + "."
            + firmware_version["FIRMWARE_MINOR"]
            + "."
            + firmware_version["FIRMWARE_FIX"]
        )
        return firmware_version_str


# 3. Create new device variable for new class instance
spark_device = Spark(SPARK_DBC_FILE_PATH, SPARK_PREFIX, MOTOR_CONTROLLER)
servo_hub_device = ServoHub(SERVO_HUB_DBC_FILE_PATH, SERVO_HUB_PREFIX, SERVO_CONTROLLER)

# 4. Add new device to devices dictionary (key: device_type_id, value: device)
devices = {
    spark_device.device_type_id: spark_device,
    servo_hub_device.device_type_id: servo_hub_device,
}


def get_wpilog_type_from_signal(signal_spec):
    if signal_spec.length == 1:
        return "boolean"

    if isinstance(signal_spec.scale, float):
        return "double"

    if signal_spec.is_float:
        if signal_spec.length == 32:
            return "float"
        else:
            return "double"
    return "int64"


def required_bytes(value):
    if value <= 0xFF:
        return 1
    if value <= 0xFFFF:
        return 2
    if value <= 0xFFFFFF:
        return 3
    else:
        return 4


def required_timestamp_bytes(value):
    if value <= 0xFF:
        return 1
    if value <= 0xFFFF:
        return 2
    if value <= 0xFFFFFF:
        return 3
    if value <= 0xFFFFFFFF:
        return 4
    if value <= 0xFFFFFFFFFF:
        return 5
    if value <= 0xFFFFFFFFFFFF:
        return 6
    if value <= 0xFFFFFFFFFFFFFF:
        return 7
    else:
        return 8


def write_control_record(log, entry_id, entry_name, data_type, metadata):
    if metadata == None:
        metadata = ""
    payload = b""
    payload += struct.pack("<BII", 0, entry_id, len(entry_name))
    payload += entry_name.encode(encoding="utf-8")
    payload += struct.pack("<I", len(data_type))
    payload += data_type.encode(encoding="utf-8")
    payload += struct.pack("<I", len(metadata))
    payload += metadata.encode(encoding="utf-8")

    header_entry_id = 0
    payload_size = len(payload)
    timestamp_us = 0

    bitfield = 0
    bitfield |= (required_bytes(header_entry_id) - 1) & 0x3
    bitfield |= ((required_bytes(payload_size) - 1) & 0x3) << 2
    bitfield |= ((required_timestamp_bytes(timestamp_us) - 1) & 0x7) << 4

    log.write(struct.pack("<B", bitfield))
    log.write(struct.pack("<Q", header_entry_id)[: required_bytes(header_entry_id)])
    log.write(struct.pack("<Q", payload_size)[: required_bytes(payload_size)])
    log.write(struct.pack("<Q", timestamp_us)[: required_timestamp_bytes(timestamp_us)])
    log.write(payload)

    RECORDS[entry_name] = (entry_id, data_type)


def write_record(log, entry_name, entry_value, timestamp):

    entry_id = RECORDS[entry_name][0]
    data_type = RECORDS[entry_name][1]
    payload = b""

    if data_type == "boolean":
        payload = struct.pack("<?", entry_value)
    elif data_type == "int64":
        payload = struct.pack("<q", entry_value)
    elif data_type == "float":
        payload = struct.pack("<f", float(entry_value))
    elif data_type == "double":
        payload = struct.pack("<d", float(entry_value))
    elif data_type == "string":
        payload = entry_value.encode(encoding="utf-8")
    else:
        return b""
    # Maybe need to add more array data types

    payload_size = len(payload)
    timestamp_us = timestamp * 1000  # convert ms to us

    bitfield = 0
    bitfield |= (required_bytes(entry_id) - 1) & 0x3
    bitfield |= ((required_bytes(payload_size) - 1) & 0x3) << 2
    bitfield |= ((required_timestamp_bytes(timestamp_us) - 1) & 0x7) << 4

    log.write(struct.pack("<B", bitfield))
    log.write(struct.pack("<Q", entry_id)[: required_bytes(entry_id)])
    log.write(struct.pack("<Q", payload_size)[: required_bytes(payload_size)])
    log.write(struct.pack("<Q", timestamp_us)[: required_timestamp_bytes(timestamp_us)])
    log.write(payload)


def read_variable_int(data, cursor, length):
    if cursor + length > len(data):
        raise IndexError("Not enough data to read variable integer.")

    int_bytes = data[cursor : cursor + length]
    # Pad with zeros to unpack as a 64-bit integer safely
    int_bytes += b"\x00" * (8 - len(int_bytes))
    value = struct.unpack("<Q", int_bytes)[0]
    return value, cursor + length


def parse_log_file(log_filepath, log_destination_filepath, log_name):
    global NEXT_RECORD_ID
    # Read the entire log file
    try:
        with open(log_filepath, "rb") as f:
            binary_data = f.read()
        if not binary_data:
            print(
                f"Warning: Log file '{log_filepath}' is empty. No output will be generated."
            )
            return
    except FileNotFoundError:
        print(f"Error: Log file not found at '{log_filepath}'")
        return
    except PermissionError:
        print(f"Error: Permission denied. Could not read '{log_filepath}'")
        return
    except ValueError:
        print("Error: Log file contains non-binary characters.")
        return

    print(f"Parsing '{log_filepath}'...")

    try:
        log = open(os.path.join(log_destination_filepath, log_name + ".wpilog"), "wb")
    except PermissionError:
        print(
            f"Error: permission denied. Could not write to '{log_destination_filepath}'"
        )
        return

    log.write(
        HEADER.encode(encoding="utf-8")
        + struct.pack("<HI", (VERSION_MAJOR << 8) | VERSION_MINOR, len(EXTRA_HEADER))
        + EXTRA_HEADER.encode(encoding="utf-8")
    )

    # Parse the stream of binary data
    cursor = 0

    progressbar = tqdm(total=len(binary_data))
    while cursor < len(binary_data):
        try:
            old_cursor = cursor
            # Decode record header
            if cursor + 1 > len(binary_data):
                break
            bitfield = binary_data[cursor]
            cursor += 1

            # Decode bitfield to get header field lengths
            entry_id_len = (bitfield & 0b00000011) + 1
            size_len = ((bitfield >> 2) & 0b00000011) + 1

            # Read variable-length header fields
            entry_id, cursor = read_variable_int(binary_data, cursor, entry_id_len)
            payload_size, cursor = read_variable_int(binary_data, cursor, size_len)

            # Read the payload
            payload_bytes = binary_data[cursor : cursor + payload_size]
            cursor += payload_size
            progressbar.update(cursor - old_cursor)

            if entry_id == 1:
                payload_cursor = 0
                message_size = 10  # 4-byte id + 6-byte data

                while payload_cursor + message_size <= len(payload_bytes):
                    message_chunk = payload_bytes[
                        payload_cursor : payload_cursor + message_size
                    ]

                    message_id = struct.unpack_from("<I", message_chunk)[0]
                    can_data = message_chunk[4:10]

                    device_type = (message_id >> 24) & 0x1F

                    try:
                        decoded_message = devices[device_type].dbc.decode_message(
                            devices[device_type].firmware_frame_spec,
                            can_data,
                            allow_truncated=True,
                        )
                        if (
                            devices[device_type].prefix
                            + str(message_id & 0x3F)
                            + "/FIRMWARE"
                            not in RECORDS
                        ):
                            write_control_record(
                                log,
                                NEXT_RECORD_ID,
                                devices[device_type].prefix
                                + str(message_id & 0x3F)
                                + "/FIRMWARE",
                                "string",
                                "",
                            )
                            NEXT_RECORD_ID += 1
                        firmware_version = ""
                        firmware_version = devices[device_type].parse_firmware_version(
                            decoded_message
                        )
                        write_record(
                            log,
                            devices[device_type].prefix
                            + str(message_id & 0x3F)
                            + "/FIRMWARE",
                            firmware_version,
                            0,
                        )
                    except KeyError:
                        pass  # Skip CAN IDs not in the DBC

                    payload_cursor += message_size
            elif entry_id == 2:
                payload_cursor = 0
                message_size = 16  # 4-byte ts + 4-byte id + 8-byte data

                while payload_cursor + message_size <= len(payload_bytes):
                    message_chunk = payload_bytes[
                        payload_cursor : payload_cursor + message_size
                    ]
                    msg_ts_ms, message_id = struct.unpack_from("<II", message_chunk)
                    can_data = message_chunk[8:16]

                    device_type = (message_id >> 24) & 0x1F

                    try:
                        frame = devices[device_type].periodic_frame_specs[
                            (message_id >> 6) & 0xF
                        ]
                        decoded_message = devices[device_type].dbc.decode_message(
                            frame, can_data
                        )
                        for signal_name, signal_value in decoded_message.items():
                            signal_spec = (
                                devices[device_type]
                                .dbc.get_message_by_frame_id(frame)
                                .get_signal_by_name(signal_name)
                            )
                            folder = "/"
                            if signal_name.endswith("FAULT"):
                                folder = "/FAULT/"
                            elif signal_name.endswith("WARNING"):
                                folder = "/WARNING/"
                            if (
                                devices[device_type].prefix
                                + str(message_id & 0x3F)
                                + folder
                                + signal_name
                                not in RECORDS
                            ):
                                if signal_value is None:
                                    continue
                                wpilog_type = get_wpilog_type_from_signal(signal_spec)
                                write_control_record(
                                    log,
                                    NEXT_RECORD_ID,
                                    devices[device_type].prefix
                                    + str(message_id & 0x3F)
                                    + folder
                                    + signal_name,
                                    wpilog_type,
                                    (
                                        signal_spec.comment
                                        if signal_spec.comment
                                        else signal_spec.unit
                                    ),
                                )
                                NEXT_RECORD_ID += 1
                            if (
                                signal_spec.maximum
                                and signal_value > signal_spec.maximum
                            ):
                                signal_value = signal_spec.maximum
                            elif (
                                signal_spec.minimum
                                and signal_value < signal_spec.minimum
                            ):
                                signal_value = signal_spec.minimum
                            write_record(
                                log,
                                devices[device_type].prefix
                                + str(message_id & 0x3F)
                                + folder
                                + signal_name,
                                signal_value,
                                msg_ts_ms,
                            )
                    except KeyError:
                        pass  # Skip CAN IDs not in the DBC

                    payload_cursor += message_size
        except (IndexError, struct.error, ValueError) as e:
            print(f"Finished parsing or encountered an error: {e}")
            break
    log.close()
    progressbar.close()


def check_revlog_file(path):
    if not os.path.isfile(path):
        raise argparse.ArgumentTypeError("Input file not found")
    if not path.lower().endswith(".revlog"):
        raise argparse.ArgumentTypeError("Input file must be a .revlog file")
    return path


def check_output_dir(path):
    if not os.path.isdir(path):
        raise argparse.ArgumentTypeError("Output directory not found")
    return path


def main():
    parser = argparse.ArgumentParser(
        prog="revlog-converter",
        description="Converts a REV binary log (.revlog) to WPILOG (.wpilog)",
        epilog="Copyright (C) 2025 REV Robotics",
    )
    parser.add_argument(
        "revlog_file", type=check_revlog_file, help="Path to input .revlog file."
    )
    parser.add_argument(
        "output_dir", type=check_output_dir, help="Output .wpilog directory"
    )

    if len(sys.argv) == 1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    args = parser.parse_args()

    log_file = args.revlog_file
    log_dest = args.output_dir
    basename = os.path.basename(log_file)
    log_name, extension = os.path.splitext(basename)
    parse_log_file(log_file, log_dest, log_name)


if __name__ == "__main__":
    main()

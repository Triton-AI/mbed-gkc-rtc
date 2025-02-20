#!/usr/bin/env python3
import serial
import struct
import time
import random

# Packet framing constants
START_BYTE = 0x02
END_BYTE   = 0x03

# CRC16 lookup table (256 entries, matching the C++ implementation)
crc16_tab = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
]

def calc_crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = crc16_tab[((crc >> 8) ^ byte) & 0xFF] ^ ((crc << 8) & 0xFFFF)
    return crc & 0xFFFF

def build_packet(payload: bytes) -> bytes:
    length = len(payload)
    crc = calc_crc16(payload)
    # Packet format: START, length (1 byte), payload, CRC16 (little-endian), END.
    return bytes([START_BYTE, length]) + payload + struct.pack("<H", crc) + bytes([END_BYTE])

def send_handshake(ser: serial.Serial) -> int:
    # Handshake #1: payload first byte is 0x04 followed by a 4-byte sequence number.
    seq = random.randint(0, 0xFFFFFFFF)
    payload = bytes([0x04]) + struct.pack("<I", seq)
    packet = build_packet(payload)
    print("Sending handshake packet:")
    print("  Packet (hex):", packet.hex())
    ser.write(packet)
    return seq

def read_packet(ser: serial.Serial, timeout: float = 5.0) -> bytes:
    """Reads from the serial port until a complete packet is received or timeout occurs."""
    start_time = time.time()
    buffer = bytearray()
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            buffer.extend(ser.read(ser.in_waiting))
            # Look for a complete packet in the buffer.
            while True:
                if len(buffer) < 5:
                    break  # Not enough bytes for a minimal packet.
                if buffer[0] != START_BYTE:
                    buffer.pop(0)
                    continue
                length = buffer[1]
                expected_len = 1 + 1 + length + 2 + 1  # start + len + payload + crc + end
                if len(buffer) < expected_len:
                    break  # Wait for more bytes.
                if buffer[expected_len - 1] != END_BYTE:
                    # Malformed packet; remove the start byte and try again.
                    buffer.pop(0)
                    continue
                # We have a complete packet.
                packet = bytes(buffer[:expected_len])
                del buffer[:expected_len]
                return packet
        time.sleep(0.1)
    return None

def parse_packet(packet: bytes) -> bytes:
    """Parses the packet and returns the payload if CRC checks out."""
    if len(packet) < 5:
        return None
    # Packet structure: [START, length, payload, crc (2 bytes), END]
    length = packet[1]
    payload = packet[2:2+length]
    crc_received = struct.unpack("<H", packet[2+length:2+length+2])[0]
    crc_calculated = calc_crc16(payload)
    if crc_received != crc_calculated:
        print("CRC mismatch: received 0x{:04X}, calculated 0x{:04X}".format(crc_received, crc_calculated))
    return payload

def main():
    # Change port to your serial port
    port = "/dev/ttyUSB0"
    baudrate = 115200

    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
    except serial.SerialException as e:
        print("Could not open serial port:", e)
        return

    # Wait a moment for the port to settle.
    time.sleep(2)
    
    # Send handshake #1
    seq = send_handshake(ser)
    print("Handshake sent with sequence number:", seq)
    
    print("Waiting for response from device...")
    pkt = read_packet(ser, timeout=5.0)
    if pkt:
        payload = parse_packet(pkt)
        print("Received packet (payload hex):", payload.hex())
        # For handshake #2, the payload should begin with 0x05 and then contain a 4-byte number.
        if payload[0] == 0x05:
            seq_resp = struct.unpack("<I", payload[1:5])[0]
            print("Handshake #2 received with sequence:", seq_resp)
            expected = (~seq) & 0xFFFFFFFF
            if seq_resp == expected:
                print("Handshake successful!")
            else:
                print("Unexpected handshake response (expected complement of 0x{:08X})".format(seq))
        else:
            print("Received unexpected packet type.")
    else:
        print("No packet received within timeout.")

    ser.close()

if __name__ == "__main__":
    main()

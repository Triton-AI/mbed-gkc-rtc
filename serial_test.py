import serial
import struct
import time

class PacketType:
    """Packet types based on the console UI"""
    HANDSHAKE1 = 0x04
    HANDSHAKE2 = 0x05
    GET_FIRMWARE = 0x06
    FIRMWARE_VERSION = 0x07
    RESET_RTC = 0xFF
    HEARTBEAT = 0xAA
    CONFIG = 0xA0
    STATE_TRANSITION = 0xA1
    CONTROL = 0xAB
    SENSOR = 0xAC
    SHUTDOWN1 = 0xA2
    SHUTDOWN2 = 0xA3
    LOG = 0xAD
    RC_CONTROL = 0xAE

class GokartController:
    """Controller for serial communication with the gokart"""
    
    # State codes
    UNINITIALIZED = 0
    INITIALIZING = 1
    INACTIVE = 2
    ACTIVE = 3
    EMERGENCY_STOP = 255
    
    def __init__(self, port='/dev/cu.usbserial-120', baudrate=115200):
        """Initialize serial connection to gokart"""
        self.ser = serial.Serial(port, baudrate)
        self.seq_number = 0
        
    def calc_crc16(self, payload):
        """Calculate CRC16 for packet validation using the CCITT-FALSE algorithm"""
        crc16_tab = [
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
            0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
            0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
            0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
            0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
            0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
            0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
            0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
            0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
            0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
            0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
            0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
            0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
            0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
            0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
            0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
            0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
            0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
            0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
            0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
            0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657,
            0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
            0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
            0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
            0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
            0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
            0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d,
            0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
            0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
        ]
        checksum = 0
        for byte in payload:
            checksum = crc16_tab[((checksum >> 8) ^ byte) & 0xFF] ^ (checksum << 8)
            checksum &= 0xFFFF
            
        checksum_bytes = struct.pack("<H", checksum)
        return checksum_bytes

    def send_packet(self, payload):
        """Send a packet with proper formatting"""
        # Construct packet with payload length as the size byte
        packet = bytearray([0x02, len(payload)])  # Start byte and size byte
        packet.extend(payload)  # Add payload
        packet.extend(self.calc_crc16(payload))  # Add CRC16
        packet.append(0x03)  # End byte
        
        print(f"Sending packet: {packet.hex()}")
        self.ser.write(packet)
        return self.read_response()
        
    def read_response(self, timeout=1.0):
        """Read response from gokart with timeout"""
        start_time = time.time()
        response = b''
        buffer = bytearray()
        packets = []
        
        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                buffer.extend(data)
                response += data
                
                # Process all complete packets in the buffer
                while True:
                    # Find start byte
                    try:
                        start_idx = buffer.index(0x02)
                        if start_idx > 0:
                            # Discard data before start byte
                            buffer = buffer[start_idx:]
                    except ValueError:
                        # No start byte found
                        break
                    
                    # Check if we have enough bytes for a minimal packet
                    if len(buffer) < 5:  # START + SIZE + MIN_PAYLOAD(1) + CRC(2) + END
                        break
                    
                    # Get payload size
                    payload_size = buffer[1]
                    
                    # Check if we have enough bytes for the full packet
                    packet_size = payload_size + 5  # START + SIZE + PAYLOAD + CRC(2) + END
                    if len(buffer) < packet_size:
                        break
                    
                    # Check if end byte is correct
                    if buffer[packet_size-1] != 0x03:
                        buffer = buffer[1:]  # Remove start byte and try again
                        continue
                    
                    # Extract complete packet
                    packet = bytes(buffer[:packet_size])
                    packets.append(packet)
                    
                    # Extract payload for decoding
                    payload = buffer[2:2+payload_size]
                    self.decode_packet(payload)
                    
                    # Remove the processed packet from buffer
                    buffer = buffer[packet_size:]
                    
                    if not buffer:
                        break
            
            time.sleep(0.01)
        
        return response

    def decode_packet(self, payload):
        """Decode the payload and print useful information"""
        if not payload:
            return
            
        packet_type = payload[0]
        
        if packet_type == PacketType.HEARTBEAT and len(payload) >= 3:
            counter = payload[1]
            state = payload[2]
            state_name = "UNKNOWN"
            
            if state == self.UNINITIALIZED:
                state_name = "UNINITIALIZED"
            elif state == self.INITIALIZING:
                state_name = "INITIALIZING"
            elif state == self.INACTIVE:
                state_name = "INACTIVE"
            elif state == self.ACTIVE:
                state_name = "ACTIVE"
            elif state == self.EMERGENCY_STOP:
                state_name = "EMERGENCY_STOP"
                
            print(f"Decoded Heartbeat: counter={counter}, state={state_name}")
            
        elif packet_type == PacketType.FIRMWARE_VERSION and len(payload) >= 4:
            major = payload[1]
            minor = payload[2]
            patch = payload[3]
            print(f"Decoded Firmware Version: {major}.{minor}.{patch}")
            
        elif packet_type == PacketType.LOG and len(payload) >= 3:
            severity = payload[1]
            try:
                message = payload[2:].decode('utf-8', errors='replace')
                print(f"Decoded Log: severity={severity}, message={message}")
            except:
                print(f"Decoded Log: severity={severity}, message=<binary data>")
    
    def send_handshake(self):
        """Send handshake packet and wait for response"""
        print("Initiating handshake...")
        self.seq_number += 1
        # Handshake1 packet format
        handshake_payload = bytearray([PacketType.HANDSHAKE1])
        handshake_payload.extend(struct.pack("<I", self.seq_number))
        response = self.send_packet(handshake_payload)
        print(f"Handshake response: {response.hex()}")
        return response
    
    def send_config(self):
        """Send configuration packet"""
        print("Sending configuration...")
        # Config packet format
        config_payload = bytearray([PacketType.CONFIG])
        config_payload.extend(struct.pack("<IIIIII", 
            1000,   # DEFAULT_MCU_HEARTBEAT_INTERVAL_MS
            2000,   # DEFAULT_MCU_HEARTBEAT_LOST_TOLERANCE_MS
            1000,   # DEFAULT_PC_HEARTBEAT_INTERVAL_MS
            2000,   # DEFAULT_PC_HEARTBEAT_LOST_TOLERANCE_MS
            10,     # DEFAULT_CTL_CMD_INTERVAL_MS
            200     # DEFAULT_CTL_CMD_LOST_TOLERANCE_MS
        ))
        response = self.send_packet(config_payload)
        print(f"Config response: {response.hex()}")
        return response
    
    def send_state_transition(self, target_state):
        """Request state transition"""
        print(f"Requesting state transition to {target_state}...")
        state_trans_payload = bytearray([PacketType.STATE_TRANSITION, target_state])
        response = self.send_packet(state_trans_payload)
        print(f"State transition response: {response.hex()}")
        return response
    
    def send_control(self, throttle, steering, brake):
        """Send control commands to the gokart
        
        Args:
            throttle: Float value [-1.0 to 1.0] for throttle
            steering: Float value in radians
            brake: Float value [0.0 to 1.0] for brake pressure
        """
        print(f"Sending control: throttle={throttle}, steering={steering}, brake={brake}")
        
        # Control packet format
        control_payload = bytearray([PacketType.CONTROL])
        control_payload.extend(struct.pack("<f", throttle))
        control_payload.extend(struct.pack("<f", steering))
        control_payload.extend(struct.pack("<f", brake))
        
        # Send packet
        response = self.send_packet(control_payload)
        print(f"Control response: {response.hex() if response else 'None'}")
        return response
    
    def get_firmware_version(self):
        """Request firmware version"""
        print("Requesting firmware version...")
        firmware_payload = bytearray([PacketType.GET_FIRMWARE])
        response = self.send_packet(firmware_payload)
        print(f"Firmware version response: {response.hex()}")
        return response
    
    def initialize_system(self):
        """Complete initialization sequence"""
        print("Starting initialization sequence...")
        
        # Step 1: Handshake
        self.send_handshake()
        time.sleep(0.5)
        
        # Step 2: Send configuration
        self.send_config()
        time.sleep(0.5)
        
        # Step 3: Request transition to Inactive state first
        self.send_state_transition(self.INACTIVE)
        time.sleep(0.5)
        
        # Step 4: Request transition to Active state
        self.send_state_transition(self.ACTIVE)
        time.sleep(0.5)
        
        print("Initialization complete!")

    def emergency_stop(self):
        """Trigger emergency stop"""
        print("EMERGENCY STOP!")
        self.send_state_transition(self.EMERGENCY_STOP)
        
    def send_heartbeat(self):
        """Send heartbeat packet"""
        print("Sending heartbeat...")
        self.rolling_counter = (getattr(self, 'rolling_counter', 0) + 1) % 256
        heartbeat_payload = bytearray([PacketType.HEARTBEAT, self.rolling_counter, self.ACTIVE])
        response = self.send_packet(heartbeat_payload)
        print(f"Heartbeat response: {response.hex()}")
        return response
        
    def close(self):
        """Close serial connection"""
        self.ser.close()

# Example usage
if __name__ == "__main__":
    controller = GokartController()
    
    try:
        # Initialize the system
        controller.initialize_system()
        
        # Get firmware version
        controller.get_firmware_version()
        time.sleep(0.5)
        
        # Send heartbeat to maintain connection
        controller.send_heartbeat()
        time.sleep(0.5)
        
        # Drive forward slowly
        controller.send_control(0.1, 0.0, 0.0)  # 10% throttle, neutral steering, no brake
        time.sleep(2)  # Drive for 2 seconds
        
        # Turn right while driving
        controller.send_control(0.1, 0.1, 0.0)  # 10% throttle, right turn, no brake
        time.sleep(2)  # Drive for 2 seconds

        # Turn left while driving
        controller.send_control(0.1, -0.1, 0.0)  # 10% throttle, left turn, no brake
        time.sleep(2)  # Drive for 2 seconds
        
        # Apply brake
        controller.send_control(0.0, 0.0, 0.0)  # No throttle, neutral steering, no brake
        time.sleep(1)  # Brake for 1 second
        
        # Emergency stop
        controller.emergency_stop()
    
    except KeyboardInterrupt:
        print("Interrupted by user")
        controller.emergency_stop()
    
    finally:
        controller.close()
        print("Connection closed")
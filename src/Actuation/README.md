# VESC CAN Tools

## Overview
This module provides a comprehensive interface for communicating with VESC motor controllers over the CAN bus protocol. It includes functions for setting motor duties, currents, positions, RPMs, and other control operations.

## Functionality

### CAN Transmission
- `CanTransmitEid`: Transmits a CAN message with an extended ID

### Buffer Manipulation
- `BufferAppendInt16/Int32`: Append integers to buffers in big-endian format
- `BufferAppendFloat16/Float32`: Append scaled floats as integers to buffers

### Motor Control Functions
- `CommCanSetDuty`: Set motor duty cycle
- `CommCanSetCurrent`: Set motor current
- `CommCanSetCurrentBrake`: Set motor brake current
- `CommCanSetRpm`: Set motor RPM
- `CommCanSetPos`: Set motor position
- `CommCanSetCurrentRel`: Set relative motor current
- `CommCanSetCurrentOffDelay`: Set motor current with off delay
- `CommCanSetHandbrake`: Set handbrake current

### Vehicle-Specific Functions
- `CommCanSetSpeed`: Converts linear speed (m/s) to motor RPM
- `CommCanSetAngle`: Converts steering angle to motor position
- `CommCanSetBrakePosition`: Sets brake position as normalized value

### Utility Functions
- `Clamp`: Constrains a value between min and max
- `MapRange`: Maps a value from one range to another
- `MapSteer2Motor`: Maps steering angle to motor angle using a lookup table

## Implementation Notes

1. **CAN Message Format**:
   - Each message uses an extended ID format
   - The packet type is shifted 8 bits and OR'd with controller ID
   - Data is big-endian encoded

2. **Scaling Constants**:
   - Duty cycle: scaled by 100,000
   - Current: scaled by 1,000
   - Relative values: scaled by 1e5
   - Position: scaled by 1,000,000 (and inverted)

3. **Vehicle Constants**:
   - Motor poles: 5.0
   - Gear ratio: 59.0/22.0
   - Wheel circumference: 0.85m

4. **Memory Management**:
   - CAN messages are dynamically allocated but always freed
   - Static buffer used for brake position commands
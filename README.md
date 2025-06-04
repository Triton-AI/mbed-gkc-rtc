# Gokart Controller Documentation

## Overview

The Triton AI Gokart Controller is an embedded software project built on Mbed OS for controlling an autonomous go-kart. It supports multiple control modes including manual RC control, autonomous override, and full autonomous operation. The system integrates communication, sensor reading, state management, CAN-based actuation, performance profiling, and comprehensive safety through a watchdog system.

> **Note:** This project is designed to run on STM32 Nucleo boards and relies on Mbed OS APIs.

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Hardware Setup](#hardware-setup)
- [Modules and Components](#modules-and-components)
- [Development Environment](#development-environment)
- [Building and Running](#building-and-running)
- [Testing](#testing)
- [Configuration](#configuration)
- [Future Work](#future-work)

## Project Structure

```
src/
├── Actuation/
│   ├── actuation_controller.cpp/hpp
│   ├── vesc_can_tools.cpp/hpp
│   └── README.md
├── Comm/
│   ├── comm.cpp/hpp
├── Controller/
│   ├── controller.cpp/hpp
├── main.cpp
├── RCController/
│   ├── rc_controller.cpp/hpp
├── Sensor/
│   ├── brake_pressure_sensor.cpp/hpp
│   ├── can_sensor_provider.cpp/hpp
│   └── sensor_reader.cpp/hpp
├── StateMachine/
│   ├── state_machine.cpp/hpp
├── Tools/
│   ├── global_profilers.hpp
│   ├── logger.hpp
│   └── profiler.hpp
├── USBJoystick/
│   ├── usb_joystick.cpp/hpp
└── Watchdog/
    ├── watchable.hpp
    ├── watchdog.cpp/hpp
    └── README.md

lib/
├── elrs_receiver/
├── PwmIn/
├── QEI/
└── tai_gokart_packet/

include/
├── config.hpp
└── config_old.hpp

Design/
├── gkc_state_machine.png
└── state_machine.md
```

## Hardware Setup

### Supported Boards
- **Primary**: Nucleo-H723ZG
- **Alternative**: Nucleo-H743ZI2, Nucleo-F767ZI

### Board Fuses
The breakout board includes protection fuses that may blow during development. If a fuse blows, you can bridge it with solder to restore functionality. There are 3 fuses located next to the 12V, 5V, and 3.3V inputs.

### Key Hardware Interfaces

**Communication:**
- **USB Device Port**: Required connection to onboard computer (system waits for this connection when controller passthrough is enabled)
- **UART Serial**: Debug/direct control via breakout board (PB_12/PB_13)
- **ELRS Receiver**: ExpressLRS RC receiver on UART7 (PE_7/PE_8)

**Actuation:**
- **CAN Bus 2**: Motor controllers (PB_5/PB_6 at 500kbaud)
- **VESC Disable Pins**: Safety disable for throttle (PD_14) and steering (PD_12)
- **Brake Pressure**: Analog brake control

**Indicators:**
- **Tower Lights**: RGB status indicators (Red: PD_15, Yellow: PD_11, Green: PE_12)
- **Onboard LEDs**: System status indication

## Modules and Components

### Main Application

**main.cpp** serves as the entry point:
- Sets up passthrough button (BUTTON1) for toggling USB joystick mode
- Instantiates the main Controller object
- Runs an infinite sleep loop

```cpp
bool g_PassthroughEnabled = false;

void TogglePassthrough() {
    g_PassthroughEnabled = !g_PassthroughEnabled;
}

int main() {
    button.rise(&TogglePassthrough);
    new tritonai::gkc::Controller();
    while (true) {
        ThisThread::sleep_for(3600000ms);
    }
}
```

### State Machine

The **GkcStateMachine** manages the system lifecycle through these states:
- **Uninitialized** (0) – Default startup state
- **Initializing** (1) – System startup in progress  
- **Inactive** (2) – System ready but not actively controlling
- **Active** (3) – Normal operational state
- **Emergency** (255) – Safety-critical emergency state

Child classes implement state-specific callbacks: `OnInitialize`, `OnActivate`, `OnDeactivate`, `OnEmergencyStop`, and `OnReinitialize`.

### Communication Manager

**CommManager** handles packet-based communication over UART:
- Asynchronous send/receive using dedicated threads
- Packet queuing system with configurable queue size
- Integration with the GKC packet protocol
- Automatic packet validation and CRC checking

### Controller (Main System Controller)

The **Controller** class is the central coordinator, implementing multiple interfaces:
- **GkcPacketSubscriber**: Receives and processes all packet types
- **Watchable**: Monitored by the watchdog system
- **ILogger**: Provides system-wide logging functionality  
- **GkcStateMachine**: Manages system lifecycle states

**Key responsibilities:**
- Coordinates all subsystems (communication, sensors, actuation, RC)
- Manages state transitions and safety protocols
- Processes control commands from both autonomous systems and RC
- Implements emergency stop and recovery procedures
- Controls visual status indicators (tower lights, LEDs)

### RC Controller

**RCController** processes ExpressLRS (ELRS) remote control inputs:
- Reads 16-channel CRSF protocol data from ELRS receiver
- Normalizes and maps analog stick/switch positions to vehicle commands
- Supports three autonomy modes: Manual, Autonomous Override, Autonomous
- Implements emergency stop through dual safety switches
- Optional USB HID joystick passthrough for testing/simulation

**Channel Mapping:**
- Channel 1: Throttle
- Channel 3: Steering  
- Channels 4 & 7: Emergency stop switches (both must be active)
- Channels 5 & 6: Mode selection switches
- Channel 8: Throttle ratio adjustment
- Channel 9: Special functions

### USB Joystick (Optional)

**USBJoystick** provides HID joystick functionality when enabled:
- 2-axis joystick (X/Y for steering/throttle)
- 3 configurable buttons
- 1 slider/dial for throttle ratio
- Activated via board button when in autonomous mode
- Useful for testing and simulation

### Sensor Reader

**SensorReader** manages multiple sensor providers through a plugin architecture:
- **ISensorProvider** interface for modular sensor integration
- Thread-safe provider registration/removal
- Configurable polling intervals
- Automatic sensor data aggregation into unified packets

**Current sensor providers:**
- **BrakePressureSensor**: Analog brake pressure (0-1000 PSI)
- **CanSensorProvider**: Steering angle and speed feedback via CAN

### Actuation Controller & VESC CAN Tools

**ActuationController** commands vehicle actuators through CAN bus:
- **Throttle**: Speed control via VESC motor controller
- **Steering**: Position control via VESC servo controller  
- **Brake**: PWM-based brake actuator control

**VESC CAN Tools** provide comprehensive motor controller interface:
- Bidirectional CAN communication (CAN2 at 500kbaud)
- Motor control: duty cycle, current, RPM, position commands
- Feedback: steering angle, speed (ERPM), motor status
- Safety features: brake current, handbrake, disable pins

### Logger and Profilers

**ILogger** interface with severity levels (DEBUG, INFO, WARNING, ERROR, FATAL):
- Console output with severity-based filtering
- Integration with packet-based logging system
- Centralized logging for all system components

**Profiler** classes provide performance monitoring:
- Microsecond-precision timing measurements
- Rolling average calculations for stability
- Named profiler instances for different system sections
- Dump functionality for performance analysis

### Watchdog & Safety System

**Watchdog** monitors all critical system components:
- **Watchable** interface for components requiring monitoring
- Configurable timeouts per component
- Automatic system reset on component failures
- Thread-safe activity monitoring

**Monitored components:**
- Controller, CommManager, SensorReader, RCController
- RC heartbeat (emergency stop on RC disconnection)

## Development Environment

### PlatformIO Setup

1. **Install Dependencies:**
   ```bash
   # Install VS Code and PlatformIO extension
   code --install-extension platformio.platformio-ide
   ```

2. **Open Project:**
   - Launch VS Code
   - Open the project folder
   - PlatformIO will automatically detect `platformio.ini`

3. **Build Targets:**
   - Primary: `nucleo_h723zg`
   - Alternative: `nucleo_h743zi2`, `nucleo_f767zi`

### Platform Compatibility

- **Linux**: Recommended development platform (tested on Ubuntu)
- **macOS**: Supported (note: USB device names may change between uses)
- **Windows**: Experimental (consider using WSL with USB passthrough)

## Building and Running

### Configuration

Edit `include/config.hpp` for your specific setup:

```cpp
// Enable USB joystick feature (optional)
#define ENABLE_USB_PASSTHROUGH

// Communication settings
#define BAUD_RATE 115200
#define CAN2_BAUDRATE 500000

// Vehicle limits
#define THROTTLE_MAX_FORWARD_SPEED 20.0f  // m/s
#define RC_MAX_SPEED_FORWARD 5.0f         // m/s for RC mode
```

### Build Process

```bash
# Build for primary target
pio run -e nucleo_h723zg

# Upload to board
pio run -e nucleo_h723zg -t upload

# Monitor serial output
pio device monitor
```

### System Startup

1. **Power on** the Nucleo board
2. **Connect USB** to onboard computer (required for initialization, when using passthrough)
3. **Check LEDs**: LED3 indicates system state (off = active, on = inactive/error)
4. **Tower lights** show operational status and RC connection state

## Testing

### Serial Control Testing

Use the included Python test script for direct serial communication:

```bash
# Install dependencies
pip install pyserial

# Basic test sequence
python serial_test.py --port /dev/ttyUSB0 --debug

# Custom speed test  
python serial_test.py --port /dev/ttyUSB0 --speed 3.0 --duration 30
```

**Script features:**
- Automatic handshake and initialization
- Control command sequences (forward, turn, brake)
- Debug logging with packet analysis
- Emergency stop capability

### RC Control Testing

1. **Bind ELRS transmitter** to receiver
2. **Test emergency stops**: Both switches must be active for operation
3. **Verify modes**: Manual, Autonomous Override, Autonomous
4. **USB passthrough**: Press board button when in autonomous mode

## Configuration

### Key Configuration Parameters

**Communication:**
```cpp
#define UART_TX_PIN PB_13              // Debug serial
#define REMOTE_UART_TX_PIN PE_7        // ELRS receiver
#define CAN2_RX PB_5                   // Motor CAN bus
```

**Safety & Timing:**
```cpp
#define DEFAULT_RC_HEARTBEAT_LOST_TOLERANCE_MS 500    // RC timeout
#define EMERGENCY_BRAKE_PRESSURE 1.0f                // Emergency brake force
#define DEFAULT_WD_MAX_INACTIVITY_MS 3000             // Watchdog timeout
```

**Vehicle Parameters:**
```cpp
#define WHEEL_DIAMETER_M 0.254f        // Wheel size for speed calculation
#define STEERING_RATIO 4.0f            // Steering gear ratio
#define GEAR_RATIO (59.0/22.0)         // Motor gear ratio
```

### Autonomy Modes

1. **MANUAL (0)**: RC control only, ignores autonomous commands
2. **AUTONOMOUS_OVERRIDE (1)**: Autonomous control with RC override capability
3. **AUTONOMOUS (2)**: Full autonomous control, RC only for emergency stop

### Tower Light Status Indicators

- **Red Flashing**: Emergency stop active
- **Yellow Flashing**: RC disconnected or unknown state
- **Red Solid**: Manual mode
- **Yellow Solid**: Autonomous override mode  
- **Green Solid**: Full autonomous mode
- **Orange (Red+Yellow)**: USB passthrough mode

## Future Work

### Critical Safety Issues
- **Emergency brake reliability**: Investigate rare cases where controller failure doesn't stop vehicle
- **Controller noise immunity**: Improve resilience to electrical interference
- **USB passthrough stability**: Fix disconnection issues with onboard computer

### Performance Improvements
- **PID steering control**: Implement closed-loop steering with encoder feedback
- **Brake pressure feedback**: Add pressure sensor feedback loop for precise braking
- **ABS implementation**: Anti-lock braking using accelerometer feedback
- **Predictive control**: Implement model-predictive control for smoother operation

### Code Quality & Architecture
- **Smart pointers**: Replace raw pointers with std::unique_ptr/shared_ptr
- **RAII lifecycle**: Implement proper system start/run/stop lifecycle
- **Thread-safe logging**: Eliminate potential race conditions in logging system
- **Profiler validation**: Complete testing and validation of performance profiling system
- **Unit testing**: Add comprehensive unit test suite for critical components

### Additional Features
- **Telemetry system**: Real-time data streaming to ground station
- **Configuration manager**: Runtime parameter adjustment without recompilation
- **Diagnostic system**: Built-in self-test and diagnostic capabilities
- **Multi-vehicle support**: Support for multiple vehicles with different configurations
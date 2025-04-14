# Mbed Project Coding Standards

This document outlines the coding standards and guidelines for our Mbed-based embedded project. Consistent coding practices help maintain code quality, readability, and collaboration efficiency.

## Table of Contents
- [Naming Conventions](#naming-conventions)
- [Code Formatting](#code-formatting)
- [Memory Management](#memory-management)
- [Error Handling](#error-handling)
- [Documentation](#documentation)
- [Hardware Interfacing](#hardware-interfacing)
- [Safety Considerations](#safety-considerations)

## Naming Conventions

### Classes and Types
- Use **PascalCase** for class/struct names: `SensorReader`, `ActuationController`
- Interface classes should be prefixed with **I**: `ILogger`, `ISensorProvider`
- Enums should be in **PascalCase** and enum values in **ALL_CAPS**: `GkcLifecycle::ACTIVE`

### Variables
- Member variables use `m_` prefix: `m_Logger`, `m_IsActive`
- Local variables use camelCase: `sensorValue`, `timerCount`
- Static class variables use `s_` prefix: `s_Instance`
- Constants and defines use ALL_CAPS: `MAX_BUFFER_SIZE`, `DEFAULT_TIMEOUT_MS`
- Boolean variables should indicate state: `m_IsConnected`, `hasData`

### Functions
- Use **PascalCase** for function names: `ReadSensor()`, `CalculatePosition()`
- Getters use **Get** prefix: `GetValue()`, `GetState()`
- Setters use **Set** prefix: `SetValue()`, `SetState()`
- Boolean queries use **Is/Has/Can** prefix: `IsActive()`, `HasConnection()`

## Code Formatting

### Indentation and Spacing
- Use **4 spaces** for indentation (not tabs)
- One space after keywords like `if`, `for`, `while`
- One space around operators: `a = b + c`, not `a=b+c`
- No space between function name and parenthesis: `DoSomething()`, not `DoSomething ()`

### Braces
- Opening brace on the same line as the statement:
```cpp
if (condition) {
    DoSomething();
}
```
- Always use braces, even for single-line blocks

### Line Length
- Maximum line length: 100 characters
- Break lines after operators when wrapping

## Memory Management

### Resource Allocation
- Avoid dynamic memory allocation when possible
- If necessary, allocate memory early in initialization
- Never allocate memory in interrupt context
- Always check allocation success: `if (buffer != nullptr)`

### Memory Efficiency
- Use static allocation for fixed-size buffers
- Consider memory alignment requirements for hardware access
- Prefer stack allocation for small, temporary objects
- Use appropriate data types (e.g., `uint8_t` instead of `int` for small values)

## Error Handling

### Error Codes
- Use clear, consistent error codes/enums
- Return error codes for conditions that can be handled
- Document all possible return values

### Critical Errors
- Centralize system reset handling with `NVIC_SystemReset()`
- Log critical errors before reset
- Consider implementing error recovery for non-fatal errors

### Watchdog Pattern
- Implement watchdog checks for critical components
- Centralize watchdog handling in dedicated classes
- Document expected timing guarantees

## Documentation

### File Headers
```cpp
/**
 * @file SensorReader.hpp
 * @brief Handles reading and processing of sensor data
 * @author [Author Name]
 * @date YYYY-MM-DD
 * 
 * @copyright Copyright [Year] [Organization]
 */
```

### Function Documentation
```cpp
/**
 * @brief Reads data from the specified sensor
 * @param sensorId The ID of the sensor to read
 * @param buffer Pointer to buffer where data will be stored
 * @param size Size of the buffer in bytes
 * @return Number of bytes read or negative error code
 */
int32_t ReadSensorData(uint8_t sensorId, uint8_t* buffer, size_t size);
```

### Class Documentation
```cpp
/**
 * @class SensorReader
 * @brief Manages periodic reading from multiple sensors
 * 
 * This class handles sensor polling, data processing, and
 * watchdog functionality to ensure sensor data reliability.
 */
```

## Hardware Interfacing

### Hardware Abstraction
- Create clear abstractions for hardware interfaces
- Use platform-specific code only in dedicated files/classes
- Document hardware dependencies and requirements

### Interrupts
- Keep ISRs (Interrupt Service Routines) as short as possible
- Use volatile for variables shared between ISRs and main code
- Document timing constraints for interrupt handlers
- Never use blocking calls in interrupt context

### Pin Management
- Document all pin assignments in a central location
- Use named constants for pin definitions
- Consider creating pin configuration structures

## Safety Considerations

### Critical Sections
- Minimize time spent in critical sections
- Document thread safety for shared resources
- Use appropriate synchronization primitives

### Timeouts
- Implement timeouts for all blocking operations
- Use the watchdog timer for system-level safety
- Document timeout behavior and recovery procedures

### Defensive Programming
- Validate all inputs, especially from external sources
- Check ranges before indexing arrays
- Verify preconditions at function entry points
- Use assertions to catch programming errors

## Build and Configuration

### Conditional Compilation
- Use #ifdef guards for platform-specific code
- Create clear configuration parameters for different build targets
- Document all configuration options

### Dependencies
- Minimize external dependencies
- Document version requirements for all dependencies
- Maintain compatibility with the Mbed ecosystem

## Version Control Practices

### Commit Messages
- Use descriptive commit messages with prefixes:
  - `[FIX]` for bug fixes
  - `[FEAT]` for new features
  - `[DOC]` for documentation updates
  - `[REFACTOR]` for code refactoring
  - `[TEST]` for test-related changes

### Branching Strategy
- `main` branch should always be stable and buildable
- Use feature branches for development
- Perform code reviews before merging

---

## Implementation Examples

### Class Definition
```cpp
/**
 * @class SensorReader
 * @brief Manages sensor data acquisition
 */
class SensorReader : public Watchable {
public:
    /**
     * @brief Constructor
     * @param logger Pointer to logger instance
     */
    SensorReader(ILogger* logger);
    
    /**
     * @brief Registers a new sensor provider
     * @param provider Pointer to sensor provider
     */
    void RegisterProvider(ISensorProvider* provider);
    
    /**
     * @brief Gets the latest sensor readings
     * @return The sensor packet with readings
     */
    const SensorPacket& GetPacket() const { return m_Packet; }

private:
    void SensorPollThreadImpl();
    void WatchdogCallback();

    ILogger* m_Logger;
    SensorPacket m_Packet{};
    std::vector<ISensorProvider*> m_Providers{};
    Mutex m_ProvidersLock;
    Thread m_SensorPollThread;
};
```

### Implementation
```cpp
SensorReader::SensorReader(ILogger* logger)
    : Watchable(DEFAULT_SENSOR_POLL_INTERVAL_MS,
                DEFAULT_SENSOR_POLL_LOST_TOLERANCE_MS,
                "SensorReader"),
      m_Logger(logger) 
{
    m_SensorPollThread.start(callback(this, &SensorReader::SensorPollThreadImpl));
    m_Logger->SendLog(LogPacket::Severity::INFO, "SensorReader initialized");
    Attach(callback(this, &SensorReader::WatchdogCallback));
}

void SensorReader::WatchdogCallback() 
{
    m_Logger->SendLog(LogPacket::Severity::FATAL, "SensorReader timeout detected");
    NVIC_SystemReset();
}
```

### Header Guard Example
```cpp
#ifndef SENSOR_READER_HPP_
#define SENSOR_READER_HPP_

// Contents of the header file...

#endif // SENSOR_READER_HPP_
```

### Class With Interface Example
```cpp
/**
 * @class ISensorProvider
 * @brief Interface for sensor data providers
 */
class ISensorProvider {
public:
    virtual ~ISensorProvider() = default;

    /**
     * @brief Checks if sensor data is ready to be read
     * @return True if data is ready
     */
    virtual bool IsReady() = 0;

    /**
     * @brief Populates sensor packet with readings
     * @param packet Packet to populate with data
     */
    virtual void PopulateReading(SensorPacket& packet) = 0;
};

/**
 * @class TemperatureSensor
 * @brief Implementation of temperature sensor provider
 */
class TemperatureSensor : public ISensorProvider {
public:
    TemperatureSensor(uint8_t sensorId);
    
    virtual bool IsReady() override;
    virtual void PopulateReading(SensorPacket& packet) override;

private:
    uint8_t m_SensorId;
    float m_LastReading;
    bool m_IsCalibrated;
};
```
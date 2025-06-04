# Mbed Project Coding Standards

This document outlines the coding standards and guidelines for our Mbed-based embedded project. Consistent coding practices help maintain code quality, readability, and make collaboration more effective for the team.

## Table of Contents
- [Naming Conventions](#naming-conventions)
- [Code Formatting](#code-formatting)
- [Memory Management](#memory-management)
- [Error Handling](#error-handling)
- [Documentation](#documentation)
- [Hardware Interfacing](#hardware-interfacing)
- [Safety Considerations](#safety-considerations)
- [Architecture Patterns](#architecture-patterns)

## Naming Conventions

### Classes and Types
- Use **PascalCase** for class/struct names: `SensorReader`, `ActuationController`, `CommManager`
- Interface classes are prefixed with **I**: `ILogger`, `ISensorProvider`
- Enums use **PascalCase** with enum values in **ALL_CAPS**: `GkcLifecycle::ACTIVE`

### Variables
- Member variables use `m_` prefix: `m_Logger`, `m_State`, `m_IsActive`
- Local variables use camelCase: `sensorValue`, `timerCount`, `packetSize`
- Static class variables use `s_` prefix: `s_Instance`
- Constants and defines use ALL_CAPS: `MAX_BUFFER_SIZE`, `DEFAULT_TIMEOUT_MS`
- Boolean variables should clearly indicate state: `m_IsConnected`, `hasData`, `isReady`

### Functions
- Use **PascalCase** for function names: `ReadSensor()`, `CalculatePosition()`, `SendPacket()`
- Getters use **Get** prefix: `GetValue()`, `GetState()`, `GetPacket()`
- Setters use **Set** prefix: `SetValue()`, `SetState()`, `SetThrottleCmd()`
- Boolean queries use **Is/Has/Can** prefix: `IsActive()`, `HasConnection()`, `CanTransmit()`

## Code Formatting

### Indentation and Spacing
- Use **4 spaces** for indentation (no tabs)
- Add one space after keywords: `if (condition)`, `for (int i = 0; i < count; i++)`
- Add one space around operators: `a = b + c`, `result != nullptr`
- No space between function name and parenthesis: `DoSomething()`, not `DoSomething ()`

### Braces and Structure
- Opening brace on the same line as the statement:
```cpp
if (condition) {
    DoSomething();
}

class SensorReader {
public:
    SensorReader();
    // ...
};
```
- Always use braces, even for single-line blocks, to prevent future errors

### Line Length
- Target maximum line length: 100 characters
- When wrapping lines, break after operators and align parameters naturally

## Memory Management

### Resource Allocation
- Prefer static allocation for embedded systems when possible
- When dynamic allocation is necessary, allocate early during initialization
- Never allocate memory in interrupt context
- Always validate allocation success: `if (buffer != nullptr)`

### Modern C++ Practices
- Use smart pointers (`std::unique_ptr`, `std::shared_ptr`) where appropriate
- Consider RAII (Resource Acquisition Is Initialization) principles
- Use appropriate data types for the context (e.g., `uint8_t` for small values)

## Error Handling

### Error Communication
- Use clear, consistent error codes or enums
- Return error codes for recoverable conditions
- Document all possible return values and their meanings

### Critical Error Management
- Use `NVIC_SystemReset()` for unrecoverable system errors
- Log critical errors before attempting reset when possible
- Consider implementing graceful degradation for non-fatal errors

### Watchdog Integration
- Implement the `Watchable` interface for components requiring monitoring
- Use centralized watchdog handling through the `Watchdog` class
- Document timing requirements and expected behavior

## Documentation

### File Headers
```cpp
/**
 * @file SensorReader.hpp
 * @brief Handles reading and processing of sensor data
 * @author [Author Name]
 * @date YYYY-MM-DD
 * 
 * @copyright Copyright [Year] Triton AI
 */
```

### Function Documentation
```cpp
/**
 * @brief Reads data from the specified sensor
 * @param sensorId The ID of the sensor to read
 * @param buffer Pointer to buffer where data will be stored
 * @param size Size of the buffer in bytes
 * @return Number of bytes read or negative error code on failure
 */
int32_t ReadSensorData(uint8_t sensorId, uint8_t* buffer, size_t size);
```

### Class Documentation
```cpp
/**
 * @class SensorReader
 * @brief Manages periodic reading from multiple sensors
 * 
 * This class coordinates sensor polling, data processing, and
 * watchdog functionality to ensure reliable sensor data availability.
 * It supports multiple sensor providers through the ISensorProvider interface.
 */
```

## Hardware Interfacing

### Hardware Abstraction
- Create clear abstractions for hardware interfaces
- Isolate platform-specific code in dedicated files/classes
- Document hardware dependencies, pin assignments, and timing requirements

### Interrupt Handling
- Keep ISRs (Interrupt Service Routines) as brief as possible
- Use `volatile` for variables shared between ISRs and main code
- Document timing constraints and critical sections
- Avoid blocking operations in interrupt context

### Pin and Resource Management
- Document all pin assignments in `config.hpp`
- Use named constants for pin definitions rather than magic numbers
- Group related configurations logically

## Safety Considerations

### Critical Sections and Threading
- Minimize time spent in critical sections
- Document thread safety requirements for shared resources
- Use appropriate synchronization primitives (`Mutex`, `Semaphore`)

### Defensive Programming
- Validate inputs, especially from external sources
- Check array bounds before indexing
- Verify preconditions at function entry points
- Use assertions for debugging programming errors (not runtime failures)

### Timeout and Recovery
- Implement reasonable timeouts for all blocking operations
- Use the watchdog system for component-level safety
- Document timeout behavior and recovery procedures

## Architecture Patterns

### Component Interfaces
Our codebase uses several key interface patterns:

#### Logger Interface
```cpp
class ILogger {
public:
    virtual void SendLog(const LogPacket::Severity& severity,
                         const std::string& what) = 0;
};
```

#### Sensor Provider Interface
```cpp
class ISensorProvider {
public:
    virtual bool IsReady() = 0;
    virtual void PopulateReading(SensorGkcPacket& packet) = 0;
};
```

#### Watchable Interface
```cpp
class Watchable {
public:
    void IncCount();
    bool CheckActivity();
    void Attach(Callback<void()> func);
    // ...
};
```

### State Machine Pattern
Implement state machines by inheriting from `GkcStateMachine`:

```cpp
class Controller : public GkcStateMachine {
protected:
    StateTransitionResult OnInitialize(const GkcLifecycle& lastState) override;
    StateTransitionResult OnActivate(const GkcLifecycle& lastState) override;
    StateTransitionResult OnDeactivate(const GkcLifecycle& lastState) override;
    StateTransitionResult OnEmergencyStop(const GkcLifecycle& lastState) override;
    StateTransitionResult OnReinitialize(const GkcLifecycle& lastState) override;
};
```

## Build and Configuration

### Conditional Compilation
- Use feature flags for optional functionality: `#ifdef ENABLE_USB_PASSTHROUGH`
- Document all configuration options in `config.hpp`
- Provide sensible defaults for optional features

### Dependencies
- Minimize external dependencies when possible
- Document version requirements for any dependencies
- Maintain compatibility with the Mbed ecosystem

## Version Control Best Practices

### Commit Messages
Use descriptive commit messages with clear prefixes:
- `[FIX]` for bug fixes
- `[FEAT]` for new features  
- `[DOC]` for documentation updates
- `[REFACTOR]` for code refactoring
- `[TEST]` for test-related changes

### Code Review
- Keep the `main` branch stable and buildable
- Use feature branches for development
- Conduct code reviews focusing on safety, clarity, and maintainability

## Implementation Examples

### Complete Class Example
```cpp
/**
 * @file SensorReader.hpp
 * @brief Manages sensor data acquisition and processing
 */

#pragma once

#include "Watchdog/watchable.hpp"
#include "Tools/logger.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"

namespace tritonai::gkc {

    /**
     * @class SensorReader
     * @brief Coordinates reading from multiple sensor providers
     */
    class SensorReader : public Watchable {
    public:
        /**
         * @brief Constructor
         * @param logger Pointer to logger instance
         */
        explicit SensorReader(ILogger* logger);
        
        /**
         * @brief Register a new sensor provider
         * @param provider Pointer to sensor provider instance
         */
        void RegisterProvider(ISensorProvider* provider);
        
        /**
         * @brief Get the latest sensor readings
         * @return Reference to current sensor packet
         */
        const SensorGkcPacket& GetPacket() const { return m_Packet; }

    protected:
        void SensorPollThreadImpl();
        void WatchdogCallback();

    private:
        ILogger* m_Logger;
        SensorGkcPacket m_Packet{};
        std::vector<ISensorProvider*> m_Providers{};
        Mutex m_ProvidersLock;
        Thread m_SensorPollThread{osPriorityNormal, OS_STACK_SIZE, nullptr, "sensor_poll_thread"};
    };

} // namespace tritonai::gkc
```

### Implementation File Example
```cpp
/**
 * @file SensorReader.cpp
 * @brief Implementation of sensor reading functionality
 */

#include "SensorReader.hpp"

namespace tritonai::gkc {

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

    void SensorReader::WatchdogCallback() {
        m_Logger->SendLog(LogPacket::Severity::FATAL, "SensorReader timeout detected");
        NVIC_SystemReset();
    }

} // namespace tritonai::gkc
```

### Header Guards
Use `#pragma once` for header guards as it's cleaner and less error-prone:

```cpp
#pragma once

#include "mbed.h"
// ... rest of header content
```

---

**Note**: These guidelines are meant to help maintain consistency and quality. When in doubt, follow the patterns established in the existing codebase, and don't hesitate to discuss improvements with the team. Code reviews are a great opportunity to learn and improve our standards together.
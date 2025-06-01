/**
 * @file config.hpp
 * @brief Configuration parameters for the embedded system
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

// ============================================================================
// Build Options
// ============================================================================

// (Currently none, previously serial over usb was an option)


// ============================================================================
// Communication Interfaces
// ============================================================================

// UART settings
#define BAUD_RATE                      115200
#define UART_RX_PIN                    PB_12   // Brake-out board
#define UART_TX_PIN                    PB_13   // Brake-out board

// Remote UART (ELRS receiver)
#define REMOTE_UART_TX_PIN             PE_7    // 14th pin, 1st pin on DuraClik, UART7_RX, ELRS_TX
#define REMOTE_UART_RX_PIN             PE_8    // 16th pin, 2nd pin on DuraClik, UART7_TX, ELRS_RX

// Communication buffers and timing
#define RECV_BUFFER_SIZE               32      // inbound packet buffer
#define WAIT_READ_MS                   5       // ms between reads
#define SEND_QUEUE_SIZE                10      // outbound packet queue depth
#define SEND_SENSOR_INTERVAL_MS        20      // sensor packet send interval

// Tower light indicators
#define TOWER_LIGHT_RED                PD_15
#define TOWER_LIGHT_YELLOW             PD_11
#define TOWER_LIGHT_GREEN              PE_12

// VESC disable pins
#define THROTTLE_VESC_DISABLE_PIN      PD_14
#define STEERING_VESC_DISABLE_PIN      PD_12


// ============================================================================
// Watchdog & Timing
// ============================================================================

// Generic watchdog
#define DEFAULT_WD_INTERVAL_MS             1000  // wake-up frequency
#define DEFAULT_WD_MAX_INACTIVITY_MS       3000  // trigger threshold
#define DEFAULT_WD_WAKEUP_INTERVAL_MS      2     // internal wake period

// Component watchdog intervals
#define DEFAULT_SENSOR_POLL_INTERVAL_MS                 1000
#define DEFAULT_SENSOR_POLL_LOST_TOLERANCE_MS           3000
#define DEFAULT_COMM_POLL_INTERVAL_MS                   1000
#define DEFAULT_COMM_POLL_LOST_TOLERANCE_MS             3000
#define DEFAULT_CONTROLLER_POLL_INTERVAL_MS             1000
#define DEFAULT_CONTROLLER_POLL_LOST_TOLERANCE_MS       3000
#define DEFAULT_RC_CONTROLLER_POLL_INTERVAL_MS          100
#define DEFAULT_RC_CONTROLLER_POLL_LOST_TOLERANCE_MS    3000

// RC heartbeat monitoring
#define DEFAULT_RC_HEARTBEAT_INTERVAL_MS        100
#define DEFAULT_RC_HEARTBEAT_LOST_TOLERANCE_MS  500
#define RC_TAKEOVER_INTERVAL_MS                 100


// ============================================================================
// Actuation
// ============================================================================

// CAN bus 1 settings (nothing on this line currently)
#define CAN1_RX                     PD_0
#define CAN1_TX                     PD_1
#define CAN1_BAUDRATE               500000

// CAN bus 2 settings (currently in use)
#define CAN2_RX                     PB_5
#define CAN2_TX                     PB_6
#define CAN2_BAUDRATE               500000

// Brake pressure limits (PSI)
#define MIN_BRAKE_VAL               600
#define MAX_BRAKE_VAL               3000
#define EMERGENCY_BRAKE_PRESSURE    1.0f  // override brake pressure in emergency

// CAN device IDs
#define THROTTLE_CAN_ID             1
#define STEER_CAN_ID                2
#define BRAKE_CAN_ID                0x00FF0000

// Throttle speed limits (m/s)
#define THROTTLE_MAX_FORWARD_SPEED  20.0f
#define THROTTLE_MAX_REVERSE_SPEED  20.0f

// RC override speed limits
#define RC_MAX_SPEED_FORWARD        5.0f
#define RC_MAX_SPEED_REVERSE        5.0f


// ============================================================================
// Sensors
// ============================================================================

// Brake pressure sensor
#define BRAKE_PRESSURE_SENSOR_PIN   PA_0

// Wheel speed & VESC control
#define WHEEL_DIAMETER_M            0.254f
#define WHEEL_CIRCUMFERENCE_M       (WHEEL_DIAMETER_M * M_PI)
#define NUM_MOTOR_POLES             5.0
#define GEAR_RATIO                  59.0/22.0

// Steering mapping (deg->rad pairs)
#define STEERING_MAPPING { \
    {0.0f,      0.0f    }, \
    {0.523599f, 0.15708f}, \
    {0.872665f, 0.20944f}, \
    {1.22173f,  0.296706f}, \
    {1.39626f,  0.349066f}, \
    {1.57079f,  0.401425f}, \
    {1.74532f,  0.453785f}, \
    {1.91986f,  0.506145f}  \
}
#define MIN_WHEEL_STEER_DEG         -15
#define MAX_WHEEL_STEER_DEG         15
#define ENCODER_OFFSET              0.3f
#define STEERING_RATIO              4.0f // for steering to encoder angle in radians

// ELRS radio channel mapping
#define ELRS_THROTTLE               1
#define ELRS_STEERING               3
#define ELRS_EMERGENCY_STOP_LEFT    4
#define ELRS_EMERGENCY_STOP_RIGHT   7
#define ELRS_TRI_SWITCH_LEFT        5
#define ELRS_TRI_SWITCH_RIGHT       6
#define ELRS_RATIO_THROTTLE         9
#define ELRS_SE                     8


// ============================================================================
// Unused / Future
// ============================================================================

// #define COMM_ETHERNET  // not implemented
// #define COMM_CAN       // not implemented
// #define DEFAULT_PC_HEARTBEAT_INTERVAL_MS       1000
// #define DEFAULT_PC_HEARTBEAT_LOST_TOLERANCE_MS 2000
// #define DEFAULT_MCU_HEARTBEAT_INTERVAL_MS      1000
// #define DEFAULT_MCU_HEARTBEAT_LOST_TOLERANCE_MS 2000
// #define DEFAULT_CTL_CMD_INTERVAL_MS 10
// #define DEFAULT_CTL_CMD_LOST_TOLERANCE_MS 200
// #define WHEEL_SPEED_SENSOR_PIN      PE_14
// #define SPEED_SAMPLE_PERIOD_MS      50
// #define PULSES_PER_REVOLUTION       52

// TODO: implement Ethernet/CAN, PC/MCU heartbeat, control timeout, actuation intervals, steering PID etc.

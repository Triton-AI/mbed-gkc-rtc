/**
 * @file config.hpp
 * @brief Configuration parameters for the embedded system
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

// *************
// Build Options
// *************

// *************
// Communication
// *************
// Choose one of the available interfaces
// Two interfaces were once available, but COMM_USB_SERIAL was removed.
// #define COMM_UART_SERIAL 
//#define COMM_ETHERNET  // not implemented
//#define COMM_CAN  // not implemented

// UART-specific settings
#define BAUD_RATE 115200
#define UART_RX_PIN PB_12       // Connected via brake-out board
#define UART_TX_PIN PB_13       // Connected via brake-out board
#define REMOTE_UART_TX_PIN PE_7 // 14th pin, 1st pin on DuraClik, UART7_RX, ELRS_TX
#define REMOTE_UART_RX_PIN PE_8 // 16th pin, 2nd pin on DuraClik, UART7_TX, ELRS_RX

// Vesc disable pins
#define THROTTLE_VESC_DISABLE_PIN PD_14
#define STEERING_VESC_DISABLE_PIN PD_12

// Tower light pins
#define TOWER_LIGHT_RED PD_15
#define TOWER_LIGHT_YELLOW PD_11
#define TOWER_LIGHT_GREEN PE_12

// Generic comm settings
#define RECV_BUFFER_SIZE 32
// millisecond to wait between each serial/ethernet/can read
#define WAIT_READ_MS 5
// outbound packet queue size
#define SEND_QUEUE_SIZE 10
// interval of sending sensor packets
#define SEND_SENSOR_INTERVAL_MS 50

// *********
// Watchdogs
// *********
// What's the update frequency of the watchdog
#define DEFAULT_WD_INTERVAL_MS 1000
#define DEFAULT_WD_MAX_INACTIVITY_MS 3000
#define DEFAULT_WD_WAKEUP_INTERVAL_MS 2

// Confused on this, RTC sending a watchdog heartbeat to itself?
// Was unused
//     // How often should the MCU send heartbeat by default
//     #define DEFAULT_MCU_HEARTBEAT_INTERVAL_MS 1000
//     #define DEFAULT_MCU_HEARTBEAT_LOST_TOLERANCE_MS 2000
// // How often should the MCU expect heartbeat from PC by default

// // These are not used in code, but should be
//     #define DEFAULT_PC_HEARTBEAT_INTERVAL_MS 1000
//     #define DEFAULT_PC_HEARTBEAT_LOST_TOLERANCE_MS 2000

// // These could be useful, but not yet used
//     // In active mode, how often should the MCU expect control command from PC
//     #define DEFAULT_CTL_CMD_INTERVAL_MS 10
//     #define DEFAULT_CTL_CMD_LOST_TOLERANCE_MS 200
//     // How often should actuation be active
//     #define DEFAULT_ACTUATION_INTERVAL_MS 1000
//     #define DEFAULT_ACTUATION_LOST_TOLERANCE_MS 2000

// How often should sensor system be checked by watchdog
#define DEFAULT_SENSOR_POLL_INTERVAL_MS 1000
#define DEFAULT_SENSOR_POLL_LOST_TOLERANCE_MS 3000
// How often should the communication manager polling happen
#define DEFAULT_COMM_POLL_INTERVAL_MS 1000
#define DEFAULT_COMM_POLL_LOST_TOLERANCE_MS 3000
// How often the controller should be checked by watchdog
#define DEFAULT_CONTROLLER_POLL_INTERVAL_MS 1000
#define DEFAULT_CONTROLLER_POLL_LOST_TOLERANCE_MS 3000
// How often should the rc_controller be checked by watchdog
#define DEFAULT_RC_CONTROLLER_POLL_INTERVAL_MS 100
#define DEFAULT_RC_CONTROLLER_POLL_LOST_TOLERANCE_MS 3000
// How often should RC Heartbeat be checked by watchdog
#define DEFAULT_RC_HEARTBEAT_INTERVAL_MS 100
#define DEFAULT_RC_HEARTBEAT_LOST_TOLERANCE_MS 500
#define RC_TAKEOVER_INTERVAL_MS 100


// *********
// Actuation
// *********
#define CAN1_RX PD_0            // Connected via brake-out board
#define CAN1_TX PD_1            // Connected via brake-out board
#define CAN1_BAUDRATE 500000
#define CAN2_RX PB_5            // Connected via brake-out board
#define CAN2_TX PB_6            // Connected via brake-out board
#define CAN2_BAUDRATE 500000

// Braking
// #define CAN_BRAKE CAN_1 // Which CAN bus to use for brake [CAN_1 | CAN_2] Not used
#define MAX_BRAKE_VAL 3000
#define MIN_BRAKE_VAL 600

    // No PID steering or feedback loop is implemented, all these are unused
    // Steering
    // #define CAN_STEER CAN_2 // Which CAN bus to use for steer [CAN_1 | CAN_2]
    // #define MAX_STEER_DEG 100.0
    // #define MIN_STEER_DEG -100.0
    // #define VIRTUAL_LIMIT_OFF 5
    // #define NEUTRAL_STEER_DEG 0.0
    // #define STEERING_CAL_OFF 0 //this changes the calibration angle
    // #define MAX_STEER_SPEED_ERPM 50000
    // #define MAX_STEER_SPEED_MA 1 //this controls the max steering current i.e strength 
    // #define MIN_STEER_SPEED_MA -1 //this controls the max steering current i.e strength 

    // #define STEERING_CALIB_CURRENT 2700 // mA

    // #define MAX_STEER_CURRENT_MA 24000 //this controls the max steering current i.e strength 
    // #define MIN_STEER_CURRENT_MA -32000 //this controls the max steering current i.e strength 
    //Good configuration for current PID
    // #define STEER_P 30000.0
    // #define STEER_I 7000
    // #define STEER_D 2000//5000.0
    //#define STEADY_STATE_CURRENT_MULT 20000

    // Need to work on getting PID working
    //Good configuration for current PID in the air
    // #define STEER_P 30000
    // #define STEER_I 5000
    // #define STEER_D 3000//5000.0
    // #define STEADY_STATE_CURRENT_MULT_POS 18000
    // #define STEADY_STATE_CURRENT_MULT_NEG 25000
    //good configuration for RPM PID
    // #define STEER_P 25000.0
    // #define STEER_I 0.0
    // #define STEER_D 100
    // #define STEADY_STATE_CURRENT_MULT 0

    // Not used.
    // #define STEER_DEADBAND_DEG 0.5 //VESC already has a limit of min ERPM := 600. Anything bellow this is already used as 0.
    // #define PID_INTERVAL_MS 10
    // #define STEER_VESC_ID 2
    // #define RIGHT_LSWITCH PF_0
    // #define LEFT_LSWITCH PF_1
    // #define ENABLE_LSWITCH      //comment to remove limit switches behaviour

// Throttle
// #define THROTTLE_CAN_PORT  2 // To which can port should the throttle be sent. Not used
#define THROTTLE_CAN_ID 1    // To which can port should the throttle be sent
#define THROTTLE_MAX_FORWARD_SPEED 5.0 // m/s
#define THROTTLE_MAX_REVERSE_SPEED 5.0 // m/s
#define RC_MAX_SPEED_FORWARD 5.0 // m/s
#define RC_MAX_SPEED_REVERSE 5.0 // m/s

// Steering
// #define STEER_CAN_PORT  2 // To which can port should the throttle be sent. Not used
#define STEER_CAN_ID 2    // To which can port should the throttle be sent

// Brake
// #define BRAKE_CAN_PORT  1 // To which can port should the throttle be sent. Not used
#define BRAKE_CAN_ID 0x00FF0000    // To which can port should the throttle be sent

#define EMERGENCY_BRAKE_PRESSURE 1.0 // 0.5 bar

// *******
// Sensors
// *******
// Wheel Speed Sensor
#define WHEEL_SPEED_SENSOR_PIN PE_14   // Digital input for hall effect sensor
#define WHEEL_DIAMETER_M 0.254          // Wheel diameter in meters
#define WHEEL_CIRCUMFERENCE_M (WHEEL_DIAMETER_M * M_PI)  // Wheel circumference in meters
#define SPEED_SAMPLE_PERIOD_MS 50     // Sample period for speed calculation in milliseconds
#define PULSES_PER_REVOLUTION 52      // Number of pulses per wheel revolution
#define GEAR_RATIO 59.0/22.0        // Expected gear ratio from axle to motor
#define NUM_MOTOR_POLES 5           // Number of motor poles

// Brake pressure sensor
#define BRAKE_PRESSURE_SENSOR_PIN PA_0
// #define BRAKE_PRESSURE_MAX_PSI 1000.0f     // Expected Maximum pressure in PSI
// #define BRAKE_PRESSURE_MIN_PSI 0.0f       // Expected Minimum pressure in PSI

// PWM steering encoder
// #define STEER_ENCODER_PIN PC_7 Note: Not used

// motor angle - left wheel - right wheel - average (in degrees)
// 0	0	0	0
// 30	8	10	9
// 50	9	15	12
// 70	12	22	17
// 80	10	30	20
// 90           23
// 100          26
// 110          29

//(in radians)
#define STERING_MAPPING    {{0, 0,},\
                            {0.523599, 0.15708,},\
                            {0.872665, 0.20944,},\
                            {1.22173, 0.296706,},\
                            {1.39626, 0.349066,},\
                            {1.57079, 0.401425},\
                            {1.74532, 0.453785},\
                            {1.91986, 0.506145}}; // takes first and last column
#define MIN_WHEEL_STEER_DEG -20
#define MAX_WHEEL_STEER_DEG 20
#define MOTOR_OFFSET 0.3

// *****
// ESTOP
// *****
// Not used?
// #define ESTOP_PIN PB_10

// //PWM pins for RC car
// #define Steer_Pin PA_5
// #define Throttle_Pin PA_6
// #define Red_Pin PD_12

//// ELRS
#define ELRS_THROTLE 1
#define ELRS_STEERING 3
#define ELRS_EMERGENCY_STOP_LEFT 4
#define ELRS_EMERGENCY_STOP_RIGHT 7
#define ELRS_TRI_SWITCH_RIGHT 6
#define ELRS_TRI_SWITCH_LEFT 5
#define ELRS_RATIO_THROTTLE 9
#define ELRS_SE 8
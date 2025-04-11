#include "RCController.hpp"
#include <iostream>
#include <string>

namespace tritonai::gkc
{
// ================== TRANSLATION FUNCTIONS ==================

double Translation::normalize(int analogValue)
{
    if (analogValue > 1800)
        analogValue = 1800;
    if (analogValue < 174)
        analogValue = 174;
    analogValue -= 992;
    return analogValue >= 0 
            ? static_cast<double>(analogValue) / (1800 - 992)
            : static_cast<double>(analogValue) / (992 - 174);
}

double Translation::throttle(int throttleVal)
{
    return normalize(throttleVal);
}

double Translation::throttle_ratio(int throttleVal)
{
    double normalized_value = normalize(throttleVal);
    return (normalized_value + 1.0) / 2.0;
}

// Removed: keep_constant_thr()

double Translation::brake(int brakeVal)
{
    return normalize(brakeVal);
}

double Translation::steering(int steerVal)
{
    double normalized_value = normalize(steerVal);
    double Steering_Ang = 0.0;
    if (normalized_value < 0)
        Steering_Ang = -normalized_value * MIN__WHEEL_STEER_DEG;
    else
        Steering_Ang = normalized_value * MAX__WHEEL_STEER_DEG;

    if (Steering_Ang > MAX__WHEEL_STEER_DEG)
        Steering_Ang = MAX__WHEEL_STEER_DEG;
    if (Steering_Ang < MIN__WHEEL_STEER_DEG)
        Steering_Ang = MIN__WHEEL_STEER_DEG;
    if (-0.1 < Steering_Ang && Steering_Ang < 0.1)
        Steering_Ang = 0.0;

    double Steering_Ang_rad = Steering_Ang * (M_PI / 180.0);
    return Steering_Ang_rad;
}

// Only the right toggle causes an emergency stop.
bool Translation::is_active(int right_toggle)
{
    double right_norm = normalize(right_toggle);
    return (right_norm < 0.0);
}

bool Translation::isControllerPassthrough(int left_toggle)
{
    double left_norm = normalize(left_toggle);
    return (left_norm > 0.5);
}

AutonomyMode Translation::getAutonomyMode(int rightTriVal)
{
    double rightTriVal_norm = normalize(rightTriVal);
    if (rightTriVal_norm < -0.5)
        return AUTONOMOUS;
    else if (rightTriVal_norm > 0.5)
        return MANUAL;
    else
        return AUTONOMOUS_OVERRIDE;
}

// ================== RCController Methods ====================

void RCController::update()
{
    // Bus data pointer is assumed to be updated within _receiver.gatherData()
    const uint16_t *busData = _receiver.busData();

    while (true)
    {
        ThisThread::sleep_for(10ms);
        inc_count(); // Increment the watchdog rolling counter

        if (!_receiver.gatherData())
            continue; // No new data available

        if (!_receiver.messageAvailable)
            continue; // Message not available

        // std::cout << "Bus data: " << (int)(100*busData[0]) <<
        //     " " << (int)(100*busData[1]) <<
        //     " " << (int)(100*busData[2]) <<
        //     " " << (int)(100*busData[3]) <<
        //     " " << (int)(100*busData[4]) <<
        //     " " << (int)(100*busData[5]) <<
        //     " " << (int)(100*busData[6]) <<
        //     " " << (int)(100*busData[7]) <<
        //     " " << (int)(100*busData[8]) <<
        //     std::endl;

        // Check emergency status based solely on the right emergency toggle.
        bool emergencyActive = Map.is_active(busData[ELRS_EMERGENCY_STOP_RIGHT]);

        // Update the passthrough condition based on the left emergency key.
        bool passthroughEnabled = (_packet.autonomy_mode == AUTONOMOUS &&
                                        Map.isControllerPassthrough(busData[ELRS_LEFT_TOGGLE]));

        // Update indicator state on every iteration.
        _indicator_state = passthroughEnabled;

        // When passthrough is enabled, emulate virtual buttons using the tri switch
        // and the designated actuator button.
        if (passthroughEnabled)
        {
            int8_t joystick_x = static_cast<int8_t>(Map.normalize(busData[ELRS_STEERING]) * 127.0);
            int8_t joystick_y = static_cast<int8_t>(Map.normalize(busData[ELRS_THROTLE]) * 127.0);
            
            uint8_t joystick_buttons = 0x00;
            
            double tri_value = Map.normalize(busData[ELRS_TRI_SWITCH_LEFT]);
            int selectedButton = 0;
            if (tri_value < -0.5)
                selectedButton = 0;  // Virtual Button 1
            else if (tri_value > 0.5)
                selectedButton = 2;  // Virtual Button 3
            else
                selectedButton = 1;  // Virtual Button 2

            bool isActuated = (Map.normalize(busData[ELRS_SE]) > 0.5);

            uint8_t dialPercent = static_cast<uint8_t>(Map.throttle_ratio(busData[ELRS_RATIO_THROTTLE]) * 100);

            if (isActuated)
            {
                joystick_buttons |= (1 << selectedButton);
            }

            _joystick.update(joystick_x, joystick_y, joystick_buttons, dialPercent);
        }

        // Process RC control commands when not in emergency stop.
        // Removed references to keep constant throttle.
        bool is_all_zero = (std::abs(100 * Map.normalize(busData[ELRS_THROTLE])) <= 5 &&
                            std::abs(100 * Map.normalize(busData[ELRS_STEERING])) <= 5);

        if (is_all_zero)
        {
            _packet.throttle = 0.0;
            _packet.steering = 0.0;
            _packet.brake = 0.0;
            _packet.is_active = emergencyActive;
            _packet.autonomy_mode = Map.getAutonomyMode(busData[ELRS_TRI_SWITCH_RIGHT]);
            _packet.publish(*_sub);
            continue;
        }

        // Always update throttle from the current channel reading.
        current_throttle = Map.throttle(busData[ELRS_THROTLE]);
        _packet.throttle = current_throttle * Map.throttle_ratio(busData[ELRS_RATIO_THROTTLE]);
        _packet.brake = 0.0; // TODO: Implement brake functionality
        _packet.steering = Map.steering(busData[ELRS_STEERING]);
        _packet.autonomy_mode = Map.getAutonomyMode(busData[ELRS_TRI_SWITCH_RIGHT]);
        _packet.is_active = emergencyActive;

        _is_ready = true;
        _packet.publish(*_sub);
    }
}

RCController::RCController(GkcPacketSubscriber *sub)
    : Watchable(DEFAULT_RC_CONTROLLER_POLL_INTERVAL_MS, DEFAULT_RC_CONTROLLER_POLL_LOST_TOLERANCE_MS, "RCController"),
        _receiver(REMOTE_UART_RX_PIN, REMOTE_UART_TX_PIN),
        _is_ready(false),
        _sub(sub),
        _joystick(true),
        _indicator_state(false)
{
    _rc_thread.start(callback(this, &RCController::update));
    attach(callback(this, &RCController::watchdog_callback));
}

void RCController::watchdog_callback()
{
    std::cout << "RCController watchdog triggered" << std::endl;
    NVIC_SystemReset();
}

bool RCController::getIndicatorState() const
{
    return _indicator_state;
}
} // namespace tritonai::gkc
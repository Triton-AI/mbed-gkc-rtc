/**
 * @file RCController.cpp
 * @brief Implementation of the Remote Control interface
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "RCController.hpp"
#include <iostream>
#include <string>

namespace tritonai::gkc {

    double Translation::Normalize(int analogValue) {
        if (analogValue > 1800)
            analogValue = 1800;
        if (analogValue < 174)
            analogValue = 174;
        analogValue -= 992;
        return analogValue >= 0 
                ? static_cast<double>(analogValue) / (1800 - 992)
                : static_cast<double>(analogValue) / (992 - 174);
    }

    double Translation::Throttle(int throttleVal) {
        return Normalize(throttleVal);
    }

    double Translation::ThrottleRatio(int throttleVal) {
        double normalizedValue = Normalize(throttleVal);
        return (normalizedValue + 1.0) / 2.0;
    }

    double Translation::Brake(int brakeVal) {
        return Normalize(brakeVal);
    }

    double Translation::Steering(int steerVal) {
        double normalizedValue = Normalize(steerVal);
        double steeringAng = 0.0;
        if (normalizedValue < 0)
            steeringAng = -normalizedValue * MIN_WHEEL_STEER_DEG;
        else
            steeringAng = normalizedValue * MAX_WHEEL_STEER_DEG;

        if (steeringAng > MAX_WHEEL_STEER_DEG)
            steeringAng = MAX_WHEEL_STEER_DEG;
        if (steeringAng < MIN_WHEEL_STEER_DEG)
            steeringAng = MIN_WHEEL_STEER_DEG;
        if (-0.1 < steeringAng && steeringAng < 0.1)
            steeringAng = 0.0;

        double steeringAngRad = steeringAng * (M_PI / 180.0);
        return steeringAngRad;
    }

    bool Translation::IsActive(int rightToggle) {
        double rightNorm = Normalize(rightToggle);
        return (rightNorm < 0.0);
    }

    bool Translation::IsControllerPassthrough(int leftToggle) {
        double leftNorm = Normalize(leftToggle);
        return (leftNorm > 0.5);
    }

    AutonomyMode Translation::GetAutonomyMode(int rightTriVal) {
        double rightTriValNorm = Normalize(rightTriVal);
        if (rightTriValNorm < -0.5)
            return AUTONOMOUS;
        else if (rightTriValNorm > 0.5)
            return MANUAL;
        else
            return AUTONOMOUS_OVERRIDE;
    }

    void RCController::Update()
    {
        const uint16_t* busData = m_Receiver.busData();

        while (true) {
            ThisThread::sleep_for(10ms);
            IncCount();

            if (!m_Receiver.gatherData())
                continue;

            if (!m_Receiver.messageAvailable)
                continue;

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

            bool emergencyActive = Map.IsActive(busData[ELRS_EMERGENCY_STOP_RIGHT]);

            // Activate the brake during emergency
            if(!emergencyActive) {
                m_Packet.throttle = 0.0;
                m_Packet.steering = 0.0;
                m_Packet.brake = Map.ThrottleRatio(busData[ELRS_RATIO_THROTTLE]);
                m_Packet.is_active = emergencyActive;
                m_Packet.publish(*m_Sub);
                continue;
            }

            bool passthroughEnabled = (m_Packet.autonomy_mode == AUTONOMOUS &&
                                    Map.IsControllerPassthrough(busData[ELRS_LEFT_TOGGLE]));

            m_IndicatorState = passthroughEnabled;

            if (passthroughEnabled) {
                m_USBConnected = m_Joystick.IsConnected();

                if (m_USBConnected) {
                    int8_t joystickX = static_cast<int8_t>(Map.Normalize(busData[ELRS_STEERING]) * 127.0);
                    int8_t joystickY = static_cast<int8_t>(Map.Normalize(busData[ELRS_THROTLE]) * 127.0);
                    
                    uint8_t joystickButtons = 0x00;
                    
                    double triValue = Map.Normalize(busData[ELRS_TRI_SWITCH_LEFT]);
                    int selectedButton = 0;
                    if (triValue < -0.5)
                        selectedButton = 0;
                    else if (triValue > 0.5)
                        selectedButton = 2;
                    else
                        selectedButton = 1;

                    bool isActuated = (Map.Normalize(busData[ELRS_SE]) > 0.5);

                    uint8_t dialPercent = static_cast<uint8_t>(Map.ThrottleRatio(busData[ELRS_RATIO_THROTTLE]) * 100);

                    if (isActuated) {
                        joystickButtons |= (1 << selectedButton);
                    }

                    m_Joystick.Update(joystickX, joystickY, joystickButtons, dialPercent);
                }
            } else {
                m_USBConnected = false;
            }

            bool isAllZero = (std::abs(100 * Map.Normalize(busData[ELRS_THROTLE])) <= 5 &&
                            std::abs(100 * Map.Normalize(busData[ELRS_STEERING])) <= 5);

            if (isAllZero) {
                m_Packet.throttle = 0.0;
                m_Packet.steering = 0.0;
                m_Packet.brake = 0.0;
                m_Packet.is_active = emergencyActive;
                m_Packet.autonomy_mode = Map.GetAutonomyMode(busData[ELRS_TRI_SWITCH_RIGHT]);
                m_Packet.publish(*m_Sub);
                continue;
            }

            m_CurrentThrottle = Map.Throttle(busData[ELRS_THROTLE]);
            m_Packet.throttle = m_CurrentThrottle * Map.ThrottleRatio(busData[ELRS_RATIO_THROTTLE]);
            m_Packet.brake = 0.0; // TODO: Implement brake
            m_Packet.steering = Map.Steering(busData[ELRS_STEERING]);
            m_Packet.autonomy_mode = Map.GetAutonomyMode(busData[ELRS_TRI_SWITCH_RIGHT]);
            m_Packet.is_active = emergencyActive;

            m_IsReady = true;
            m_Packet.publish(*m_Sub);
        }
    }

    RCController::RCController(GkcPacketSubscriber* sub, ILogger* logger)
        : Watchable(DEFAULT_RC_CONTROLLER_POLL_INTERVAL_MS, DEFAULT_RC_CONTROLLER_POLL_LOST_TOLERANCE_MS, "RCController"),
        m_Receiver(REMOTE_UART_RX_PIN, REMOTE_UART_TX_PIN),
        m_Packet{},
        m_Sub(sub),
        m_Logger(logger),
        m_Joystick(true),
        m_IsReady(false),
        m_IndicatorState(false)
    {
        m_RCThread.start(callback(this, &RCController::Update));
        Attach(callback(this, &RCController::WatchdogCallback));
    }

    void RCController::WatchdogCallback() {
        m_Logger->SendLog(LogPacket::Severity::FATAL, "RCController watchdog triggered");
        NVIC_SystemReset();
    }

    bool RCController::GetIndicatorState() const {
        return m_IndicatorState;
    }

} // namespace tritonai::gkc
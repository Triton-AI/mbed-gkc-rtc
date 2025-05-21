/**
 * @file actuation_controller.cpp
 * @brief Implementation of vehicle actuation control
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "Actuation/actuation_controller.hpp"
#include "Tools/logger.hpp"
#include "Actuation/vesc_can_tools.hpp"
#include "config.hpp"
#include <algorithm>

namespace tritonai::gkc {

    ActuationController::ActuationController(ILogger* logger) : m_Logger(logger) {
        InitializeCanWithCallback();
        m_Logger->SendLog(LogPacket::Severity::INFO, "ActuationController initialized with CAN callbacks");
    }

    void ActuationController::SetThrottleCmd(float cmd) {
        cmd = ActuationController::Clamp(cmd, THROTTLE_MAX_FORWARD_SPEED, -1.0f*THROTTLE_MAX_REVERSE_SPEED);
        CommCanSetSpeed(cmd);

        float realSpeed = GetCurrentSpeed();
        float error = cmd - realSpeed;
        m_Logger->SendLog(LogPacket::Severity::FATAL, "Real Speed " + std::to_string(realSpeed));
        m_Logger->SendLog(LogPacket::Severity::FATAL, "Commanded Speed " + std::to_string(cmd));
        m_Logger->SendLog(LogPacket::Severity::FATAL, "Error Speed " + std::to_string(error));
    }

    void ActuationController::FullRelRevCurrentBrake() {
        CommCanSetCurrentBrakeRel(THROTTLE_CAN_ID, 1.0);
    }

    void ActuationController::SetSteeringCmd(float cmd) {
        CommCanSetAngle(cmd);

        float realAngle = GetSteeringAngle();
        float error = cmd - realAngle;
        m_Logger->SendLog(LogPacket::Severity::FATAL, "Real Angle " + std::to_string(realAngle));
        m_Logger->SendLog(LogPacket::Severity::FATAL, "Commanded Angle " + std::to_string(cmd));
        m_Logger->SendLog(LogPacket::Severity::FATAL, "Error Angle " + std::to_string(error));
    }

    void ActuationController::SetBrakeCmd(float cmd) {
        CommCanSetBrakePosition(cmd);
    }

    float ActuationController::GetSteeringAngle() const {
        return CommCanGetAngle();
    }

    float ActuationController::GetCurrentSpeed() const {
        return CommCanGetSpeed();
    }

} // namespace tritonai::gkc
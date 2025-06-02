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
        InitializeCan();
        m_Logger->SendLog(LogPacket::Severity::INFO, "ActuationController initialized with CAN callbacks");
    }

    void ActuationController::SetThrottleCmd(float cmd) {
        cmd = ActuationController::Clamp(cmd, THROTTLE_MAX_FORWARD_SPEED, -1.0f*THROTTLE_MAX_REVERSE_SPEED);
        CommCanSetSpeed(cmd);
    }

    void ActuationController::FullRelRevCurrentBrake() {
        CommCanSetCurrentBrakeRel(THROTTLE_CAN_ID, 1.0);
    }

    void ActuationController::SetSteeringCmd(float cmd) {
        CommCanSetAngle(cmd);
        // Log the command for steering PID tuning
        // float motorAngle = cmd * STEERING_RATIO + ENCODER_OFFSET;
        // float radToDeg = 180.0 / M_PI * motorAngle;
        // m_Logger->SendLog(LogPacket::Severity::FATAL, "Set steering command: " + std::to_string(radToDeg) + "Deg");
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
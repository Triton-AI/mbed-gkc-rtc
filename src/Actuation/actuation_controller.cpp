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

    ActuationController::ActuationController(ILogger* logger) 
        : m_Logger(logger) {
    }

    void ActuationController::SetThrottleCmd(float cmd) {
        cmd = ActuationController::Clamp(cmd, THROTTLE_MAX_REVERSE_SPEED, -1.0f*THROTTLE_MAX_REVERSE_SPEED);
        m_Logger->SendLog(LogPacket::Severity::DEBUG, "Set throttle command: " + std::to_string(cmd));
        CommCanSetSpeed(cmd);
    }

    void ActuationController::FullRelRevCurrentBrake() {
        CommCanSetCurrentBrakeRel(THROTTLE_CAN_ID, 1.0);
    }

    void ActuationController::SetSteeringCmd(float cmd) {
        m_Logger->SendLog(LogPacket::Severity::DEBUG, "Set steering command: " + std::to_string(cmd));
        CommCanSetAngle(cmd);
    }

    void ActuationController::SetBrakeCmd(float cmd) {
        m_Logger->SendLog(LogPacket::Severity::DEBUG, "Set brake command: " + std::to_string(cmd));
        CommCanSetBrakePosition(cmd);
    }

} // namespace tritonai::gkc
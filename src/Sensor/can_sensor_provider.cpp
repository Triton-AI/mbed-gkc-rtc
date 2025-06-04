/**
 * @file can_sensor_provider.cpp
 * @brief Implementation of CAN-based sensor provider
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "can_sensor_provider.hpp"
#include "Actuation/vesc_can_tools.hpp"
#include <cmath>
#include <string>

namespace tritonai::gkc {

    CanSensorProvider::CanSensorProvider(ILogger* logger)
        : m_Logger(logger)
    {
        m_Logger->SendLog(LogPacket::Severity::INFO, "CAN sensor provider initialized");
    }

    bool CanSensorProvider::IsReady() {
        // CAN sensor data may be stale but is always readable
        return true;
    }

    void CanSensorProvider::PopulateReading(SensorGkcPacket& pkt) {
        // Get steering angle from CAN feedback
        float steeringAngle = CommCanGetAngle();
        if (!std::isnan(steeringAngle)) {
            pkt.values.steering_angle_rad = steeringAngle;
        } else {
            // Set to 0 as fallback
            m_Logger->SendLog(LogPacket::Severity::WARNING, 
                "CAN steering angle feedback not available - setting to 0");
            pkt.values.steering_angle_rad = 0.0f;
        }
        
        // Get speed from CAN feedback
        float canSpeed = CommCanGetSpeed();
        if (!std::isnan(canSpeed)) {
            // CAN speed is used on all wheels, best estimate we have
            pkt.values.wheel_speed_fl = canSpeed;
            pkt.values.wheel_speed_fr = canSpeed;
            pkt.values.wheel_speed_rl = canSpeed;
            pkt.values.wheel_speed_rr = canSpeed;
        } else {
            // Set all wheel speeds to 0 as fallback
            m_Logger->SendLog(LogPacket::Severity::WARNING, 
                "CAN speed feedback not available - setting all wheel speeds to 0");
            
            pkt.values.wheel_speed_fl = 0.0f;
            pkt.values.wheel_speed_fr = 0.0f;
            pkt.values.wheel_speed_rl = 0.0f;
            pkt.values.wheel_speed_rr = 0.0f;
        }
    }

    float CanSensorProvider::GetSteeringAngle() const {
        return CommCanGetAngle();
    }

    float CanSensorProvider::GetSpeed() const {
        return CommCanGetSpeed();
    }

    bool CanSensorProvider::IsSteeringDataValid() const {
        float angle = CommCanGetAngle();
        return !std::isnan(angle);
    }

    bool CanSensorProvider::IsSpeedDataValid() const {
        float speed = CommCanGetSpeed();
        return !std::isnan(speed);
    }

} // namespace tritonai::gkc
/**
 * @file BrakePressureSensor.cpp
 * @brief Implementation of the brake pressure sensor
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "brake_pressure_sensor.hpp"
#include <string>

namespace tritonai::gkc {

    BrakePressureSensor::BrakePressureSensor(ILogger* logger)
        : m_BrakeSensor(BRAKE_PRESSURE_SENSOR_PIN), m_Logger(logger), m_CurrentPressure(0.0f) 
    {
        m_Logger->SendLog(LogPacket::Severity::DEBUG, 
                        "Brake pressure sensor initialized on pin " + std::to_string(BRAKE_PRESSURE_SENSOR_PIN));
    }

    bool BrakePressureSensor::IsReady() {
        // Analog sensors are always ready to read
        return true;
    }

    void BrakePressureSensor::PopulateReading(SensorGkcPacket& pkt) {
        float normalizedValue = m_BrakeSensor.read();
        m_CurrentPressure = normalizedValue * 1000.0f;

        // Populate the packet
        pkt.values.brake_pressure = m_CurrentPressure;

        m_Logger->SendLog(LogPacket::Severity::DEBUG, 
            "Brake pressure: " + std::to_string((int)m_CurrentPressure) + 
            " PSI (Raw: " + std::to_string(normalizedValue) + ")");
    }

    float BrakePressureSensor::GetPressure() const {
        return m_CurrentPressure;
    }

} // namespace tritonai::gkc
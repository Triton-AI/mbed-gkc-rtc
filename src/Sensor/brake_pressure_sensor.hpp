/**
 * @file BrakePressureSensor.hpp
 * @brief Brake pressure sensor implementation
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include "Sensor/sensor_reader.hpp"
#include "mbed.h"
#include "Tools/logger.hpp"
#include "config.hpp"

namespace tritonai::gkc {

    /**
     * @brief Reads brake pressure from an analog sensor
     * 
     * Converts normalized analog input (0-1.0) to pressure (0-100 psi)
     */
    class BrakePressureSensor : public ISensorProvider {
    public:
        BrakePressureSensor(ILogger* logger);
        bool IsReady() override;
        void PopulateReading(SensorGkcPacket& pkt) override;
        float GetPressure() const;
        
    private:
        AnalogIn m_BrakeSensor{BRAKE_PRESSURE_SENSOR_PIN};
        ILogger* m_Logger;
        float m_CurrentPressure;
    };

} // namespace tritonai::gkc
/**
 * @file can_sensor_provider.hpp
 * @brief CAN-based sensor provider for steering angle and speed feedback
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include "Sensor/sensor_reader.hpp"
#include "Tools/logger.hpp"
#include "config.hpp"

namespace tritonai::gkc {

    /**
     * @brief Provides sensor data received via CAN bus
     * 
     * This class interfaces with the CAN system to retrieve sensor data
     * such as steering angle and speed feedback from VESC controllers
     */
    class CanSensorProvider : public ISensorProvider {
    public:
        /**
         * @brief Constructor
         * @param logger Logger instance for debugging
         */
        CanSensorProvider(ILogger* logger);
        
        /**
         * @brief Check if CAN sensor data is available
         * @return Always true - CAN data may be stale but is always readable
         */
        bool IsReady() override;
        
        /**
         * @brief Populate sensor packet with CAN-based sensor readings
         * @param pkt Packet to populate with steering angle and other CAN data
         */
        void PopulateReading(SensorGkcPacket& pkt) override;
        
        /**
         * @brief Get the current steering angle from CAN feedback
         * @return Steering angle in radians, or NaN if not available
         */
        float GetSteeringAngle() const;
        
        /**
         * @brief Get the current speed from CAN feedback
         * @return Speed in m/s, or NaN if not available
         */
        float GetSpeed() const;
        
        /**
         * @brief Check if steering angle data has been received
         * @return True if valid steering data is available
         */
        bool IsSteeringDataValid() const;
        
        /**
         * @brief Check if speed data has been received
         * @return True if valid speed data is available
         */
        bool IsSpeedDataValid() const;
        
    private:
        ILogger* m_Logger;
    };

} // namespace tritonai::gkc
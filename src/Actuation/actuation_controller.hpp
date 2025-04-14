/**
 * @file actuation_controller.hpp
 * @brief Controller for vehicle actuation systems
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include "Mutex.h"
#include "Queue.h"
#include "Tools/logger.hpp"
#include "mbed.h"
#include "Sensor/sensor_reader.hpp"
#include <cstdint>

namespace tritonai::gkc {

    /**
     * @brief Controls vehicle actuation systems (throttle, steering, brakes)
     */
    class ActuationController {
    public:
        /**
         * @brief Constructor
         * @param logger Logger instance for debugging
         */
        explicit ActuationController(ILogger* logger);

        /**
         * @brief Set throttle command
         * @param cmd Throttle value
         */
        void SetThrottleCmd(float cmd);
        
        /**
         * @brief Set steering command
         * @param cmd Steering value
         */
        void SetSteeringCmd(float cmd);
        
        /**
         * @brief Set brake command
         * @param cmd Brake value
         */
        void SetBrakeCmd(float cmd);
        
        /**
         * @brief Apply full reverse current brake
         */
        void FullRelRevCurrentBrake();

        /**
         * @brief Clamp a value between min and max
         * @param val Value to clamp
         * @param max Maximum allowed value
         * @param min Minimum allowed value
         * @return Clamped value
         */
        float Clamp(float val, float max, float min) {
            if (val < min)
                return min;
            else if (val > max)
                return max;
            else
                return val;
        }

        ILogger* m_Logger;
    };

} // namespace tritonai::gkc
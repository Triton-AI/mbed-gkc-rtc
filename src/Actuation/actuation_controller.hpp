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

    class ActuationController {
    public:
        explicit ActuationController(ILogger* logger);

        void SetThrottleCmd(float cmd);
        void SetSteeringCmd(float cmd);
        void SetBrakeCmd(float cmd);
        void FullRelRevCurrentBrake();

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
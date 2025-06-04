/**
 * @file global_profilers.hpp
 * @brief Global profiler instances for system performance monitoring
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include "profiler.hpp"

namespace tritonai::gkc {

    // Defines global profilers for each section of the code
    Profiler CommProfiler("Comm");
    Profiler ControlProfiler("Control");
    Profiler SensorProfiler("Sensor");

} // namespace tritonai::gkc
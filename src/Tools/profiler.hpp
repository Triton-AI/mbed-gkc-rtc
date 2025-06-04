/**
 * @file profiler.hpp
 * @brief Performance profiling functionality
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <iostream>
#include <mbed.h>
#include <sstream>
#include <string>
#include <vector>

#define AVERAGE_WINDOW_SIZE 10

namespace tritonai::gkc {

    /**
    * @brief Provides functionality for profiling code performance
    */
    class Profiler {
    public:
        /**
        * @brief Construct a new Profiler object
        * @param name Name of the section being profiled
        */
        Profiler(const char* name) : m_Name(name) {}

        /**
        * @brief Start the timer
        */
        void StartTimer() {
            if (!m_Profiling) {
                m_Timer.reset();
                m_Timer.start();
                m_Profiling = true;
            }
        }

        /**
        * @brief Stop the timer
        */
        void StopTimer() {
            m_Timer.stop();
            m_Profiling = false;
            if (m_Buffer.size() == AVERAGE_WINDOW_SIZE) {
                m_Buffer.erase(m_Buffer.begin());
            }
            m_Buffer.push_back(m_Timer.elapsed_time());
        }

        /**
        * @brief Get the last recorded time
        * @return Last time measurement
        */
        std::chrono::microseconds GetLastTime() const { 
            return m_Buffer.back(); 
        }

        /**
        * @brief Get the average recorded time
        * @return Average time measurement
        */
        std::chrono::microseconds GetAverageTime() const {
            std::chrono::microseconds total(0);
            for (const auto& time : m_Buffer) {
                total += time;
            }
            return total / m_Buffer.size();
        }

        /**
        * @brief Get the name of this profiler
        * @return Name string
        */
        std::string GetName() const { 
            return m_Name; 
        }

        /**
        * @brief Dump profiler information to a string
        * @param newline Whether to add a newline character
        * @return Formatted string with profiler info
        */
        std::string Dump(const bool& newline = true) const {
            std::stringstream ss;
            ss << "[Profiler " << GetName()
            << "]: last_time (us): " << GetLastTime().count()
            << ", ave_time (us): " << GetAverageTime().count()
            << (newline ? "\n" : "\r");
            return ss.str();
        }

    protected:
        Timer m_Timer;
        std::vector<std::chrono::microseconds> m_Buffer;
        std::string m_Name;
        bool m_Profiling{false};
    };

} // namespace tritonai::gkc
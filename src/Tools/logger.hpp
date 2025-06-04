/**
 * @file logger.hpp
 * @brief Logger interface for the system
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include "tai_gokart_packet/gkc_packets.hpp"
#include <string>

namespace tritonai::gkc {

    /**
     * @brief Logger interface
     * Abstract class for logger, allowing one to implement different loggers
     * of different severities and destinations.
     */
    class ILogger {
    public:
        ILogger() {}
        
        /**
         * @brief Sends a log packet with the given severity and message
         */
        virtual void SendLog(const LogPacket::Severity& severity,
                             const std::string& what) = 0;
    };

} // namespace tritonai::gkc
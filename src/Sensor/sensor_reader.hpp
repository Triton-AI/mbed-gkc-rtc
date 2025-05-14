/**
 * @file sensor_reader.hpp
 * @brief Sensor reading and management functionality
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include "Mutex.h"
#include "config.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"
#include "Watchdog/watchable.hpp"
#include "Tools/logger.hpp"
#include <chrono>
#include <cstdint>
#include <vector>

namespace tritonai::gkc {

    /**
    * @brief Interface for sensor data providers
    */
    class ISensorProvider {
    public:
        ISensorProvider() {}
        
        /**
        * @brief Check if sensor data is ready to be read
        * @return True if data is ready
        */
        virtual bool IsReady() = 0;
        
        /**
        * @brief Populates the sensor packet with readings
        * @param pkt Packet to populate with data
        */
        virtual void PopulateReading(SensorGkcPacket& pkt) = 0;
    };

    /**
    * @brief Manages sensor readings from multiple providers
    */
    class SensorReader : public Watchable {
    public:
        /**
        * @brief Constructor
        * @param logger Logger instance for debugging
        */
        SensorReader(ILogger* logger);
        
        /**
        * @brief Add a sensor provider to the reading list
        * @param provider Pointer to provider instance
        */
        void RegisterProvider(ISensorProvider* provider);
        
        /**
        * @brief Remove a sensor provider from the reading list
        * @param provider Pointer to provider instance to remove
        */
        void RemoveProvider(ISensorProvider* provider);
        
        /**
        * @brief Get the current sensor packet
        * @return Reference to the latest sensor readings
        */
        const SensorGkcPacket& GetPacket() const { 
            return m_Packet; 
        }
        
        /**
        * @brief Set the polling interval
        * @param val New interval in milliseconds
        */
        void SetPollInterval(std::chrono::milliseconds val) {
            m_PollInterval = val;
        }
        
        /**
        * @brief Get the current polling interval
        * @return Current interval in milliseconds
        */
        std::chrono::milliseconds GetPollInterval() { 
            return m_PollInterval; 
        }

        /**
        * @brief Callback for watchdog timeouts
        */
        void WatchdogCallback();

    protected:
        ILogger* m_Logger;
        SensorGkcPacket m_Packet{};
        std::vector<ISensorProvider*> m_Providers{};
        Mutex m_ProvidersLock;
        std::chrono::milliseconds m_PollInterval{SEND_SENSOR_INTERVAL_MS};

        Thread m_SensorPollThread{osPriorityNormal, OS_STACK_SIZE, nullptr, "sensor_poll_thread"};
        void SensorPollThreadImpl();
    };

} // namespace tritonai::gkc
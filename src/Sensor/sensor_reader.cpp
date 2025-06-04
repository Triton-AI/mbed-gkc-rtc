/**
 * @file sensor_reader.cpp
 * @brief Implementation of sensor reading functionality
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "sensor_reader.hpp"
#include "ThisThread.h"
#include "config.hpp"
#include "Watchdog/watchdog.hpp"
#include <cstdio>
#include <string>

namespace tritonai::gkc {

    SensorReader::SensorReader(ILogger* logger)
        : Watchable(DEFAULT_SENSOR_POLL_INTERVAL_MS,
                    DEFAULT_SENSOR_POLL_LOST_TOLERANCE_MS,
                    "SensorReader"),
        m_Logger(logger) {
        m_SensorPollThread.start(callback(this, &SensorReader::SensorPollThreadImpl));
        m_Logger->SendLog(LogPacket::Severity::INFO, "SensorReader initialized");
        Attach(callback(this, &SensorReader::WatchdogCallback));
    }

    void SensorReader::SensorPollThreadImpl() {
        while (!ThisThread::flags_get()) {
            m_ProvidersLock.lock();
            for (auto& provider : m_Providers) {
                if (provider->IsReady()) {
                    provider->PopulateReading(m_Packet);
                }
            }
            m_ProvidersLock.unlock();
            this->IncCount(); // Increments the count of the watchdog
            ThisThread::sleep_for(m_PollInterval);
        }
    }

    void SensorReader::RegisterProvider(ISensorProvider* provider) {
        m_ProvidersLock.lock();
        m_Providers.push_back(provider);
        m_ProvidersLock.unlock();
    }

    void SensorReader::RemoveProvider(ISensorProvider* provider) {
        m_ProvidersLock.lock();
        m_Providers.erase(std::remove(m_Providers.begin(), m_Providers.end(), provider),
                        m_Providers.end());
        m_ProvidersLock.unlock();
    }

    void SensorReader::WatchdogCallback() {
        m_Logger->SendLog(LogPacket::Severity::FATAL, "SensorReader Timeout detected");
        NVIC_SystemReset();
    }

} // namespace tritonai::gkc
/**
 * @file watchdog.cpp
 * @brief Implementation of the watchdog monitoring system
 */

#include "watchdog.hpp"
#include <chrono>
#include <functional>
#include <iostream>
#include <string>

namespace tritonai::gkc {

    Watchdog::Watchdog(uint32_t updateIntervalMs, uint32_t maxInactivityLimitMs, 
                    uint32_t wakeupEveryMs, ILogger* logger)
        : Watchable(updateIntervalMs, maxInactivityLimitMs, "Watchdog"),
        m_WatchdogIntervalMs(wakeupEveryMs),
        m_Logger(logger)
    {
        AddToWatchlist(this);
        Attach(callback(this, &Watchdog::WatchdogCallback));
        m_WatchThread.start(callback(this, &Watchdog::StartWatchThread));
    }

    void Watchdog::AddToWatchlist(Watchable* toWatch) {
        m_Watchlist.push_back(WatchlistEntry(toWatch, 0));
    }

    void Watchdog::Arm() {
        for (auto& entry : m_Watchlist) {
            entry.first->Activate();
        }
    }

    void Watchdog::Disarm() {
        for (auto& entry : m_Watchlist) {
            entry.first->Deactivate();
            entry.second = 0;
        }
    }

    void Watchdog::WatchdogCallback() {
        m_Logger->SendLog(LogPacket::Severity::FATAL, "Watchdog timeout detected");
        NVIC_SystemReset();
    }

    void Watchdog::StartWatchThread() {
        static auto lastTime = Kernel::Clock::now();
        while (true) {
            if (IsActivated()) {
                // m_Logger->SendLog(LogPacket::Severity::DEBUG, "Watchdog checking components");

                auto timeElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    Kernel::Clock::now() - lastTime);
                    
                for (auto& entry : m_Watchlist) {
                    if (!entry.first->IsActivated()) {
                        continue;
                    }

                    entry.second += timeElapsedMs.count();

                    if (entry.second > entry.first->GetUpdateInterval()) {
                        if (entry.first->CheckActivity()) {
                            entry.second = 0;
                        } else {
                            entry.second += timeElapsedMs.count();
                            
                            if (entry.second > entry.first->GetMaxInactivityLimitMs()) {
                                m_Logger->SendLog(LogPacket::Severity::FATAL, 
                                                "Watchdog triggered for " + entry.first->GetName());
                                entry.first->WatchdogTrigger();
                            }
                        }
                    }
                }
            }

            IncCount();
            lastTime = Kernel::Clock::now();
            ThisThread::sleep_for(std::chrono::milliseconds(m_WatchdogIntervalMs));
        }
    }

} // namespace tritonai::gkc
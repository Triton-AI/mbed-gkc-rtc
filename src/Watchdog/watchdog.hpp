/**
 * @file watchdog.hpp
 * @brief Defines the Watchdog class for monitoring component activity
 */

#pragma once

#include <cstdint>
#include <stdint.h>
#include <utility>
#include <vector>

#include "Tools/logger.hpp"
#include "watchable.hpp"

namespace tritonai::gkc {

    class Watchdog : public Watchable {
    public:
        Watchdog() = delete;
        Watchdog(uint32_t update_interval_ms, uint32_t max_inactivity_limit_ms,
                uint32_t wakeup_every_ms, ILogger* logger);

        void AddToWatchlist(Watchable* to_watch);
        void Arm();
        void Disarm();

        void WatchdogCallback(); // Watchable API

    protected:
        typedef uint32_t TimeElapsed;
        typedef std::pair<Watchable*, TimeElapsed> WatchlistEntry;
        typedef std::vector<WatchlistEntry> Watchlist;
        Watchlist m_Watchlist{};
        Thread m_WatchThread{osPriorityNormal, OS_STACK_SIZE, nullptr, "watch_thread"};
        uint32_t m_WatchdogIntervalMs;

        void StartWatchThread();
        ILogger* m_Logger;
    };

} // namespace tritonai::gkc
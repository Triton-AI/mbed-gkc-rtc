/**
 * @file watchable.hpp
 * @brief Interface for components that can be monitored by the watchdog
 */

#pragma once

#include <stdint.h>
#include <iostream>
#include <string>
#include "mbed.h"

namespace tritonai::gkc {

class Watchable {
public:
    Watchable(uint32_t updateIntervalMs, uint32_t maxInactivityLimitMs, std::string name)
        : m_UpdateIntervalMs(updateIntervalMs),
        m_MaxInactivityLimitMs(maxInactivityLimitMs),
        m_Name(name) {}

    void Activate() { m_Active = true; }
    void Deactivate() { m_Active = false; }
    void IncCount() { ++m_RollingCounter; }
    uint32_t GetUpdateInterval() { return m_UpdateIntervalMs; }
    void SetUpdateInterval(const uint32_t& updateIntervalMs) {
        this->m_UpdateIntervalMs = updateIntervalMs;
    }
    bool IsActivated() { return m_Active; }
    uint32_t GetMaxInactivityLimitMs() { return m_MaxInactivityLimitMs; }
    virtual bool CheckActivity() {
        bool activity = m_LastCheckRollingCounterVal != m_RollingCounter;
        m_LastCheckRollingCounterVal = m_RollingCounter;
        return activity;
    }
    void Attach(Callback<void()> func) { m_CallbackFunc = func; }
    void WatchdogTrigger() { m_CallbackFunc(); }
    std::string GetName() { return m_Name; }

protected:
    bool m_Active = false;
    uint32_t m_RollingCounter = 0;
    uint32_t m_UpdateIntervalMs = 0;
    uint32_t m_MaxInactivityLimitMs = 0;
    Callback<void()> m_CallbackFunc;
    
private:
    uint32_t m_LastCheckRollingCounterVal = 0;
    std::string m_Name;
};

} // namespace tritonai::gkc
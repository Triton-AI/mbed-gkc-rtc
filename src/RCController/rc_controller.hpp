/**
 * @file RCController.hpp
 * @brief Remote Control interface for the gokart controller
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include "config.hpp"
#include "math.h"
#include "elrs_receiver.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"
#include "Watchdog/watchable.hpp"
#include "Tools/logger.hpp"
#include <Thread.h>

#ifdef ENABLE_USB_PASSTHROUGH
#include "USBJoystick/usb_joystick.hpp"
extern bool g_PassthroughEnabled;
#endif

namespace tritonai::gkc {

    struct Translation
    {
        double Normalize(int analogValue);
        double Steering(int steerVal);
        double Throttle(int throttleVal);
        double ThrottleRatio(int throttleVal);
        bool IsConstantThrottle(int throttleVal);
        double Brake(int brakeVal);
        bool IsActive(int leftToggle, int rightToggle);
        AutonomyMode GetAutonomyMode(int rightTriVal);
    };

    /**
     * @class RCController
     * @brief Manages remote control inputs and converts them to vehicle commands
     */
    class RCController : public Watchable
    {
    public:
        explicit RCController(GkcPacketSubscriber* sub, ILogger* logger);
        
        const RCControlGkcPacket& GetPacket() {
            m_IsReady = false;
            return m_Packet;
        }
        
#ifdef ENABLE_USB_PASSTHROUGH
        bool GetIndicatorState() const;
        bool IsUSBConnected() const { return m_USBConnected; }
#endif
        
    protected:
        void Update();
        Translation Map;
        Thread m_RCThread{osPriorityNormal, OS_STACK_SIZE*2, nullptr, "rc_thread"};
        void WatchdogCallback();

    private:
        elrc_receiver m_Receiver;
        RCControlGkcPacket m_Packet{};
        GkcPacketSubscriber* m_Sub;
        ILogger* m_Logger;
#ifdef ENABLE_USB_PASSTHROUGH
        USBJoystick m_Joystick;
#endif
        bool m_IsReady;
        float m_CurrentThrottle = 0.0;
#ifdef ENABLE_USB_PASSTHROUGH
        bool m_IndicatorState;
        bool m_USBConnected{false};
#endif
    };

} // namespace tritonai::gkc
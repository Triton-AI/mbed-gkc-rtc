/**
 * @file comm.hpp
 * @brief Communication manager for packet-based messaging
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>

#include "BufferedSerial.h"
#include "mbed.h"

#include "config.hpp"
#include "Watchdog/watchable.hpp"
#include "Tools/logger.hpp"

#include "tai_gokart_packet/gkc_packet_factory.hpp"
#include "tai_gokart_packet/gkc_packet_utils.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"

namespace tritonai::gkc {

    class CommManager : public Watchable {
    public:
        explicit CommManager(GkcPacketSubscriber* sub, ILogger* logger);
        void Send(const GkcPacket& packet);

    protected:
        ILogger* m_Logger;

        std::unique_ptr<GkcPacketFactory> m_Factory;
        Queue<GkcBuffer, SEND_QUEUE_SIZE> m_SendQueue;
        std::queue<std::shared_ptr<GkcBuffer>> m_SendQueueData;
        Thread m_SendThread{osPriorityNormal, OS_STACK_SIZE, nullptr, "send_thread"};

        std::unique_ptr<BufferedSerial> m_UartSerial;
        Thread m_UartSerialThread{osPriorityNormal, OS_STACK_SIZE, nullptr, "uart_serial_thread"};

        void RecvCallback();
        void WatchdogCallback();
        void SendThreadImpl();
        size_t SendImpl(const GkcBuffer& buffer);
    };

} // namespace tritonai::gkc
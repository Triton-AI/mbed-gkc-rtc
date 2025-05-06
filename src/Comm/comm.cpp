/**
 * @file comm.cpp
  * @brief Implementation of the communication manager
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include <chrono>
#include <memory>
#include <ratio>
#include <string>

#include "Kernel.h"
#include "comm.hpp"
#include "mbed.h"

namespace tritonai::gkc {

    CommManager::CommManager(GkcPacketSubscriber* sub, ILogger* logger)
        : Watchable(DEFAULT_COMM_POLL_INTERVAL_MS, DEFAULT_COMM_POLL_LOST_TOLERANCE_MS, "CommManager"),
        m_Logger(logger),
        m_Factory(std::make_unique<GkcPacketFactory>(sub, GkcPacketUtils::debug_cout)) 
    {
        Attach(callback(this, &CommManager::WatchdogCallback));
        m_Logger->SendLog(LogPacket::Severity::INFO, "CommManager initialized");

        m_UartSerial = std::make_unique<BufferedSerial>(UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
        m_UartSerialThread.start(mbed::callback(this, &CommManager::RecvCallback));

        m_SendThread.start(callback(this, &CommManager::SendThreadImpl));
    }

    void CommManager::Send(const GkcPacket& packet) {
        auto toSend = m_Factory->Send(packet);
        if (m_SendQueue.try_put(toSend.get())) {
            m_SendQueueData.push(toSend);
        }
    }

    size_t CommManager::SendImpl(const GkcBuffer& buffer) {
        if (!m_UartSerial->writable()) {
            m_Logger->SendLog(LogPacket::Severity::ERROR, "Serial not writable");
            return 0;
        }
        size_t bytes = m_UartSerial->write(buffer.data(), buffer.size());
        return bytes;
    }

    void CommManager::WatchdogCallback() {
        m_Logger->SendLog(LogPacket::Severity::FATAL, "CommManager watchdog timeout detected");
        NVIC_SystemReset();
    }

    void CommManager::RecvCallback() {
        static auto buffer = GkcBuffer(RECV_BUFFER_SIZE, 0);
        static auto waitTime = std::chrono::milliseconds(WAIT_READ_MS);
        
        while (!ThisThread::flags_get()) {
            IncCount();
            if (m_UartSerial->readable()) {
                auto numByteRead = m_UartSerial->read(buffer.data(), buffer.size());
                if (numByteRead > 0) {
                    RawGkcBuffer buff;
                    buff.data = buffer.data();
                    buff.size = numByteRead;

                    // std::cout << "Got something of size " << num_byte_read << "\n";
                    // cout << "Data: ";

                    // for (int i = 0; i < num_byte_read; i++) {
                    //   cout << hex << static_cast<int>(buffer[i]) << " ";
                    // }

                    // cout << endl;
                    m_Factory->Receive(buff);
                }
            }
        }
    }

    void CommManager::SendThreadImpl() {
        while (!ThisThread::flags_get()) {
            GkcBuffer* bufToSend;
            m_SendQueue.try_get_for(Kernel::wait_for_u32_forever, &bufToSend);
            SendImpl(*bufToSend);
            m_SendQueueData.pop();
        }
    }

} // namespace tritonai::gkc
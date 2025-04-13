/**
 * @file comm.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-02-10
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include <chrono>
#include <memory>
#include <ratio>
#include <string>

#include "Kernel.h"
#include "comm.hpp"
#include "mbed.h"

namespace tritonai {
namespace gkc {
CommManager::CommManager(GkcPacketSubscriber *sub, ILogger *logger)
    : Watchable(DEFAULT_COMM_POLL_INTERVAL_MS, DEFAULT_COMM_POLL_LOST_TOLERANCE_MS, "CommManager"),
      _logger(logger),
      factory_(std::make_unique<GkcPacketFactory>(sub, GkcPacketUtils::debug_cout)) 
{
  attach(callback(this, &CommManager::watchdog_callback));
  _logger->send_log(LogPacket::Severity::INFO, "CommManager initialized");

  uart_serial_ = std::make_unique<BufferedSerial>(UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
  uart_serial_thread_.start(mbed::callback(this, &CommManager::recv_callback));

  send_thread.start(callback(this, &CommManager::send_thread_impl));
}

void CommManager::send(const GkcPacket &packet) {
  auto to_send = factory_->Send(packet);
  if (send_queue_.try_put(to_send.get())) {
    send_queue_data_.push(to_send);
  }
}

size_t CommManager::send_impl(const GkcBuffer &buffer) {
  if (!uart_serial_->writable()) {
    _logger->send_log(LogPacket::Severity::ERROR, "Serial not writable");
    return 0;
  }
  size_t bytes = uart_serial_->write(buffer.data(), buffer.size());
  _logger->send_log(LogPacket::Severity::DEBUG, "Sent " + std::to_string(bytes) + " bytes");
  return bytes;
}

void CommManager::watchdog_callback() {
  _logger->send_log(LogPacket::Severity::FATAL, "CommManager watchdog timeout detected");
  NVIC_SystemReset();
}

void CommManager::recv_callback() {
  _logger->send_log(LogPacket::Severity::DEBUG, "CommManager received data");

  static auto buffer = GkcBuffer(RECV_BUFFER_SIZE, 0);
  static auto wait_time = std::chrono::milliseconds(WAIT_READ_MS);
  while (!ThisThread::flags_get()) {
    inc_count();
    if (uart_serial_->readable()) {
      auto num_byte_read = uart_serial_->read(buffer.data(), buffer.size());
      if (num_byte_read > 0) {
        RawGkcBuffer buff;
        buff.data = buffer.data();
        buff.size = num_byte_read;

        // std::cout << "Got something of size " << num_byte_read << "\n";
        // cout << "Data: ";

        // for (int i = 0; i < num_byte_read; i++) {
        //   cout << hex << static_cast<int>(buffer[i]) << " ";
        // }

        // cout << endl;

        factory_->Receive(buff);
      }
    }
    // TODO(haoru): log the number of frequence compromises (sleep_time >
    // wait_time)
  }

}

void CommManager::send_thread_impl() {
  while (!ThisThread::flags_get()) {
    GkcBuffer *buf_to_send;
    send_queue_.try_get_for(Kernel::wait_for_u32_forever, &buf_to_send);
    send_impl(*buf_to_send);
    send_queue_data_.pop();
  }
}
} // namespace gkc
} // namespace tritonai

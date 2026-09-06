#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace uart {
class UARTComponent;
}  // namespace uart
namespace xrce_dds {

// Serial shim for uxrCustomTransport over a UARTComponent bus.
// Maps directly onto write_array (send) and available + read_array
// (recv); both are non-blocking. Only compiled when a uart: bus exists
// (USE_UART); the component selects this path via set_transport_serial().
class XrceSerialTransport {
 public:
  void set_parent(uart::UARTComponent *parent) { this->parent_ = parent; }
  bool is_open() const { return this->parent_ != nullptr; }

  // Returns bytes transferred, 0 when there is nothing to move, < 0 when
  // no bus is attached. Never blocks.
  int send(const uint8_t *buf, size_t len);
  int recv(uint8_t *buf, size_t cap);

 protected:
  uart::UARTComponent *parent_{nullptr};
};

}  // namespace xrce_dds
}  // namespace esphome

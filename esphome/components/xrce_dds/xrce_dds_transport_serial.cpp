#include "xrce_dds_transport_serial.h"

#ifdef USE_UART
#include "esphome/components/uart/uart_component.h"
#endif

namespace esphome {
namespace xrce_dds {

int XrceSerialTransport::send(const uint8_t *buf, size_t len) {
#ifdef USE_UART
  if (this->parent_ == nullptr || buf == nullptr || len == 0)
    return -1;
  this->parent_->write_array(buf, len);
  return (int) len;
#else
  (void) buf;
  (void) len;
  return -1;
#endif
}

int XrceSerialTransport::recv(uint8_t *buf, size_t cap) {
#ifdef USE_UART
  if (this->parent_ == nullptr || buf == nullptr || cap == 0)
    return -1;
  size_t avail = this->parent_->available();
  if (avail == 0)
    return 0;
  size_t n = avail < cap ? avail : cap;
  return this->parent_->read_array(buf, n) ? (int) n : -1;
#else
  (void) buf;
  (void) cap;
  return -1;
#endif
}

}  // namespace xrce_dds
}  // namespace esphome

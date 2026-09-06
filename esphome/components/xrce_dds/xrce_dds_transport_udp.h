#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace xrce_dds {

// Unicast UDP shim for uxrCustomTransport, owning its own datagram socket.
//
// ESPHome's UDPComponent is broadcast-oriented (send_packet + listeners)
// with no unicast send-to/recv-from API, so this shim uses the POSIX
// socket API directly (available on both ESP-IDF and Arduino-ESP32 via
// lwIP) and declares its budget via socket.consume_sockets(1, UDP) in
// __init__.py. The socket is connect()ed to the agent: outbound datagrams
// go to the agent and inbound ones are filtered to the agent by the stack.
class XrceUdpTransport {
 public:
  XrceUdpTransport() = default;
  ~XrceUdpTransport() { this->close(); }

  XrceUdpTransport(const XrceUdpTransport &) = delete;
  XrceUdpTransport &operator=(const XrceUdpTransport &) = delete;

  bool open(const char *ip, uint16_t port);
  void close();
  bool is_open() const { return this->fd_ >= 0; }

  // Returns bytes transferred, 0 when the peer sent nothing (non-blocking),
  // < 0 on error. Never blocks.
  int send(const uint8_t *buf, size_t len);
  int recv(uint8_t *buf, size_t cap);

 protected:
  int fd_{-1};
};

}  // namespace xrce_dds
}  // namespace esphome

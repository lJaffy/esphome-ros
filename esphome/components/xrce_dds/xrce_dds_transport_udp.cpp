#include "xrce_dds_transport_udp.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "esphome/core/log.h"

namespace esphome {
namespace xrce_dds {

static const char *const TAG = "xrce_dds";

bool XrceUdpTransport::open(const char *ip, uint16_t port) {
  this->close();
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    ESP_LOGE(TAG, "UDP socket() failed: %d", errno);
    return false;
  }
  if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0) {
    ESP_LOGE(TAG, "UDP fcntl(O_NONBLOCK) failed: %d", errno);
    ::close(fd);
    return false;
  }
  // NOTE: no SO_SNDBUF/SO_RCVBUF tuning here — lwIP rejects them with
  // ENOPROTOOPT (errno 109). Socket stays at stack defaults.
  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
    ESP_LOGE(TAG, "Invalid agent_address '%s': IPv4 literal required", ip);
    ::close(fd);
    return false;
  }
  if (connect(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "UDP connect(%s:%u) failed: %d", ip, port, errno);
    ::close(fd);
    return false;
  }
  this->fd_ = fd;
  return true;
}

void XrceUdpTransport::close() {
  if (this->fd_ >= 0) {
    ::close(this->fd_);
    this->fd_ = -1;
  }
}

int XrceUdpTransport::send(const uint8_t *buf, size_t len) {
  if (this->fd_ < 0 || buf == nullptr || len == 0)
    return -1;
  ssize_t n = ::send(this->fd_, buf, len, MSG_DONTWAIT);
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    return 0;
  return (int) n;
}

int XrceUdpTransport::recv(uint8_t *buf, size_t cap) {
  if (this->fd_ < 0 || buf == nullptr || cap == 0)
    return -1;
  ssize_t n = ::recv(this->fd_, buf, cap, MSG_DONTWAIT);
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    return 0;
  return (int) n;
}

}  // namespace xrce_dds
}  // namespace esphome

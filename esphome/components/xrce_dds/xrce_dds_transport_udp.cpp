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

// lwIP buffers sized for ~32 in-flight 1472 B fragments (one 40 kB frame +
// ACK/heartbeat headroom). setsockopt may clamp on constrained builds; best
// effort only, never fatal.
constexpr int UDP_SND_BUF = 65536;
constexpr int UDP_RCV_BUF = 65536;

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
  int snd = UDP_SND_BUF;
  if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &snd, sizeof(snd)) < 0)
    ESP_LOGW(TAG, "UDP SO_SNDBUF %d failed: %d", snd, errno);
  int rcv = UDP_RCV_BUF;
  if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv)) < 0)
    ESP_LOGW(TAG, "UDP SO_RCVBUF %d failed: %d", rcv, errno);
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

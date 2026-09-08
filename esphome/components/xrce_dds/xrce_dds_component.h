#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <utility>

#include "xrce_dds_vendor.h"

#include "esphome/core/component.h"

#include "../ros2/ros2_middleware.h"
#include "../ros2/ros2_types.h"
#include "xrce_dds_codec.h"
#include "xrce_dds_transport_serial.h"
#include "xrce_dds_transport_udp.h"

namespace esphome {
namespace uart {
class UARTComponent;
}  // namespace uart
namespace xrce_dds {

// Compile-time caps. The YAML max_* knobs gate at runtime up to these.
constexpr size_t XRCE_MAX_TOPICS = 16;
constexpr size_t XRCE_MAX_READERS = 8;
constexpr size_t XRCE_MAX_WRITERS = 8;
constexpr size_t XRCE_STREAM_BUF_SIZE = 2048;
constexpr uint16_t XRCE_STREAM_HISTORY = 4;
// One history slot: samples larger than this (e.g. Odometry with zeroed
// covariances) go out via the fragmented write path instead.
constexpr size_t XRCE_STREAM_BLOCK = XRCE_STREAM_BUF_SIZE / XRCE_STREAM_HISTORY;
constexpr size_t XRCE_TOPIC_NAME_LEN = 96;
constexpr size_t XRCE_XML_BUF_LEN = 384;

enum class TransportType : uint8_t {
  TRANSPORT_UDP,
  TRANSPORT_SERIAL,
};

enum class LinkState : uint8_t {
  LINK_DOWN,
  LINK_UP,
};

class XrceDdsComponent : public Component, public ros2::Ros2Middleware {
 public:
  struct ReaderEntry {
    std::string topic;
    const ros2::TypeDef *type{nullptr};
    ros2::SampleCallback cb;
    bool reliable{true};
    uxrObjectId topic_id{};
    uxrObjectId reader_id{};
    bool created{false};
  };

  struct WriterEntry {
    std::string topic;
    const ros2::TypeDef *type{nullptr};
    bool reliable{true};
    uxrObjectId topic_id{};
    uxrObjectId writer_id{};
    bool created{false};
  };

  XrceDdsComponent() = default;
  ~XrceDdsComponent() override;

  void set_agent_address(const std::string &address) { this->agent_address_ = address; }
  void set_agent_port(uint16_t port) { this->agent_port_ = port; }
  void set_transport_udp() { this->transport_ = TransportType::TRANSPORT_UDP; }
  void set_transport_serial(uart::UARTComponent *parent);
  void set_domain_id(uint8_t domain_id) { this->domain_id_ = domain_id; }
  void set_client_name(const std::string &name) { this->client_name_ = name; }
  void set_max_packet_length(uint16_t len) { this->max_packet_length_ = len; }
  void set_process_interval(uint32_t ms) { this->process_interval_ms_ = ms; }
  void set_keepalive_timeout(uint32_t ms) { this->keepalive_timeout_ms_ = ms; }
  void set_max_topics(uint8_t n) { this->max_topics_ = n; }
  void set_max_datawriters(uint8_t n) { this->max_datawriters_ = n; }
  void set_max_datareaders(uint8_t n) { this->max_datareaders_ = n; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  bool subscribe(const std::string &topic, const ros2::TypeDef *type, ros2::SampleCallback cb,
                 const ros2::MiddlewareOptions *opts = nullptr) override;
  bool publish(const std::string &topic, const ros2::TypeDef *type, const void *sample, size_t len,
               const ros2::MiddlewareOptions *opts = nullptr) override;
  bool publish_image(const std::string &topic, const uint8_t *jpeg, size_t len,
                     const ros2::MiddlewareOptions *opts = nullptr) override;
  bool connected() const override { return this->link_ == LinkState::LINK_UP; }
  const char *name() const override { return "xrce_dds"; }

  // Called from the C transport callbacks (args back-pointer), so public.
  int transport_write(const uint8_t *buf, size_t len);
  int transport_read(uint8_t *buf, size_t len);
  bool pump_once();

 protected:
  void drop_link_();
  bool try_connect_();
  bool create_pending_entities_();
  bool create_reader_(ReaderEntry &entry);
  bool create_writer_(WriterEntry &entry);
  ReaderEntry *find_reader_(const std::string &topic);
  WriterEntry *find_writer_(const std::string &topic);
  ReaderEntry *find_reader_by_id_(uxrObjectId id);
  // Distinct DDS topics across readers+writers. Each new topic mints a
  // topic entity on the agent, bounded by max_topics_.
  size_t count_topics_();
  bool topic_allowed_(const std::string &topic);
  void on_data_(uxrObjectId reader_id, ucdrBuffer *ub, uint16_t length);
  static void topic_trampoline_(uxrSession *session, uxrObjectId object_id, uint16_t request_id,
                                uxrStreamId stream_id, ucdrBuffer *ub, uint16_t length, void *args);

  std::string agent_address_;
  uint16_t agent_port_{8888};
  TransportType transport_{TransportType::TRANSPORT_UDP};
  uint8_t domain_id_{0};
  std::string client_name_{"esp32-node"};
  uint16_t max_packet_length_{512};
  uint32_t process_interval_ms_{10};
  uint32_t keepalive_timeout_ms_{5000};
  uint8_t max_topics_{16};
  uint8_t max_datawriters_{8};
  uint8_t max_datareaders_{8};

  LinkState link_{LinkState::LINK_DOWN};
  uint32_t next_attempt_{0};
  uint32_t last_pump_{0};
  uint32_t last_rx_{0};

  uxrSession session_{};
  uxrCustomTransport custom_{};
  bool session_init_{false};
  bool participant_created_{false};
  bool publisher_created_{false};
  bool subscriber_created_{false};
  uxrStreamId out_stream_{};
  uxrStreamId in_stream_{};
  uxrStreamId img_stream_{};
  // Best-effort pair for qos: best_effort endpoints. Default client profile
  // allows exactly one of each; images always stay on img_stream_.
  uxrStreamId out_be_stream_{};
  uxrStreamId in_be_stream_{};
  uxrObjectId participant_id_{};
  uxrObjectId publisher_id_{};
  uxrObjectId subscriber_id_{};
  uint16_t next_topic_n_{1};
  uint16_t next_reader_n_{1};
  uint16_t next_writer_n_{1};

  XrceUdpTransport udp_;
  XrceSerialTransport serial_;
  XcdrCodec codec_;

  std::array<ReaderEntry, XRCE_MAX_READERS> readers_{};
  size_t num_readers_{0};
  std::array<WriterEntry, XRCE_MAX_WRITERS> writers_{};
  size_t num_writers_{0};

  std::array<uint8_t, XRCE_STREAM_BUF_SIZE> out_buf_{};
  std::array<uint8_t, XRCE_STREAM_BUF_SIZE> in_buf_{};
  std::array<uint8_t, XRCE_STREAM_BUF_SIZE> out_be_buf_{};
  std::array<uint8_t, UXR_CONFIG_CUSTOM_TRANSPORT_MTU> img_buf_{};
  // Single-threaded main loop: one shared scratch sample, no per-message heap.
  uint8_t sample_buf_[sizeof(ros2::JointTrajectoryMsg)]{0};
  // Cumulative transport counters (never reset; integrity signal for HIL).
  uint32_t tx_ok_{0};
  uint32_t tx_fail_{0};
  uint32_t rx_count_{0};
};

}  // namespace xrce_dds
}  // namespace esphome

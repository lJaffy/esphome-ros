#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

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
// Image stream: small slots x deep history so each XRCE fragment fits in one
// UDP datagram (no IP fragmentation). Slot wire size ~= IMG_BUF/HISTORY
// (~1412 B with 44 kB/32) stays under the 1472 B MTU; 19 kB frames need ~14
// slots, 40 kB ~29, all without mid-frame ACKs. History must stay a power
// of two for the vendored seq-num arithmetic.
constexpr size_t XRCE_IMG_BUF_SIZE = 45056;
constexpr uint16_t XRCE_IMG_HISTORY = 32;
constexpr size_t XRCE_TOPIC_NAME_LEN = 96;
constexpr size_t XRCE_XML_BUF_LEN = 384;

// Worker task topology (Phase 2, hard-coded): one task on the APP core owns
// every uxr_* call, stream/table/buffer state, and all counters. The loop()
// thread only enqueues queue items below and reads atomics. Queues carry
// memcpy payloads (never pointers into caller buffers) so no mutex is needed
// on the data path; FreeRTOS queue ops provide the memory barriers.
constexpr int XRCE_WORKER_CORE = 1;  // APP core, alongside Arduino/IDF loopTask
constexpr int XRCE_WORKER_PRIO = 5;
constexpr size_t XRCE_WORKER_STACK = 24576;
constexpr size_t XRCE_OUT_QUEUE_DEPTH = 8;
constexpr size_t XRCE_CTRL_QUEUE_DEPTH = 16;
// Image mailbox cap: full-size frames on camera builds, token size
// elsewhere (no camera can produce frames, so the mailbox stays empty).
// Non-camera boards save ~98 kB of RAM this way (mailbox + stage below).
#ifdef USE_CAMERA
constexpr size_t XRCE_IMG_MAILBOX_MAX = 49152;  // 48 kB frame cap (> 44 kB stream)
#else
constexpr size_t XRCE_IMG_MAILBOX_MAX = 64;
#endif
constexpr uint8_t XRCE_CTRL_SUBSCRIBE = 1;
constexpr size_t XRCE_SUB_STAGING_MAX = 16;

// Queue-safe copy of the middleware options (frame_id materialized: the
// live MiddlewareOptions only borrows its caller's char buffer).
struct QueueOpts {
  bool reliable{true};
  bool qos_explicit{false};
  bool use_b64{true};
  int32_t stamp_sec{0};
  uint32_t stamp_nsec{0};
  char frame_id[ros2::ROS2_FRAME_ID_LEN]{0};
};

// Largest MCU sample bounds every queue payload (JointTrajectoryMsg ~1.1 kB).
constexpr size_t XRCE_SAMPLE_MAX = sizeof(ros2::JointTrajectoryMsg);
static_assert(sizeof(ros2::JointStateMsg) <= XRCE_SAMPLE_MAX, "sample payload too small");
static_assert(sizeof(ros2::TFMessageMsg) <= XRCE_SAMPLE_MAX, "sample payload too small");
static_assert(sizeof(ros2::ImuMsg) <= XRCE_SAMPLE_MAX, "sample payload too small");
static_assert(sizeof(ros2::OdometryMsg) <= XRCE_SAMPLE_MAX, "sample payload too small");

struct OutboundItem {
  char topic[XRCE_TOPIC_NAME_LEN]{0};
  const ros2::TypeDef *type{nullptr};
  QueueOpts opts;
  uint16_t len{0};
  uint8_t data[XRCE_SAMPLE_MAX]{0};
};

struct CtrlItem {
  uint8_t op{0};  // XRCE_CTRL_SUBSCRIBE
  char topic[XRCE_TOPIC_NAME_LEN]{0};
  const ros2::TypeDef *type{nullptr};
  QueueOpts opts;
  uint8_t slot{0};
};

struct ImageItem {
  char topic[XRCE_TOPIC_NAME_LEN]{0};
  QueueOpts opts;
  uint32_t len{0};
  uint32_t seq{0};
  uint8_t jpeg[XRCE_IMG_MAILBOX_MAX]{0};
};

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
  bool connected() const override { return this->link_up_.load(std::memory_order_relaxed); }
  const char *name() const override { return "xrce_dds"; }

  // Called from the C transport callbacks (args back-pointer), so public.
  // Worker-only: invoked via session pump/prepare on the worker task.
  int transport_write(const uint8_t *buf, size_t len);
  int transport_read(uint8_t *buf, size_t len);
  bool pump_once();

 protected:
  // Worker entry + per-iteration handlers. Everything below runs on the
  // worker task; the loop() thread never calls these directly.
  static void worker_trampoline_(void *arg);
  void worker_loop_();
  void handle_ctrl_(const CtrlItem &c);
  void handle_outbound_(const OutboundItem &o);
  void handle_image_(const ImageItem &img);
  void snapshot_tables_();  // worker-only: refresh dump snapshots
  // Former subscribe/publish/publish_image bodies, now worker-only. The
  // public overrides are thin enqueue wrappers (loop-safe, non-blocking).
  bool subscribe_on_worker_(const char *topic, const ros2::TypeDef *type, uint8_t slot,
                            const QueueOpts &opts);
  bool publish_on_worker_(const char *topic, const ros2::TypeDef *type, const void *sample, size_t len,
                          const QueueOpts &opts);
  bool publish_image_on_worker_(const char *topic, const uint8_t *jpeg, size_t len,
                                const QueueOpts &opts);
  uint32_t now_ms_() const;
  void free_image_buffers_();  // setup-failure + destructor path only
  void drop_link_();
  void reset_img_stream_();
  bool pump_timed_(int timeout_ms);
  bool try_connect_();
  bool create_pending_entities_();
  bool create_reader_(ReaderEntry &entry);
  bool create_writer_(WriterEntry &entry);
  ReaderEntry *find_reader_(const std::string &topic);
  WriterEntry *find_writer_(const std::string &topic);
  // Hot-path variants: compare without constructing a std::string per call.
  ReaderEntry *find_reader_cstr_(const char *topic);
  WriterEntry *find_writer_cstr_(const char *topic);
  ReaderEntry *find_reader_by_id_(uxrObjectId id);
  // Distinct DDS topics across readers+writers. Each new topic mints a
  // topic entity on the agent, bounded by max_topics_.
  size_t count_topics_();
  bool topic_allowed_(const std::string &topic);
  bool topic_allowed_cstr_(const char *topic);
  void on_data_(uxrObjectId reader_id, ucdrBuffer *ub, uint16_t length);
  static void topic_trampoline_(uxrSession *session, uxrObjectId object_id, uint16_t request_id,
                                uxrStreamId stream_id, ucdrBuffer *ub, uint16_t length, void *args);

  std::string agent_address_;
  uint16_t agent_port_{8888};
  TransportType transport_{TransportType::TRANSPORT_UDP};
  uint8_t domain_id_{0};
  std::string client_name_{"esp32-node"};
  uint16_t max_packet_length_{1472};
  uint32_t process_interval_ms_{10};
  uint32_t keepalive_timeout_ms_{5000};
  uint8_t max_topics_{16};
  uint8_t max_datawriters_{8};
  uint8_t max_datareaders_{8};

  LinkState link_{LinkState::LINK_DOWN};
  // Loop-visible snapshot of link_ (worker writes, loop wrappers read).
  std::atomic<bool> link_up_{false};
  uint32_t next_attempt_{0};
  uint32_t last_pump_{0};
  // Inbound-data clock, read by dump_config() on the loop thread.
  std::atomic<uint32_t> last_rx_{0};
  // Last time the session reported fully-confirmed output. Separate from
  // last_rx_ (inbound data) so publish-only nodes still see agent ACKs.
  uint32_t last_confirm_{0};

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
  // Bulk image buffers are heap-allocated in setup() (PSRAM-preferred via
  // ExternalRAMAllocator): mailbox + stage + stream would overflow DRAM .bss
  // as statics (~142 kB). Null until setup() succeeds.
  uint8_t *img_buf_{nullptr};
  bool images_available_{false};
  // Single-threaded main loop: one shared scratch sample, no per-message heap.
  uint8_t sample_buf_[sizeof(ros2::JointTrajectoryMsg)]{0};
  // Cumulative transport counters (never reset; integrity signal for HIL).
  // Atomic: incremented on the worker, read by dump_config()/wrappers on loop.
  std::atomic<uint32_t> tx_ok_{0};
  std::atomic<uint32_t> tx_fail_{0};
  std::atomic<uint32_t> rx_count_{0};
  // Granular image drop reasons (subset of TX; tx_ok_/tx_fail_ kept for parsers).
  std::atomic<uint32_t> img_ok_{0};
  std::atomic<uint32_t> img_drop_link_down_{0};
  std::atomic<uint32_t> img_drop_no_writer_{0};
  std::atomic<uint32_t> img_drop_prepare_{0};
  std::atomic<uint32_t> img_drop_encode_{0};
  // Camera overwrote the 1-deep mailbox before the worker sent the frame.
  std::atomic<uint32_t> img_drop_mailbox_overwrite_{0};
  // Consecutive mid-frame encode failures; falls back to drop_link_ at threshold.
  // Worker-only (written + read on the worker task).
  uint8_t img_encode_fails_{0};
  // TEMP PROBE (tx-timing diagnosis; remove after): per-datagram send stats,
  // frame serialize cost, and ACK turnaround. Reported throttled, never
  // per-packet.
  uint32_t probe_dgrams_{0};
  uint32_t probe_bytes_{0};
  uint64_t probe_send_us_{0};
  uint32_t probe_send_max_us_{0};
  uint32_t probe_eagain_{0};
  uint32_t probe_send_err_{0};
  uint32_t probe_rx_dgrams_{0};
  uint32_t probe_rx_bytes_{0};
  uint32_t probe_ser_us_{0};
  uint32_t probe_ser_max_us_{0};
  uint32_t probe_last_frame_ms_{0};
  uint32_t probe_confirm_lat_ms_{0};
  uint32_t probe_confirm_max_ms_{0};
  uint32_t probe_frames_{0};
  uint32_t probe_last_log_ms_{0};

  // --- Worker handoff (Phase 2) -------------------------------------------
  // Queues are created in setup(); the task starts at the end of setup().
  // subscribe/publish/publish_image only touch these + atomics above.
  TaskHandle_t worker_{nullptr};
  QueueHandle_t out_queue_{nullptr};
  QueueHandle_t ctrl_queue_{nullptr};
  QueueHandle_t img_box_{nullptr};
  StaticQueue_t out_queue_ctrl_{};
  StaticQueue_t ctrl_queue_ctrl_{};
  StaticQueue_t img_box_ctrl_{};
  uint8_t out_queue_storage_[XRCE_OUT_QUEUE_DEPTH * sizeof(OutboundItem)]{};
  uint8_t ctrl_queue_storage_[XRCE_CTRL_QUEUE_DEPTH * sizeof(CtrlItem)]{};
  // Mailbox queue storage is heap-allocated with the buffers below (an
  // ImageItem is ~49 kB: far too big for .bss alongside the rest).
  uint8_t *img_box_storage_{nullptr};
  // SampleCallback staging: loop thread writes slot i once before enqueueing
  // its control item; the worker moves it into readers_[] exactly once.
  // Queue send/receive barriers make the handoff safe without a mutex.
  std::array<ros2::SampleCallback, XRCE_SUB_STAGING_MAX> sub_staging_{};
  size_t num_sub_staging_{0};  // loop-only
  uint32_t img_seq_{0};        // loop-only mailbox sequence
  uint32_t img_mailbox_seen_{0};  // worker-only last handled sequence
  // Loop-only mailbox staging (heap, like the queue storage: a 48 kB item
  // would overflow both the loop task stack and DRAM .bss). The worker only
  // ever sees the queue copy.
  ImageItem *img_stage_{nullptr};
  // Worker-only image receive buffer (heap: receiving the 1-deep mailbox
  // into a stack local smashed the heap past the worker stack top).
  ImageItem *img_work_{nullptr};
  // Table snapshots for dump_config() (loop thread): tables are
  // worker-exclusive, so the worker refreshes these after each mutation.
  std::atomic<uint32_t> snap_readers_{0};
  std::atomic<uint32_t> snap_writers_{0};
  std::atomic<uint32_t> snap_topics_{0};
};

}  // namespace xrce_dds
}  // namespace esphome

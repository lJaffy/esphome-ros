#include "xrce_dds_component.h"

#include <cstdio>
#include <cstring>

#include "esp_timer.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "../ros2/ros2_types.h"

// Internal vendored stream helpers (defs linked via xrce_dds_vendor.c).
// Forward-declared here so the public amalgamated header stays untouched.
// File scope: linkage specifications are not permitted inside function bodies.
extern "C" {
struct uxrOutputReliableStream *uxr_get_output_reliable_stream(struct uxrStreamStorage *,
                                                               uint8_t);
void uxr_reset_output_reliable_stream(struct uxrOutputReliableStream *);
}

namespace esphome {
namespace xrce_dds {

static const char *const TAG = "xrce_dds";

namespace {
constexpr uint32_t SESSION_KEY = 0x58445245;  // "XRCE"
constexpr uint16_t TOPIC_ID_BASE = 0x01;
constexpr uint16_t READER_ID_BASE = 0x11;
constexpr uint16_t WRITER_ID_BASE = 0x21;
constexpr int ENTITY_TIMEOUT_MS = 1000;
}  // namespace

XrceDdsComponent::~XrceDdsComponent() {
  // No uxr_delete_session here: it blocks on agent round-trips and the
  // destructor runs on the restart path. The agent reaps dead sessions via
  // its liveliness timeout. The UDP socket closes via XrceUdpTransport RAII.
  // Stop the worker so it can never touch a half-destroyed *this.
  if (this->worker_ != nullptr) {
    vTaskDelete(this->worker_);
    this->worker_ = nullptr;
  }
  this->free_image_buffers_();
}

void XrceDdsComponent::free_image_buffers_() {
  ExternalRAMAllocator<uint8_t> ext_alloc;
  if (this->img_buf_ != nullptr) {
    ext_alloc.deallocate(this->img_buf_, XRCE_IMG_BUF_SIZE);
    this->img_buf_ = nullptr;
  }
  if (this->img_box_storage_ != nullptr) {
    ext_alloc.deallocate(this->img_box_storage_, sizeof(ImageItem));
    this->img_box_storage_ = nullptr;
  }
  ExternalRAMAllocator<ImageItem> img_alloc;
  if (this->img_stage_ != nullptr) {
    img_alloc.deallocate(this->img_stage_, 1);
    this->img_stage_ = nullptr;
  }
  if (this->img_work_ != nullptr) {
    img_alloc.deallocate(this->img_work_, 1);
    this->img_work_ = nullptr;
  }
  this->images_available_ = false;
}

void XrceDdsComponent::set_transport_serial(uart::UARTComponent *parent) {
  this->serial_.set_parent(parent);
  this->transport_ = TransportType::TRANSPORT_SERIAL;
}

float XrceDdsComponent::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }

// --- Custom transport glue (C callbacks, args = this) -----------------------

namespace {

bool custom_open(uxrCustomTransport *transport) {
  (void) transport;
  // No-op: shim lifecycle is owned by try_connect_/drop_link_ (socket
  // opened before session creation, closed on drop). Read/write report
  // shim state per call, so a dead shim fails fast here instead of hanging
  // the session handshake.
  return true;
}

bool custom_close(uxrCustomTransport *transport) {
  (void) transport;
  return true;
}

size_t custom_write(uxrCustomTransport *transport, const uint8_t *buf, size_t len,
                    uint8_t *err) {
  auto *self = static_cast<XrceDdsComponent *>(transport->args);
  int n = self->transport_write(buf, len);
  if (n < 0) {
    *err = 1;
    return 0;
  }
  *err = 0;
  return (size_t) n;
}

size_t custom_read(uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                   uint8_t *err) {
  (void) timeout;  // Always non-blocking; the session pump uses timeout 0 and
                   // connect-time waits bound the total block instead.
  auto *self = static_cast<XrceDdsComponent *>(transport->args);
  int n = self->transport_read(buf, len);
  if (n < 0) {
    *err = 1;
    return 0;
  }
  *err = 0;
  return (size_t) n;
}

// Flush callback for fragmented image writes: non-blocking single pump.
// Returns false on transport failure (aborts the frame) or when the
// reliable history is full (congestion backpressure: frame dropped).
bool image_flush(uxrSession *session, void *args) {
  auto *self = static_cast<XrceDdsComponent *>(args);
  return self->pump_once();
}

}  // namespace

int XrceDdsComponent::transport_write(const uint8_t *buf, size_t len) {
  // TEMP PROBE: time the non-blocking send (syscall enqueue only, not wire).
  const int64_t t0 = esp_timer_get_time();
  int n = (this->transport_ == TransportType::TRANSPORT_UDP) ? this->udp_.send(buf, len)
                                                             : this->serial_.send(buf, len);
  const uint32_t dt = (uint32_t) (esp_timer_get_time() - t0);
  this->probe_dgrams_++;
  this->probe_send_us_ += dt;
  if (dt > this->probe_send_max_us_)
    this->probe_send_max_us_ = dt;
  if (n < 0) {
    this->probe_send_err_++;
  } else if (n == 0) {
    this->probe_eagain_++;
  } else {
    this->probe_bytes_ += (uint32_t) n;
  }
  return n;
}

int XrceDdsComponent::transport_read(uint8_t *buf, size_t len) {
  int n = (this->transport_ == TransportType::TRANSPORT_UDP) ? this->udp_.recv(buf, len)
                                                             : this->serial_.recv(buf, len);
  if (n > 0) {
    this->probe_rx_dgrams_++;
    this->probe_rx_bytes_ += (uint32_t) n;
  }
  return n;
}

bool XrceDdsComponent::pump_once() { return uxr_run_session_timeout(&this->session_, 0); }

bool XrceDdsComponent::pump_timed_(int timeout_ms) {
  return uxr_run_session_timeout(&this->session_, timeout_ms);
}

void XrceDdsComponent::reset_img_stream_() {
  // Light recovery for a poisoned fragmented image frame: reset only the
  // image reliable stream history, preserving participant/writers/socket.
  auto *s = uxr_get_output_reliable_stream(&this->session_.streams, this->img_stream_.index);
  if (s != nullptr)
    uxr_reset_output_reliable_stream(s);
  // Pinned fragments previously forced pump_once() to read "unconfirmed";
  // re-anchor confirmation so the keepalive gate does not false-trip.
  // Worker-only.
  this->last_confirm_ = this->now_ms_();
}

// --- Lifecycle --------------------------------------------------------------

void XrceDdsComponent::setup() {
  bool framing = (this->transport_ == TransportType::TRANSPORT_SERIAL);
  uxr_set_custom_transport_callbacks(&this->custom_, framing, custom_open, custom_close,
                                     custom_write, custom_read);
  if (!uxr_init_custom_transport(&this->custom_, this)) {
    ESP_LOGE(TAG, "Custom transport init failed");
    return;
  }
  uxr_init_session(&this->session_, &this->custom_.comm, SESSION_KEY);
  // Bulk image buffers on the heap, PSRAM-preferred (setup-time only, no
  // churn after): mailbox + stage + stream as statics overflowed DRAM .bss.
  // Without camera the mailbox constant is token-sized (see header), so
  // non-camera boards pay ~44 kB of heap for the idle image stream only.
  // Allocated here: the stream creation below borrows img_buf_ for the
  // session lifetime.
  ExternalRAMAllocator<uint8_t> ext_alloc;
  this->img_buf_ = ext_alloc.allocate(XRCE_IMG_BUF_SIZE);
  this->img_box_storage_ = ext_alloc.allocate(sizeof(ImageItem));
  ExternalRAMAllocator<ImageItem> img_alloc;
  this->img_stage_ = img_alloc.allocate(1);
  this->img_work_ = img_alloc.allocate(1);
  if (this->img_buf_ != nullptr && this->img_box_storage_ != nullptr &&
      this->img_stage_ != nullptr && this->img_work_ != nullptr) {
    this->images_available_ = true;
  } else {
    ESP_LOGE(TAG, "Image buffer allocation failed; image publishing disabled");
  }
  this->out_stream_ = uxr_create_output_reliable_stream(&this->session_, this->out_buf_.data(),
                                                        this->out_buf_.size(), XRCE_STREAM_HISTORY);
  this->in_stream_ = uxr_create_input_reliable_stream(&this->session_, this->in_buf_.data(),
                                                      this->in_buf_.size(), XRCE_STREAM_HISTORY);
  // No image stream without buffers: the publish_image wrapper rejects
  // everything while !images_available_, so the worker never touches it.
  if (this->images_available_) {
    this->img_stream_ = uxr_create_output_reliable_stream(&this->session_, this->img_buf_,
                                                          XRCE_IMG_BUF_SIZE, XRCE_IMG_HISTORY);
  }
  this->out_be_stream_ = uxr_create_output_best_effort_stream(&this->session_, this->out_be_buf_.data(),
                                                              this->out_be_buf_.size());
  this->in_be_stream_ = uxr_create_input_best_effort_stream(&this->session_);
  uxr_set_topic_callback(&this->session_, &XrceDdsComponent::topic_trampoline_, this);
  this->session_init_ = true;
  if (this->max_packet_length_ != UXR_CONFIG_CUSTOM_TRANSPORT_MTU) {
    ESP_LOGW(TAG, "max_packet_length %u ignored: vendored client MTU is %u",
             this->max_packet_length_, (unsigned) UXR_CONFIG_CUSTOM_TRANSPORT_MTU);
  }
  // Worker handoff queues (static storage for small items, heap for the
  // image mailbox; no heap churn after setup).
  this->out_queue_ = xQueueCreateStatic(XRCE_OUT_QUEUE_DEPTH, sizeof(OutboundItem),
                                        this->out_queue_storage_, &this->out_queue_ctrl_);
  this->ctrl_queue_ = xQueueCreateStatic(XRCE_CTRL_QUEUE_DEPTH, sizeof(CtrlItem),
                                         this->ctrl_queue_storage_, &this->ctrl_queue_ctrl_);
  this->img_box_ = nullptr;
  if (this->images_available_) {
    this->img_box_ = xQueueCreateStatic(1, sizeof(ImageItem), this->img_box_storage_,
                                        &this->img_box_ctrl_);
  }
  if (this->out_queue_ == nullptr || this->ctrl_queue_ == nullptr ||
      (this->images_available_ && this->img_box_ == nullptr)) {
    ESP_LOGE(TAG, "Worker queue creation failed");
    this->free_image_buffers_();
    return;
  }
  // Start last: every member the worker touches is initialized above.
  if (xTaskCreatePinnedToCore(&XrceDdsComponent::worker_trampoline_, "xrce_dds", XRCE_WORKER_STACK,
                              this, XRCE_WORKER_PRIO, &this->worker_, XRCE_WORKER_CORE) != pdPASS) {
    ESP_LOGE(TAG, "Worker task creation failed");
    this->worker_ = nullptr;
    this->free_image_buffers_();
    return;
  }
  ros2::MiddlewareRegistry::register_middleware("xrce_dds", this);
  ESP_LOGCONFIG(TAG, "XRCE-DDS middleware registered (agent=%s:%u, domain=%u)",
                this->agent_address_.c_str(), this->agent_port_, this->domain_id_);
}

void XrceDdsComponent::loop() {
  // Intentionally empty: the worker task owns pump/connect/publish/dispatch.
  // Inbound samples reach ros2 via its own queue (see Ros2Component::loop).
}

void XrceDdsComponent::worker_trampoline_(void *arg) {
  static_cast<XrceDdsComponent *>(arg)->worker_loop_();
  vTaskDelete(nullptr);
}

uint32_t XrceDdsComponent::now_ms_() const {
  // Boot-epoch ms like App.get_loop_component_start_time(), but callable
  // from the worker task (the App accessor is loop-cached).
  return (uint32_t) (esp_timer_get_time() / 1000LL);
}

void XrceDdsComponent::worker_loop_() {
  for (;;) {
    // Control plane first (reader registrations unblock entity creation).
    CtrlItem c;
    while (xQueueReceive(this->ctrl_queue_, &c, 0) == pdTRUE)
      this->handle_ctrl_(c);
    // Data plane: drain all pending small samples, then the latest image.
    // (Image received into the heap work buffer: a 49 kB stack local here
    // smashed the heap past the worker stack top.)
    OutboundItem o;
    while (xQueueReceive(this->out_queue_, &o, 0) == pdTRUE)
      this->handle_outbound_(o);
    if (this->img_box_ != nullptr && this->img_work_ != nullptr &&
        xQueueReceive(this->img_box_, this->img_work_, 0) == pdTRUE)
      this->handle_image_(*this->img_work_);
    // Session pump / connect state machine (moved from loop() verbatim,
    // with worker-epoch now). uxr_run_session_timeout(0) reports output
    // confirmation, not transport health: a just-queued image reads
    // "unconfirmed" until agent ACKs arrive. Only drop after a full
    // keepalive window without confirmation.
    // KNOWN GAP: run health only catches transport errors. A silently dead
    // UDP agent (blackhole, no ICMP) looks healthy until traffic fails.
    // HIL follow-up: periodic time-sync ping when now - last_rx_ is large.
    if (!this->session_init_) {
      vTaskDelay(pdMS_TO_TICKS(this->process_interval_ms_));
      continue;
    }
    const uint32_t now = this->now_ms_();
    if (this->link_ == LinkState::LINK_DOWN) {
      if (now >= this->next_attempt_)
        this->try_connect_();
    } else {
      if (now - this->last_pump_ >= this->process_interval_ms_) {
        this->last_pump_ = now;
        if (this->pump_once()) {
          this->last_confirm_ = now;
          // TEMP PROBE: end-to-end ACK turnaround for the last queued frame.
          if (this->probe_last_frame_ms_ != 0) {
            const uint32_t lat = now - this->probe_last_frame_ms_;
            this->probe_confirm_lat_ms_ = lat;
            if (lat > this->probe_confirm_max_ms_)
              this->probe_confirm_max_ms_ = lat;
            this->probe_last_frame_ms_ = 0;
          }
        } else if (now - this->last_confirm_ > this->keepalive_timeout_ms_) {
          this->drop_link_();
        }
      }
      // TEMP PROBE: throttled summary (every 10 frames or 5 s).
      if (this->probe_frames_ >= 10 || now - this->probe_last_log_ms_ >= 5000) {
        const uint32_t avg_send =
            this->probe_dgrams_ != 0 ? (uint32_t) (this->probe_send_us_ / this->probe_dgrams_) : 0;
        ESP_LOGI(TAG,
                 "PROBE tx: %u dgrams %u B (rx %u dgrams %u B) send avg/max %u/%u us eagain %u err "
                 "%u | "
                 "ser %u/%u us frames %u confirm lat %u/%u ms ok %u fail %u",
                 (unsigned) this->probe_dgrams_, (unsigned) this->probe_bytes_,
                 (unsigned) this->probe_rx_dgrams_, (unsigned) this->probe_rx_bytes_,
                 (unsigned) avg_send, (unsigned) this->probe_send_max_us_,
                 (unsigned) this->probe_eagain_, (unsigned) this->probe_send_err_,
                 (unsigned) this->probe_ser_us_, (unsigned) this->probe_ser_max_us_,
                 (unsigned) this->probe_frames_, (unsigned) this->probe_confirm_lat_ms_,
                 (unsigned) this->probe_confirm_max_ms_, (unsigned) this->tx_ok_.load(),
                 (unsigned) this->tx_fail_.load());
        this->probe_dgrams_ = 0;
        this->probe_bytes_ = 0;
        this->probe_send_us_ = 0;
        this->probe_send_max_us_ = 0;
        this->probe_eagain_ = 0;
        this->probe_send_err_ = 0;
        this->probe_rx_dgrams_ = 0;
        this->probe_rx_bytes_ = 0;
        this->probe_ser_us_ = 0;
        this->probe_ser_max_us_ = 0;
        this->probe_frames_ = 0;
        this->probe_last_log_ms_ = now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(this->process_interval_ms_));
  }
}

void XrceDdsComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "XRCE-DDS middleware:");
  ESP_LOGCONFIG(TAG, "  Agent: %s:%u, domain: %u, client: %s", this->agent_address_.c_str(),
                this->agent_port_, this->domain_id_, this->client_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Transport: %s",
                this->transport_ == TransportType::TRANSPORT_UDP ? "udp" : "serial");
  ESP_LOGCONFIG(TAG, "  Readers: %u/%u, writers: %u/%u", (unsigned) this->snap_readers_.load(),
                (unsigned) this->max_datareaders_, (unsigned) this->snap_writers_.load(),
                (unsigned) this->max_datawriters_);
  ESP_LOGCONFIG(TAG, "  Topics: %u/%u", (unsigned) this->snap_topics_.load(),
                (unsigned) this->max_topics_);
  const uint32_t now = App.get_loop_component_start_time();
  const uint32_t last_rx = this->last_rx_.load(std::memory_order_relaxed);
  ESP_LOGCONFIG(TAG, "  TX ok: %u, TX fail: %u, RX: %u, last RX age: %ums",
                (unsigned) this->tx_ok_.load(std::memory_order_relaxed),
                (unsigned) this->tx_fail_.load(std::memory_order_relaxed),
                (unsigned) this->rx_count_.load(std::memory_order_relaxed),
                (unsigned) (last_rx == 0 ? 0 : now - last_rx));
  ESP_LOGCONFIG(TAG, "  Images ok: %u drop(link/no-writer/prepare/encode/mbox): %u/%u/%u/%u/%u",
                (unsigned) this->img_ok_.load(std::memory_order_relaxed),
                (unsigned) this->img_drop_link_down_.load(std::memory_order_relaxed),
                (unsigned) this->img_drop_no_writer_.load(std::memory_order_relaxed),
                (unsigned) this->img_drop_prepare_.load(std::memory_order_relaxed),
                (unsigned) this->img_drop_encode_.load(std::memory_order_relaxed),
                (unsigned) this->img_drop_mailbox_overwrite_.load(std::memory_order_relaxed));
  if (this->worker_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Worker stack HWM: %u bytes free",
                  (unsigned) (uxTaskGetStackHighWaterMark(this->worker_) * sizeof(StackType_t)));
  }
}

void XrceDdsComponent::drop_link_() {
  // Always runs: closes the socket (no-op when already closed) and paces
  // the next attempt, even when the link was already down (connect-time
  // failures funnel through here too). Worker-only.
  this->udp_.close();
  this->next_attempt_ = this->now_ms_() + this->keepalive_timeout_ms_;
  if (this->link_ == LinkState::LINK_DOWN)
    return;
  this->link_ = LinkState::LINK_DOWN;
  this->link_up_.store(false, std::memory_order_relaxed);
  this->participant_created_ = false;
  this->publisher_created_ = false;
  this->subscriber_created_ = false;
  for (size_t i = 0; i < this->num_readers_; i++)
    this->readers_[i].created = false;
  for (size_t i = 0; i < this->num_writers_; i++)
    this->writers_[i].created = false;
  this->next_topic_n_ = 1;
  this->next_reader_n_ = READER_ID_BASE;
  this->next_writer_n_ = WRITER_ID_BASE;
  ESP_LOGW(TAG, "Link to agent lost; retrying");
}

bool XrceDdsComponent::try_connect_() {
  const uint32_t now = this->now_ms_();
  if (this->transport_ == TransportType::TRANSPORT_UDP) {
    if (!this->udp_.open(this->agent_address_.c_str(), this->agent_port_)) {
      this->drop_link_();
      return false;
    }
  } else {
    if (!this->serial_.is_open()) {
      ESP_LOGW(TAG, "Serial transport has no uart bus; check uart_id");
      this->drop_link_();
      return false;
    }
  }
  // Single session attempt: blocks up to one connection interval worst case
  // (agent down). Runs on the worker task, so the main loop keeps moving.
  if (!uxr_create_session_retries(&this->session_, 1)) {
    ESP_LOGW(TAG, "No agent at %s:%u; retrying", this->agent_address_.c_str(), this->agent_port_);
    this->drop_link_();
    return false;
  }
  if (!this->create_pending_entities_()) {
    ESP_LOGW(TAG, "Entity creation failed; retrying");
    this->drop_link_();
    return false;
  }
  this->link_ = LinkState::LINK_UP;
  this->link_up_.store(true, std::memory_order_relaxed);
  this->last_rx_ = now;
  this->last_pump_ = now;
  this->last_confirm_ = now;
  ESP_LOGI(TAG, "Connected to agent at %s:%u", this->agent_address_.c_str(), this->agent_port_);
  return true;
}

// --- Entity management ------------------------------------------------------

namespace {

void build_participant_xml(char *out, size_t cap, const char *name) {
  snprintf(out, cap, "<dds><participant><rtps><name>%s</name></rtps></participant></dds>", name);
}

void build_topic_xml(char *out, size_t cap, const char *topic, const char *type) {
  snprintf(out, cap, "<dds><topic><name>%s</name><dataType>%s</dataType></topic></dds>", topic, type);
}

void build_endpoint_xml(char *out, size_t cap, const char *kind, const char *topic,
                        const char *type) {
  snprintf(out, cap,
           "<dds><%s><topic><kind>NO_KEY</kind><name>%s</name><dataType>%s</dataType></topic></%s></"
           "dds>",
           kind, topic, type, kind);
}

// Same endpoint with an explicit reliability QoS. Only emitted for
// best_effort endpoints; reliable ones keep the bare form above so default
// behavior is byte-identical. If the agent rejects this dialect the entity
// creation fails loudly (never silently) — verify against your agent.
void build_endpoint_qos_xml(char *out, size_t cap, const char *kind, const char *topic,
                            const char *type, bool reliable) {
  if (reliable) {
    build_endpoint_xml(out, cap, kind, topic, type);
    return;
  }
  snprintf(out, cap,
           "<dds><%s><topic><kind>NO_KEY</kind><name>%s</name><dataType>%s</dataType></topic>"
           "<qos><reliability><kind>BEST_EFFORT</kind></reliability></qos></%s></dds>",
           kind, topic, type, kind);
}

bool topic_fits(const std::string &topic) {
  char tmp[XRCE_TOPIC_NAME_LEN];
  return dds_topic_name(topic.c_str(), tmp, sizeof(tmp));
}

// Materialize borrowed option strings into a queue-safe copy (loop thread).
QueueOpts make_queue_opts(const ros2::MiddlewareOptions *opts) {
  QueueOpts q;
  if (opts == nullptr)
    return q;
  q.reliable = opts->reliable;
  q.qos_explicit = opts->qos_explicit;
  q.use_b64 = opts->use_b64;
  q.stamp_sec = opts->stamp_sec;
  q.stamp_nsec = opts->stamp_nsec;
  if (opts->frame_id != nullptr) {
    strncpy(q.frame_id, opts->frame_id, sizeof(q.frame_id) - 1);
    q.frame_id[sizeof(q.frame_id) - 1] = '\0';
  }
  return q;
}

void copy_topic_str(char *dst, const std::string &src) {
  strncpy(dst, src.c_str(), XRCE_TOPIC_NAME_LEN - 1);
  dst[XRCE_TOPIC_NAME_LEN - 1] = '\0';
}

}  // namespace

XrceDdsComponent::ReaderEntry *XrceDdsComponent::find_reader_(const std::string &topic) {
  for (size_t i = 0; i < this->num_readers_; i++)
    if (this->readers_[i].topic == topic)
      return &this->readers_[i];
  return nullptr;
}

XrceDdsComponent::WriterEntry *XrceDdsComponent::find_writer_(const std::string &topic) {
  for (size_t i = 0; i < this->num_writers_; i++)
    if (this->writers_[i].topic == topic)
      return &this->writers_[i];
  return nullptr;
}

XrceDdsComponent::ReaderEntry *XrceDdsComponent::find_reader_cstr_(const char *topic) {
  for (size_t i = 0; i < this->num_readers_; i++)
    if (this->readers_[i].topic == topic)
      return &this->readers_[i];
  return nullptr;
}

XrceDdsComponent::WriterEntry *XrceDdsComponent::find_writer_cstr_(const char *topic) {
  for (size_t i = 0; i < this->num_writers_; i++)
    if (this->writers_[i].topic == topic)
      return &this->writers_[i];
  return nullptr;
}

XrceDdsComponent::ReaderEntry *XrceDdsComponent::find_reader_by_id_(uxrObjectId id) {
  for (size_t i = 0; i < this->num_readers_; i++)
    if (this->readers_[i].created && this->readers_[i].reader_id.id == id.id)
      return &this->readers_[i];
  return nullptr;
}

size_t XrceDdsComponent::count_topics_() {
  const std::string *seen[XRCE_MAX_READERS + XRCE_MAX_WRITERS];
  size_t n = 0;
  auto add = [&](const std::string &topic) {
    for (size_t i = 0; i < n; i++)
      if (*seen[i] == topic)
        return;
    seen[n++] = &topic;
  };
  for (size_t i = 0; i < this->num_readers_; i++)
    add(this->readers_[i].topic);
  for (size_t i = 0; i < this->num_writers_; i++)
    add(this->writers_[i].topic);
  return n;
}

bool XrceDdsComponent::topic_allowed_(const std::string &topic) {
  return this->topic_allowed_cstr_(topic.c_str());
}

bool XrceDdsComponent::topic_allowed_cstr_(const char *topic) {
  for (size_t i = 0; i < this->num_readers_; i++)
    if (this->readers_[i].topic == topic)
      return true;
  for (size_t i = 0; i < this->num_writers_; i++)
    if (this->writers_[i].topic == topic)
      return true;
  size_t budget = this->max_topics_ < XRCE_MAX_TOPICS ? this->max_topics_ : XRCE_MAX_TOPICS;
  if (this->count_topics_() >= budget) {
    ESP_LOGE(TAG, "Too many topics (max %u)", (unsigned) budget);
    return false;
  }
  return true;
}

bool XrceDdsComponent::create_pending_entities_() {
  char xml[XRCE_XML_BUF_LEN];
  uint16_t reqs[4 + 2 * (XRCE_MAX_READERS + XRCE_MAX_WRITERS)];
  uint8_t status[4 + 2 * (XRCE_MAX_READERS + XRCE_MAX_WRITERS)];
  size_t n = 0;

  auto need_topic = [&](const std::string &topic, uxrObjectId *id_out) -> const char * {
    for (size_t i = 0; i < this->num_readers_; i++)
      if (this->readers_[i].created && this->readers_[i].topic == topic) {
        *id_out = this->readers_[i].topic_id;
        return nullptr;
      }
    for (size_t i = 0; i < this->num_writers_; i++)
      if (this->writers_[i].created && this->writers_[i].topic == topic) {
        *id_out = this->writers_[i].topic_id;
        return nullptr;
      }
    return topic.c_str();
  };

  if (!this->participant_created_) {
    this->participant_id_ = uxr_object_id(TOPIC_ID_BASE, UXR_PARTICIPANT_ID);
    build_participant_xml(xml, sizeof(xml), this->client_name_.c_str());
    reqs[n++] = uxr_buffer_create_participant_xml(&this->session_, this->out_stream_,
                                                  this->participant_id_, this->domain_id_, xml,
                                                  UXR_REPLACE);
  }
  if (!this->publisher_created_) {
    this->publisher_id_ = uxr_object_id(TOPIC_ID_BASE, UXR_PUBLISHER_ID);
    reqs[n++] = uxr_buffer_create_publisher_xml(&this->session_, this->out_stream_,
                                                this->publisher_id_, this->participant_id_, "",
                                                UXR_REPLACE);
  }
  if (!this->subscriber_created_) {
    this->subscriber_id_ = uxr_object_id(TOPIC_ID_BASE, UXR_SUBSCRIBER_ID);
    reqs[n++] = uxr_buffer_create_subscriber_xml(&this->session_, this->out_stream_,
                                                 this->subscriber_id_, this->participant_id_, "",
                                                 UXR_REPLACE);
  }

  struct Pending {
    ReaderEntry *reader{nullptr};
    WriterEntry *writer{nullptr};
    uint16_t topic_req{UXR_INVALID_REQUEST_ID};
    uint16_t endpoint_req{UXR_INVALID_REQUEST_ID};
  };
  std::array<Pending, XRCE_MAX_READERS + XRCE_MAX_WRITERS> pending{};
  size_t num_pending = 0;

  char dds_topic[XRCE_TOPIC_NAME_LEN];
  for (size_t i = 0; i < this->num_readers_ && num_pending < pending.size(); i++) {
    ReaderEntry &e = this->readers_[i];
    if (e.created || e.type == nullptr)
      continue;
    // Length pre-validated at subscribe() time; a failure here would skip
    // before any request is buffered, keeping status[] aligned.
    if (!dds_topic_name(e.topic.c_str(), dds_topic, sizeof(dds_topic)))
      continue;
    Pending p;
    p.reader = &e;
    const char *need = need_topic(e.topic, &e.topic_id);
    if (need != nullptr) {
      e.topic_id = uxr_object_id(this->next_topic_n_++, UXR_TOPIC_ID);
      build_topic_xml(xml, sizeof(xml), dds_topic, dds_type_name(e.type));
      p.topic_req = uxr_buffer_create_topic_xml(&this->session_, this->out_stream_, e.topic_id,
                                                this->participant_id_, xml, UXR_REPLACE);
      reqs[n++] = p.topic_req;
    }
    e.reader_id = uxr_object_id(this->next_reader_n_++, UXR_DATAREADER_ID);
    build_endpoint_qos_xml(xml, sizeof(xml), "data_reader", dds_topic, dds_type_name(e.type),
                           e.reliable);
    p.endpoint_req = uxr_buffer_create_datareader_xml(&this->session_, this->out_stream_,
                                                      e.reader_id, this->subscriber_id_, xml,
                                                      UXR_REPLACE);
    reqs[n++] = p.endpoint_req;
    pending[num_pending++] = p;
  }
  for (size_t i = 0; i < this->num_writers_ && num_pending < pending.size(); i++) {
    WriterEntry &e = this->writers_[i];
    if (e.created || e.type == nullptr)
      continue;
    if (!dds_topic_name(e.topic.c_str(), dds_topic, sizeof(dds_topic)))
      continue;
    Pending p;
    p.writer = &e;
    const char *need = need_topic(e.topic, &e.topic_id);
    if (need != nullptr) {
      e.topic_id = uxr_object_id(this->next_topic_n_++, UXR_TOPIC_ID);
      build_topic_xml(xml, sizeof(xml), dds_topic, dds_type_name(e.type));
      p.topic_req = uxr_buffer_create_topic_xml(&this->session_, this->out_stream_, e.topic_id,
                                                this->participant_id_, xml, UXR_REPLACE);
      reqs[n++] = p.topic_req;
    }
    e.writer_id = uxr_object_id(this->next_writer_n_++, UXR_DATAWRITER_ID);
    build_endpoint_qos_xml(xml, sizeof(xml), "data_writer", dds_topic, dds_type_name(e.type),
                           e.reliable);
    p.endpoint_req = uxr_buffer_create_datawriter_xml(&this->session_, this->out_stream_,
                                                      e.writer_id, this->publisher_id_, xml,
                                                      UXR_REPLACE);
    reqs[n++] = p.endpoint_req;
    pending[num_pending++] = p;
  }

  if (n == 0)
    return true;
  if (!uxr_run_session_until_all_status(&this->session_, ENTITY_TIMEOUT_MS, reqs, status, n)) {
    ESP_LOGW(TAG, "Entity status timeout (%u requests)", (unsigned) n);
    return false;
  }

  size_t s = 0;
  if (!this->participant_created_) {
    if (status[s++] != UXR_STATUS_OK)
      return false;
    this->participant_created_ = true;
  }
  if (!this->publisher_created_) {
    if (status[s++] != UXR_STATUS_OK)
      return false;
    this->publisher_created_ = true;
  }
  if (!this->subscriber_created_) {
    if (status[s++] != UXR_STATUS_OK)
      return false;
    this->subscriber_created_ = true;
  }
  uxrDeliveryControl dc{};
  dc.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
  for (size_t i = 0; i < num_pending; i++) {
    Pending &p = pending[i];
    bool ok = true;
    if (p.topic_req != UXR_INVALID_REQUEST_ID)
      ok = status[s++] == UXR_STATUS_OK;
    if (ok && p.endpoint_req != UXR_INVALID_REQUEST_ID)
      ok = status[s++] == UXR_STATUS_OK;
    if (!ok)
      continue;
    if (p.reader != nullptr) {
      p.reader->created = true;
      uxrStreamId in = p.reader->reliable ? this->in_stream_ : this->in_be_stream_;
      uxr_buffer_request_data(&this->session_, this->out_stream_, p.reader->reader_id, in, &dc);
      ESP_LOGI(TAG, "Subscribed: %s (%s)", p.reader->topic.c_str(), p.reader->type->name);
    } else if (p.writer != nullptr) {
      p.writer->created = true;
      ESP_LOGI(TAG, "Advertising: %s (%s)", p.writer->topic.c_str(), p.writer->type->name);
    }
  }
  // Flush the READ_DATA requests without blocking.
  this->pump_once();
  return true;
}

bool XrceDdsComponent::create_reader_(ReaderEntry &entry) {
  (void) entry;
  return this->create_pending_entities_();
}

bool XrceDdsComponent::create_writer_(WriterEntry &entry) {
  (void) entry;
  return this->create_pending_entities_();
}

// --- Ros2Middleware (loop-safe enqueue wrappers) --------------------------------
// These run on the loop()/camera thread: validate, memcpy into a queue item,
// enqueue with zero timeout. Never touch session/stream/table state.

bool XrceDdsComponent::subscribe(const std::string &topic, const ros2::TypeDef *type,
                                 ros2::SampleCallback cb, const ros2::MiddlewareOptions *opts) {
  if (type == nullptr)
    return false;
  if (!topic_fits(topic)) {
    ESP_LOGE(TAG, "Topic name too long: %s", topic.c_str());
    return false;
  }
  if (this->ctrl_queue_ == nullptr)
    return false;
  if (this->num_sub_staging_ >= XRCE_SUB_STAGING_MAX)
    return false;
  // Stage the callback loop-side; the worker moves it into readers_[] once.
  const uint8_t slot = (uint8_t) this->num_sub_staging_++;
  this->sub_staging_[slot] = std::move(cb);
  CtrlItem c;
  c.op = XRCE_CTRL_SUBSCRIBE;
  copy_topic_str(c.topic, topic);
  c.type = type;
  c.opts.reliable = opts == nullptr || opts->reliable;
  c.opts.qos_explicit = opts != nullptr && opts->qos_explicit;
  c.slot = slot;
  if (xQueueSend(this->ctrl_queue_, &c, 0) != pdTRUE)
    return false;
  return true;
}

bool XrceDdsComponent::subscribe_on_worker_(const char *topic, const ros2::TypeDef *type,
                                            uint8_t slot, const QueueOpts &opts) {
  if (type == nullptr || slot >= XRCE_SUB_STAGING_MAX)
    return false;
  if (ReaderEntry *e = this->find_reader_cstr_(topic)) {
    e->cb = std::move(this->sub_staging_[slot]);
    e->type = type;
    e->reliable = opts.reliable;
    return true;
  }
  if (!this->topic_allowed_cstr_(topic)) {
    return false;
  }
  if (this->num_readers_ >= XRCE_MAX_READERS || this->num_readers_ >= this->max_datareaders_) {
    ESP_LOGE(TAG, "Too many readers for %s", topic);
    return false;
  }
  ReaderEntry &e = this->readers_[this->num_readers_++];
  e.topic = topic;
  e.type = type;
  e.cb = std::move(this->sub_staging_[slot]);
  e.reliable = opts.reliable;
  if (this->link_ == LinkState::LINK_UP)
    this->create_reader_(e);
  return true;
}

void XrceDdsComponent::snapshot_tables_() {
  this->snap_readers_.store((uint32_t) this->num_readers_, std::memory_order_relaxed);
  this->snap_writers_.store((uint32_t) this->num_writers_, std::memory_order_relaxed);
  this->snap_topics_.store((uint32_t) this->count_topics_(), std::memory_order_relaxed);
}

void XrceDdsComponent::handle_ctrl_(const CtrlItem &c) {
  if (c.op == XRCE_CTRL_SUBSCRIBE) {
    this->subscribe_on_worker_(c.topic, c.type, c.slot, c.opts);
    this->snapshot_tables_();
  }
}

bool XrceDdsComponent::publish(const std::string &topic, const ros2::TypeDef *type,
                               const void *sample, size_t len,
                               const ros2::MiddlewareOptions *opts) {
  if (type == nullptr || sample == nullptr)
    return false;
  if (!topic_fits(topic)) {
    ESP_LOGE(TAG, "Topic name too long: %s", topic.c_str());
    return false;
  }
  if (this->out_queue_ == nullptr)
    return false;
  if (len > sizeof(OutboundItem::data)) {
    ESP_LOGW(TAG, "Sample too large for %s (%u bytes)", topic.c_str(), (unsigned) len);
    this->tx_fail_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  OutboundItem o;
  copy_topic_str(o.topic, topic);
  o.type = type;
  o.opts = make_queue_opts(opts);
  o.len = (uint16_t) len;
  memcpy(o.data, sample, len);
  if (xQueueSend(this->out_queue_, &o, 0) != pdTRUE) {
    // Freshest wins: evict the oldest queued sample for the new one.
    OutboundItem drop;
    xQueueReceive(this->out_queue_, &drop, 0);
    xQueueSend(this->out_queue_, &o, 0);
    this->tx_fail_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGW(TAG, "Outbound queue full; dropped oldest for %s", topic.c_str());
  }
  return true;
}

bool XrceDdsComponent::publish_on_worker_(const char *topic, const ros2::TypeDef *type,
                                          const void *sample, size_t len, const QueueOpts &opts) {
  if (type == nullptr || sample == nullptr)
    return false;
  if (this->link_ != LinkState::LINK_UP)
    return false;
  WriterEntry *w = this->find_writer_cstr_(topic);
  if (w == nullptr) {
    if (!this->topic_allowed_cstr_(topic)) {
      this->tx_fail_++;
      return false;
    }
    if (this->num_writers_ >= XRCE_MAX_WRITERS || this->num_writers_ >= this->max_datawriters_) {
      ESP_LOGE(TAG, "Too many writers for %s", topic);
      this->tx_fail_++;
      return false;
    }
    WriterEntry &e = this->writers_[this->num_writers_++];
    e.topic = topic;
    e.type = type;
    e.reliable = opts.reliable;
    w = &e;
  }
  if (!w->created && !this->create_writer_(*w)) {
    this->tx_fail_++;
    return false;
  }
  uint32_t size = this->codec_.size_of(type, sample, len);
  if (size == 0) {
    ESP_LOGW(TAG, "Unencodable sample for %s", topic);
    this->tx_fail_++;
    return false;
  }
  uxrStreamId out = w->reliable ? this->out_stream_ : this->out_be_stream_;
  ucdrBuffer ub;
  bool prepared;
  if (size > XRCE_STREAM_BLOCK && w->reliable) {
    // Large sample (e.g. Odometry): fragment across reliable stream windows,
    // pumping without blocking like the image path. Best-effort streams have
    // no history split, so the whole buffer fits the sample directly.
    prepared = uxr_prepare_output_stream_fragmented(&this->session_, out, w->writer_id, &ub, size,
                                                    image_flush, this) != UXR_INVALID_REQUEST_ID;
  } else {
    prepared = uxr_prepare_output_stream(&this->session_, out, w->writer_id, &ub, size) !=
               UXR_INVALID_REQUEST_ID;
  }
  if (!prepared) {
    this->tx_fail_++;
    return false;
  }
  if (!this->codec_.serialize(&ub, type, sample, len) || ub.error) {
    ESP_LOGW(TAG, "XCDR encode failed for %s", topic);
    this->tx_fail_++;
    return false;
  }
  this->tx_ok_++;
  return true;
}

void XrceDdsComponent::handle_outbound_(const OutboundItem &o) {
  this->publish_on_worker_(o.topic, o.type, o.data, o.len, o.opts);
  this->snapshot_tables_();  // writer emplace happens lazily on first publish
}

bool XrceDdsComponent::publish_image(const std::string &topic, const uint8_t *jpeg, size_t len,
                                      const ros2::MiddlewareOptions *opts) {
  if (jpeg == nullptr || len == 0)
    return false;
  if (!topic_fits(topic)) {
    ESP_LOGE(TAG, "Topic name too long: %s", topic.c_str());
    return false;
  }
  if (this->img_box_ == nullptr || !this->images_available_ || this->img_stage_ == nullptr)
    return false;
  if (!this->link_up_.load(std::memory_order_relaxed)) {
    this->img_drop_link_down_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (len > XRCE_IMG_MAILBOX_MAX) {
    ESP_LOGW(TAG, "Image too large for %s (%u bytes)", topic.c_str(), (unsigned) len);
    this->tx_fail_.fetch_add(1, std::memory_order_relaxed);
    this->img_drop_prepare_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // 1-deep mailbox: the camera overwrites whatever the worker has not sent
  // yet. Overwrites are counted worker-side via the sequence gap.
  // (Staged on the heap: a 48 kB item would overflow the loop task stack.)
  ImageItem &img = *this->img_stage_;
  copy_topic_str(img.topic, topic);
  img.opts = make_queue_opts(opts);
  img.len = (uint32_t) len;
  img.seq = ++this->img_seq_;
  memcpy(img.jpeg, jpeg, len);
  xQueueOverwrite(this->img_box_, &img);
  return true;
}

bool XrceDdsComponent::publish_image_on_worker_(const char *topic, const uint8_t *jpeg, size_t len,
                                                const QueueOpts &opts) {
  if (jpeg == nullptr || len == 0)
    return false;
  if (this->link_ != LinkState::LINK_UP) {
    this->img_drop_link_down_++;
    return false;
  }
  const ros2::TypeDef *type = ros2::find_type("sensor_msgs/CompressedImage");
  if (type == nullptr)
    return false;
  WriterEntry *w = this->find_writer_cstr_(topic);
  if (w == nullptr) {
    if (!this->topic_allowed_cstr_(topic)) {
      this->tx_fail_++;
      this->img_drop_no_writer_++;
      return false;
    }
    if (this->num_writers_ >= XRCE_MAX_WRITERS || this->num_writers_ >= this->max_datawriters_) {
      ESP_LOGE(TAG, "Too many writers for %s", topic);
      this->tx_fail_++;
      this->img_drop_no_writer_++;
      return false;
    }
    WriterEntry &e = this->writers_[this->num_writers_++];
    e.topic = topic;
    e.type = type;
    e.reliable = true;  // frames always use the fragmented reliable image stream
    w = &e;
  }
  if (!w->created && !this->create_writer_(*w)) {
    this->tx_fail_++;
    this->img_drop_no_writer_++;
    return false;
  }
  // CompressedImage members: header (stamp/frame from opts when the bridge
  // has clock sync, else zero/empty), format "jpeg", then the length-prefixed
  // raw bytes. The ub window is one MTU; ucdr drives image_flush per full
  // window, which pumps the session without blocking, so frames of any size
  // stream in one call and abort fast on congestion (history exhaustion
  // -> ub.error).
  const int32_t sec = opts.stamp_sec;
  const uint32_t nsec = opts.stamp_nsec;
  const char *frame_id = opts.frame_id;
  uint32_t total = 8;  // stamp sec + nanosec
  total += (uint32_t) (ucdr_alignment(total, 4) + 4 + strlen(frame_id) + 1);
  total += (uint32_t) (ucdr_alignment(total, 4) + 4 + 5);  // "jpeg" + NUL
  total += (uint32_t) (ucdr_alignment(total, 4) + 4);      // data length
  total += (uint32_t) len;
  ucdrBuffer ub;
  constexpr int IMG_PREPARE_RETRY_MS = 50;
  constexpr uint8_t IMG_ENCODE_FAIL_LIMIT = 3;
  uint16_t prepare_id =
      uxr_prepare_output_stream_fragmented(&this->session_, this->img_stream_, w->writer_id, &ub,
                                           total, image_flush, this);
  if (prepare_id == UXR_INVALID_REQUEST_ID) {
    // One bounded wait for in-flight ACKs before dropping: transient
    // congestion (history full) often clears within tens of ms at 21kB/s.
    this->pump_timed_(IMG_PREPARE_RETRY_MS);
    prepare_id =
        uxr_prepare_output_stream_fragmented(&this->session_, this->img_stream_, w->writer_id, &ub,
                                             total, image_flush, this);
  }
  if (prepare_id == UXR_INVALID_REQUEST_ID) {
    ESP_LOGW(TAG, "Image prepare congested for %s (%u bytes)", topic, (unsigned) len);
    this->tx_fail_++;
    this->img_drop_prepare_++;
    return false;
  }
  // TEMP PROBE: serialize (memcpy/format) cost per frame.
  const int64_t ser_t0 = esp_timer_get_time();
  const bool ser_ok = ucdr_serialize_int32_t(&ub, sec) && ucdr_serialize_uint32_t(&ub, nsec) &&
                      ucdr_serialize_string(&ub, frame_id) && ucdr_serialize_string(&ub, "jpeg") &&
                      ucdr_serialize_uint32_t(&ub, (uint32_t) len) &&
                      ucdr_serialize_array_uint8_t(&ub, jpeg, len) && !ub.error;
  const uint32_t ser_dt = (uint32_t) (esp_timer_get_time() - ser_t0);
  this->probe_ser_us_ += ser_dt;
  if (ser_dt > this->probe_ser_max_us_)
    this->probe_ser_max_us_ = ser_dt;
  if (!ser_ok) {
    ESP_LOGW(TAG, "Image publish failed for %s (%u bytes)", topic, (unsigned) len);
    this->tx_fail_++;
    this->img_drop_encode_++;
    // Soft recovery: reset only the image stream so one poisoned frame does
    // not pin history and force every later pump to read "unconfirmed".
    // A truly dead agent still heals via the keepalive gate, and persistent
    // encode failures fall back to a full link drop.
    this->reset_img_stream_();
    if (++this->img_encode_fails_ >= IMG_ENCODE_FAIL_LIMIT) {
      this->img_encode_fails_ = 0;
      this->drop_link_();
    }
    return false;
  }
  this->tx_ok_++;
  this->img_ok_++;
  this->img_encode_fails_ = 0;
  this->probe_frames_++;
  this->probe_last_frame_ms_ = this->now_ms_();
  return true;
}

void XrceDdsComponent::handle_image_(const ImageItem &img) {
  // Mailbox overwrite accounting: skipped sequence numbers are frames the
  // camera replaced before the worker sent them (freshest wins by design).
  if (this->img_mailbox_seen_ != 0 && img.seq > this->img_mailbox_seen_ + 1)
    this->img_drop_mailbox_overwrite_ += (img.seq - this->img_mailbox_seen_ - 1);
  this->img_mailbox_seen_ = img.seq;
  this->publish_image_on_worker_(img.topic, img.jpeg, img.len, img.opts);
  this->snapshot_tables_();  // writer emplace happens lazily on first frame
}

// --- Inbound dispatch -------------------------------------------------------

void XrceDdsComponent::topic_trampoline_(uxrSession *session, uxrObjectId object_id,
                                         uint16_t request_id, uxrStreamId stream_id, ucdrBuffer *ub,
                                         uint16_t length, void *args) {
  (void) session;
  (void) request_id;
  (void) stream_id;
  (void) length;
  static_cast<XrceDdsComponent *>(args)->on_data_(object_id, ub, length);
}

void XrceDdsComponent::on_data_(uxrObjectId reader_id, ucdrBuffer *ub, uint16_t length) {
  (void) length;
  ReaderEntry *e = this->find_reader_by_id_(reader_id);
  if (e == nullptr || e->type == nullptr)
    return;
  this->last_rx_.store(this->now_ms_(), std::memory_order_relaxed);
  this->rx_count_.fetch_add(1, std::memory_order_relaxed);
  if (this->codec_.deserialize(ub, e->type, this->sample_buf_, sizeof(this->sample_buf_)))
    e->cb(this->sample_buf_, e->type->size);
}

}  // namespace xrce_dds
}  // namespace esphome

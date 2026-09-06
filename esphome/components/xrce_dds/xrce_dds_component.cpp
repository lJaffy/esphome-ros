#include "xrce_dds_component.h"

#include <cstdio>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include "../ros2/ros2_types.h"

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
  if (this->transport_ == TransportType::TRANSPORT_UDP)
    return this->udp_.send(buf, len);
  return this->serial_.send(buf, len);
}

int XrceDdsComponent::transport_read(uint8_t *buf, size_t len) {
  if (this->transport_ == TransportType::TRANSPORT_UDP)
    return this->udp_.recv(buf, len);
  return this->serial_.recv(buf, len);
}

bool XrceDdsComponent::pump_once() { return uxr_run_session_timeout(&this->session_, 0); }

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
  this->out_stream_ = uxr_create_output_reliable_stream(&this->session_, this->out_buf_.data(),
                                                        this->out_buf_.size(), XRCE_STREAM_HISTORY);
  this->in_stream_ = uxr_create_input_reliable_stream(&this->session_, this->in_buf_.data(),
                                                      this->in_buf_.size(), XRCE_STREAM_HISTORY);
  this->img_stream_ = uxr_create_output_reliable_stream(&this->session_, this->img_buf_.data(),
                                                        this->img_buf_.size(), XRCE_STREAM_HISTORY);
  uxr_set_topic_callback(&this->session_, &XrceDdsComponent::topic_trampoline_, this);
  this->session_init_ = true;
  if (this->max_packet_length_ != UXR_CONFIG_CUSTOM_TRANSPORT_MTU) {
    ESP_LOGW(TAG, "max_packet_length %u ignored: vendored client MTU is %u",
             this->max_packet_length_, (unsigned) UXR_CONFIG_CUSTOM_TRANSPORT_MTU);
  }
  ros2::MiddlewareRegistry::register_middleware("xrce_dds", this);
  ESP_LOGCONFIG(TAG, "XRCE-DDS middleware registered (agent=%s:%u, domain=%u)",
                this->agent_address_.c_str(), this->agent_port_, this->domain_id_);
}

void XrceDdsComponent::loop() {
  if (!this->session_init_)
    return;
  const uint32_t now = App.get_loop_component_start_time();
  if (this->link_ == LinkState::LINK_DOWN) {
    if (now >= this->next_attempt_)
      this->try_connect_();
    return;
  }
  if (now - this->last_pump_ < this->process_interval_ms_)
    return;
  this->last_pump_ = now;
  // KNOWN GAP: run health only catches transport errors. A silently dead
  // UDP agent (blackhole, no ICMP) looks healthy until traffic fails.
  // HIL follow-up: periodic time-sync ping when now - last_rx_ is large.
  if (!this->pump_once())
    this->drop_link_();
}

void XrceDdsComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "XRCE-DDS middleware:");
  ESP_LOGCONFIG(TAG, "  Agent: %s:%u, domain: %u, client: %s", this->agent_address_.c_str(),
                this->agent_port_, this->domain_id_, this->client_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Transport: %s",
                this->transport_ == TransportType::TRANSPORT_UDP ? "udp" : "serial");
  ESP_LOGCONFIG(TAG, "  Readers: %u/%u, writers: %u/%u", (unsigned) this->num_readers_,
                (unsigned) this->max_datareaders_, (unsigned) this->num_writers_,
                (unsigned) this->max_datawriters_);
  ESP_LOGCONFIG(TAG, "  Topics: %u/%u", (unsigned) this->count_topics_(),
                (unsigned) this->max_topics_);
}

void XrceDdsComponent::drop_link_() {
  // Always runs: closes the socket (no-op when already closed) and paces
  // the next attempt, even when the link was already down (connect-time
  // failures funnel through here too).
  this->udp_.close();
  this->next_attempt_ = App.get_loop_component_start_time() + this->keepalive_timeout_ms_;
  if (this->link_ == LinkState::LINK_DOWN)
    return;
  this->link_ = LinkState::LINK_DOWN;
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
  const uint32_t now = App.get_loop_component_start_time();
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
  // (agent down), paced by keepalive_timeout so the main loop keeps moving.
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
  this->last_rx_ = now;
  this->last_pump_ = now;
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
           "<dds><%s><topic><kind>NO_KEY</kind><name>%s</name><dataType>%s</dataType></topic></%s>",
           kind, topic, type, kind);
}

bool topic_fits(const std::string &topic) {
  char tmp[XRCE_TOPIC_NAME_LEN];
  return dds_topic_name(topic.c_str(), tmp, sizeof(tmp));
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
    build_endpoint_xml(xml, sizeof(xml), "data_reader", dds_topic, dds_type_name(e.type));
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
    build_endpoint_xml(xml, sizeof(xml), "data_writer", dds_topic, dds_type_name(e.type));
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
      uxr_buffer_request_data(&this->session_, this->out_stream_, p.reader->reader_id,
                              this->in_stream_, &dc);
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

// --- Ros2Middleware ---------------------------------------------------------

bool XrceDdsComponent::subscribe(const std::string &topic, const ros2::TypeDef *type,
                                 ros2::SampleCallback cb, const ros2::MiddlewareOptions *opts) {
  (void) opts;
  if (type == nullptr)
    return false;
  if (!topic_fits(topic)) {
    ESP_LOGE(TAG, "Topic name too long: %s", topic.c_str());
    return false;
  }
  if (ReaderEntry *e = this->find_reader_(topic)) {
    e->cb = std::move(cb);
    e->type = type;
    return true;
  }
  if (!this->topic_allowed_(topic)) {
    return false;
  }
  if (this->num_readers_ >= XRCE_MAX_READERS || this->num_readers_ >= this->max_datareaders_) {
    ESP_LOGE(TAG, "Too many readers for %s", topic.c_str());
    return false;
  }
  ReaderEntry &e = this->readers_[this->num_readers_++];
  e.topic = topic;
  e.type = type;
  e.cb = std::move(cb);
  if (this->link_ == LinkState::LINK_UP)
    this->create_reader_(e);
  return true;
}

bool XrceDdsComponent::publish(const std::string &topic, const ros2::TypeDef *type,
                               const void *sample, size_t len,
                               const ros2::MiddlewareOptions *opts) {
  (void) opts;
  if (type == nullptr || sample == nullptr)
    return false;
  if (!topic_fits(topic)) {
    ESP_LOGE(TAG, "Topic name too long: %s", topic.c_str());
    return false;
  }
  if (this->link_ != LinkState::LINK_UP)
    return false;
  WriterEntry *w = this->find_writer_(topic);
  if (w == nullptr) {
    if (!this->topic_allowed_(topic)) {
      return false;
    }
    if (this->num_writers_ >= XRCE_MAX_WRITERS || this->num_writers_ >= this->max_datawriters_) {
      ESP_LOGE(TAG, "Too many writers for %s", topic.c_str());
      return false;
    }
    WriterEntry &e = this->writers_[this->num_writers_++];
    e.topic = topic;
    e.type = type;
    w = &e;
  }
  if (!w->created && !this->create_writer_(*w))
    return false;
  uint32_t size = this->codec_.size_of(type, sample, len);
  if (size == 0) {
    ESP_LOGW(TAG, "Unencodable sample for %s", topic.c_str());
    return false;
  }
  ucdrBuffer ub;
  if (uxr_prepare_output_stream(&this->session_, this->out_stream_, w->writer_id, &ub, size) ==
      UXR_INVALID_REQUEST_ID) {
    return false;
  }
  if (!this->codec_.serialize(&ub, type, sample, len) || ub.error) {
    ESP_LOGW(TAG, "XCDR encode failed for %s", topic.c_str());
    return false;
  }
  return true;
}

bool XrceDdsComponent::publish_image(const std::string &topic, const uint8_t *jpeg, size_t len) {
  if (jpeg == nullptr || len == 0)
    return false;
  if (!topic_fits(topic)) {
    ESP_LOGE(TAG, "Topic name too long: %s", topic.c_str());
    return false;
  }
  if (this->link_ != LinkState::LINK_UP)
    return false;
  const ros2::TypeDef *type = ros2::find_type("sensor_msgs/CompressedImage");
  if (type == nullptr)
    return false;
  WriterEntry *w = this->find_writer_(topic);
  if (w == nullptr) {
    if (!this->topic_allowed_(topic)) {
      return false;
    }
    if (this->num_writers_ >= XRCE_MAX_WRITERS || this->num_writers_ >= this->max_datawriters_) {
      ESP_LOGE(TAG, "Too many writers for %s", topic.c_str());
      return false;
    }
    WriterEntry &e = this->writers_[this->num_writers_++];
    e.topic = topic;
    e.type = type;
    w = &e;
  }
  if (!w->created && !this->create_writer_(*w))
    return false;
  // CompressedImage members: header (zero stamp, empty frame_id — the
  // bridge has no clock sync yet), format "jpeg", then the length-prefixed
  // raw bytes. The ub window is one MTU; ucdr drives image_flush per full
  // window, which pumps the session without blocking, so frames of any size
  // stream in one call and abort fast on congestion (history exhaustion
  // -> ub.error).
  uint32_t total = 8 + (4 + 4 + 1) + (4 + 4 + 5) + 4 + (uint32_t) len;
  ucdrBuffer ub;
  if (uxr_prepare_output_stream_fragmented(&this->session_, this->img_stream_, w->writer_id, &ub,
                                           total, image_flush, this) == UXR_INVALID_REQUEST_ID) {
    return false;
  }
  if (!ucdr_serialize_int32_t(&ub, 0) || !ucdr_serialize_uint32_t(&ub, 0) ||
      !ucdr_serialize_string(&ub, "") || !ucdr_serialize_string(&ub, "jpeg") ||
      !ucdr_serialize_uint32_t(&ub, (uint32_t) len) ||
      !ucdr_serialize_array_uint8_t(&ub, jpeg, len) || ub.error) {
    ESP_LOGW(TAG, "Image publish failed for %s (%u bytes)", topic.c_str(), (unsigned) len);
    return false;
  }
  return true;
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
  this->last_rx_ = App.get_loop_component_start_time();
  if (this->codec_.deserialize(ub, e->type, this->sample_buf_, sizeof(this->sample_buf_)))
    e->cb(this->sample_buf_, e->type->size);
}

}  // namespace xrce_dds
}  // namespace esphome

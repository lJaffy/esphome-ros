#include "ros2_mqtt.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esphome/components/mqtt/mqtt_client.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#if __has_include("esp_heap_caps.h")
#include "esp_heap_caps.h"
#define ROS2_MQTT_HAVE_HEAP_CAPS 1
#endif

namespace esphome {
namespace ros2_mqtt {

static const char *const TAG = "ros2_mqtt";

// std::string lives in internal heap, which fragments under WiFi/camera
// load. Build image JSON in PSRAM instead; the MQTT backend copies it into
// its own (PSRAM-preferred) queue, so oversize frames fail gracefully in
// publish rather than aborting the loop task.
static const char K_B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

float Ros2MqttComponent::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }

void Ros2MqttComponent::setup() {
  ros2::MiddlewareRegistry::register_middleware("mqtt", this);
  ESP_LOGCONFIG(TAG, "MQTT middleware registered (prefix='%s')", this->prefix_.c_str());
}

void Ros2MqttComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "ROS 2 MQTT middleware:");
  ESP_LOGCONFIG(TAG, "  Topic prefix: '%s'", this->prefix_.c_str());
  ESP_LOGCONFIG(TAG, "  Default QoS: %u, retain: %s", this->default_qos_,
                this->default_retain_ ? "true" : "false");
}

std::string Ros2MqttComponent::expand_prefix_(const std::string &topic) const {
  if (this->prefix_.empty())
    return topic;
  if (!topic.empty() && topic[0] == '/')
    return this->prefix_ + topic;
  return this->prefix_ + "/" + topic;
}

uint8_t Ros2MqttComponent::effective_qos_(const ros2::MiddlewareOptions *opts) const {
  if (opts != nullptr && opts->qos_explicit)
    return opts->reliable ? 1 : 0;
  return this->default_qos_;
}

bool Ros2MqttComponent::subscribe(const std::string &topic, const ros2::TypeDef *type, ros2::SampleCallback cb,
                                  const ros2::MiddlewareOptions *opts) {
  if (type == nullptr)
    return false;
  if (mqtt::global_mqtt_client == nullptr) {
    ESP_LOGE(TAG, "MQTT client not available; check mqtt: is configured");
    return false;
  }
  std::string full_topic = this->expand_prefix_(topic);
  mqtt::global_mqtt_client->subscribe(
      full_topic,
      [this, type, cb](const std::string &t, const std::string &payload) {
        (void) t;
        if (this->codec_.deserialize(type, payload, this->sample_buf_, sizeof(this->sample_buf_)))
          cb(this->sample_buf_, type->size);
      },
      this->effective_qos_(opts));
  return true;
}

bool Ros2MqttComponent::publish(const std::string &topic, const ros2::TypeDef *type, const void *sample, size_t len,
                                const ros2::MiddlewareOptions *opts) {
  if (type == nullptr || sample == nullptr)
    return false;
  std::string payload = this->codec_.serialize(type, sample, len);
  return this->CustomMQTTDevice::publish(this->expand_prefix_(topic), payload, this->effective_qos_(opts),
                                         this->default_retain_);
}

bool Ros2MqttComponent::publish_image(const std::string &topic, const uint8_t *jpeg, size_t len,
                                      const ros2::MiddlewareOptions *opts) {
  if (jpeg == nullptr || len == 0)
    return false;
  if (mqtt::global_mqtt_client == nullptr)
    return false;
  // Raw mode (use_b64: false): publish JPEG bytes straight as the MQTT
  // payload. No base64 CPU, ~25% less bandwidth, no envelope alloc here (the
  // backend copies into its own PSRAM-preferred queue). The subscriber is
  // responsible for wrapping into sensor_msgs/CompressedImage (stamp/frame
  // live on the ROS side, e.g. receive time); never retained.
  if (opts != nullptr && !opts->use_b64) {
    return mqtt::global_mqtt_client->publish(this->expand_prefix_(topic).c_str(),
                                             reinterpret_cast<const char *>(jpeg), len,
                                             this->effective_qos_(opts), false);
  }
  // No std::string payload: large image JSON does not fit internal heap once
  // fragmented. Build one PSRAM buffer, base64 straight into it.
  int32_t sec = 0;
  uint32_t nsec = 0;
  const char *frame_id = "";
  if (opts != nullptr) {
    sec = opts->stamp_sec;
    nsec = opts->stamp_nsec;
    if (opts->frame_id != nullptr)
      frame_id = opts->frame_id;
  }
  char stamp[224];
  snprintf(stamp, sizeof(stamp), "{\"header\":{\"stamp\":{\"sec\":%d,\"nanosec\":%u},\"frame_id\":\"%s\"},"
                                "\"format\":\"jpeg\",\"data\":\"",
           sec, nsec, frame_id);
  static const char kSuffix[] = "\"}";
  const size_t prefix_len = strlen(stamp);
  const size_t b64_len = ((len + 2) / 3) * 4;
  const size_t need = prefix_len + b64_len + (sizeof(kSuffix) - 1);
#ifdef ROS2_MQTT_HAVE_HEAP_CAPS
  char *buf = static_cast<char *>(heap_caps_malloc(need + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (buf == nullptr)
    buf = static_cast<char *>(heap_caps_malloc(need + 1, MALLOC_CAP_DEFAULT));
#else
  char *buf = static_cast<char *>(malloc(need + 1));
#endif
  if (buf == nullptr) {
    ESP_LOGW(TAG, "Out of memory for ~%u byte image JSON; dropping", (unsigned) need);
    return false;
  }
  memcpy(buf, stamp, prefix_len);
  char *out = buf + prefix_len;
  size_t o = 0;
  size_t i = 0;
  while (i + 3 <= len) {
    uint32_t triple =
        (static_cast<uint32_t>(jpeg[i]) << 16) | (static_cast<uint32_t>(jpeg[i + 1]) << 8) | jpeg[i + 2];
    out[o++] = K_B64[(triple >> 18) & 0x3F];
    out[o++] = K_B64[(triple >> 12) & 0x3F];
    out[o++] = K_B64[(triple >> 6) & 0x3F];
    out[o++] = K_B64[triple & 0x3F];
    i += 3;
  }
  const size_t rem = len - i;
  if (rem == 1) {
    uint32_t triple = static_cast<uint32_t>(jpeg[i]) << 16;
    out[o++] = K_B64[(triple >> 18) & 0x3F];
    out[o++] = K_B64[(triple >> 12) & 0x3F];
    out[o++] = '=';
    out[o++] = '=';
  } else if (rem == 2) {
    uint32_t triple = (static_cast<uint32_t>(jpeg[i]) << 16) | (static_cast<uint32_t>(jpeg[i + 1]) << 8);
    out[o++] = K_B64[(triple >> 18) & 0x3F];
    out[o++] = K_B64[(triple >> 12) & 0x3F];
    out[o++] = K_B64[(triple >> 6) & 0x3F];
    out[o++] = '=';
  }
  memcpy(out + o, kSuffix, sizeof(kSuffix) - 1);
  buf[need] = '\0';
  // Never retained: a stale frame must not replay to late subscribers.
  // Raw publish: backend copies into its (PSRAM-preferred) queue, so buf can
  // be freed on return.
  bool ok = mqtt::global_mqtt_client->publish(this->expand_prefix_(topic).c_str(), buf, need,
                                              this->effective_qos_(opts), false);
#ifdef ROS2_MQTT_HAVE_HEAP_CAPS
  heap_caps_free(buf);
#else
  free(buf);
#endif
  return ok;
}

bool Ros2MqttComponent::call_service(const std::string &service, const ros2::ServiceDef *type,
                                      const void *req, size_t len, uint32_t timeout_ms,
                                      ros2::ServiceReplyCallback cb) {
  (void) service;
  (void) type;
  (void) req;
  (void) len;
  (void) timeout_ms;
  (void) cb;
  ESP_LOGE(TAG, "Services are DDS-only; service client refused on MQTT");
  return false;
}

bool Ros2MqttComponent::cancel_service(const std::string &service) {
  (void) service;
  ESP_LOGE(TAG, "Services are DDS-only; service client refused on MQTT");
  return false;
}

bool Ros2MqttComponent::connected() const {
  return const_cast<Ros2MqttComponent *>(this)->is_connected();
}

}  // namespace ros2_mqtt
}  // namespace esphome

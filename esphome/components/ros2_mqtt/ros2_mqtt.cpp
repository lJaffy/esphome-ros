#include "ros2_mqtt.h"

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

// Single-payload budget: UXGA/q14 JPEGs land ~80-150 kB (-> ~110-200 kB
// base64). Anything larger risks a >250 kB contiguous std::string, which the
// S3 loop heap cannot reliably provide alongside WiFi/MQTT/camera buffers.
static constexpr size_t ROS2_MQTT_MAX_JPEG_BYTES = 192 * 1024;
// Keep headroom for the MQTT client's own copy plus WiFi/TLS churn.
static constexpr size_t ROS2_MQTT_HEAP_MARGIN_BYTES = 60 * 1024;

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
  // Zero-copy base64-in-JSON: the old path built a ~200 kB b64 std::string
  // via per-char push_back (realloc storm) then copied it into a second
  // ~200 kB payload -> ~400 kB transient + the MQTT client's own copy, which
  // aborted the S3 loop task in __cxa_allocate_exception. Encode straight
  // into one reserved payload instead. Base64 output needs no escaping (no
  // '"' or '\\' in the alphabet); frame_id is restricted to [A-Za-z0-9/_-]
  // at validation so it needs none either.
  if (len > ROS2_MQTT_MAX_JPEG_BYTES) {
    ESP_LOGW(TAG, "Image %u bytes exceeds %u byte MQTT budget; dropping", (unsigned) len,
             (unsigned) ROS2_MQTT_MAX_JPEG_BYTES);
    return false;
  }
  int32_t sec = 0;
  uint32_t nsec = 0;
  const char *frame_id = "";
  if (opts != nullptr) {
    sec = opts->stamp_sec;
    nsec = opts->stamp_nsec;
    if (opts->frame_id != nullptr)
      frame_id = opts->frame_id;
  }
  std::string prefix = "{\"header\":{\"stamp\":{\"sec\":";
  prefix += std::to_string(sec);
  prefix += ",\"nanosec\":";
  prefix += std::to_string(nsec);
  prefix += "},\"frame_id\":\"";
  prefix += frame_id;
  prefix += "\"},\"format\":\"jpeg\",\"data\":\"";
  static const char kSuffix[] = "\"}";
  const size_t b64_len = ((len + 2) / 3) * 4;
  const size_t need = prefix.size() + b64_len + sizeof(kSuffix);
#ifdef ROS2_MQTT_HAVE_HEAP_CAPS
  const size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  if (free_heap < need + ROS2_MQTT_HEAP_MARGIN_BYTES) {
    ESP_LOGW(TAG, "Low heap (%u free, need ~%u); dropping %u byte image", (unsigned) free_heap,
             (unsigned) need, (unsigned) len);
    return false;
  }
#endif
  std::string payload;
  payload.reserve(need);
  payload += prefix;
  const size_t out_start = payload.size();
  payload.resize(out_start + b64_len);
  char *out = &payload[out_start];
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
  payload += kSuffix;
  // Never retained: a stale frame must not replay to late subscribers.
  return this->CustomMQTTDevice::publish(this->expand_prefix_(topic), payload,
                                         this->effective_qos_(opts), false);
}

bool Ros2MqttComponent::connected() const {
  return const_cast<Ros2MqttComponent *>(this)->is_connected();
}

}  // namespace ros2_mqtt
}  // namespace esphome

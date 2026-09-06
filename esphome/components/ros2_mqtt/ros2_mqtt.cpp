#include "ros2_mqtt.h"

#include "esphome/components/mqtt/mqtt_client.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ros2_mqtt {

static const char *const TAG = "ros2_mqtt";

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
  // Manual concat, not ArduinoJson: a UXGA frame base64s to ~200 kB and a
  // JSON doc that size would need a 400 kB+ arena. Base64 output needs no
  // escaping (no '"' or '\\' in the alphabet); frame_id is restricted to
  // [A-Za-z0-9/_-] at validation so it needs none either.
  int32_t sec = 0;
  uint32_t nsec = 0;
  const char *frame_id = "";
  if (opts != nullptr) {
    sec = opts->stamp_sec;
    nsec = opts->stamp_nsec;
    if (opts->frame_id != nullptr)
      frame_id = opts->frame_id;
  }
  std::string b64 = base64_encode(jpeg, len);
  std::string payload;
  payload.reserve(b64.size() + 128);
  payload += "{\"header\":{\"stamp\":{\"sec\":";
  payload += std::to_string(sec);
  payload += ",\"nanosec\":";
  payload += std::to_string(nsec);
  payload += "},\"frame_id\":\"";
  payload += frame_id;
  payload += "\"},\"format\":\"jpeg\",\"data\":\"";
  payload += b64;
  payload += "\"}";
  // Never retained: a stale frame must not replay to late subscribers.
  return this->CustomMQTTDevice::publish(this->expand_prefix_(topic), payload,
                                         this->effective_qos_(opts), false);
}

bool Ros2MqttComponent::connected() const {
  return const_cast<Ros2MqttComponent *>(this)->is_connected();
}

}  // namespace ros2_mqtt
}  // namespace esphome

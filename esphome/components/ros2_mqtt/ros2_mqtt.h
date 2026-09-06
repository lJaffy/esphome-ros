#pragma once

#include <string>

#include "esphome/core/component.h"
#include "esphome/components/mqtt/custom_mqtt_device.h"

#include "../ros2/ros2_json.h"
#include "../ros2/ros2_middleware.h"
#include "../ros2/ros2_types.h"

namespace esphome {
namespace ros2_mqtt {

class Ros2MqttComponent : public Component, public mqtt::CustomMQTTDevice, public ros2::Ros2Middleware {
 public:
  void set_topic_prefix(const std::string &prefix) { this->prefix_ = prefix; }
  void set_default_qos(uint8_t qos) { this->default_qos_ = qos; }
  void set_default_retain(bool retain) { this->default_retain_ = retain; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  bool subscribe(const std::string &topic, const ros2::TypeDef *type, ros2::SampleCallback cb,
                 const ros2::MiddlewareOptions *opts = nullptr) override;
  bool publish(const std::string &topic, const ros2::TypeDef *type, const void *sample, size_t len,
               const ros2::MiddlewareOptions *opts = nullptr) override;
  bool publish_image(const std::string &topic, const uint8_t *jpeg, size_t len) override;
  bool connected() const override;
  const char *name() const override { return "mqtt"; }

 protected:
  std::string expand_prefix_(const std::string &topic) const;

  std::string prefix_;
  uint8_t default_qos_{0};
  bool default_retain_{false};
  ros2::JsonCodec codec_;
  // Single-threaded main loop: one shared scratch buffer, no per-message heap.
  uint8_t sample_buf_[sizeof(ros2::JointTrajectoryMsg)]{0};
};

}  // namespace ros2_mqtt
}  // namespace esphome

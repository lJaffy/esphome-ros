#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "esphome/core/component.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/servo/servo.h"
#include "esphome/components/switch/switch.h"

#include "ros2_middleware.h"
#include "ros2_types.h"

namespace esphome {
namespace ros2 {

constexpr size_t ROS2_MAX_SUBSCRIPTIONS = 16;
constexpr size_t ROS2_MAX_PUBLICATIONS = 16;
constexpr size_t ROS2_MAX_TARGETS = 16;

enum class SubKind : uint8_t {
  SERVO_SINGLE,
  SWITCH_SINGLE,
  JOINT_MULTI,
};

enum class PubKind : uint8_t {
  SENSOR_SINGLE,
  SWITCH_SINGLE,
  BINARY_SENSOR_SINGLE,
  JOINT_MULTI,
};

struct JointTarget {
  char joint_name[ROS2_NAME_LEN]{0};
  servo::Servo *servo{nullptr};
  float min_rad{-3.14159265f};
  float max_rad{3.14159265f};
};

struct Subscription {
  std::string topic;
  const TypeDef *type{nullptr};
  SubKind kind{SubKind::SWITCH_SINGLE};
  switch_::Switch *sw{nullptr};
  servo::Servo *servo{nullptr};
  float min_rad{-3.14159265f};
  float max_rad{3.14159265f};
  std::array<JointTarget, ROS2_MAX_TARGETS> joints{};
  size_t num_joints{0};
};

struct JointSource {
  char joint_name[ROS2_NAME_LEN]{0};
  servo::Servo *servo{nullptr};
  float min_rad{-3.14159265f};
  float max_rad{3.14159265f};
};

struct Publication {
  std::string topic;
  const TypeDef *type{nullptr};
  PubKind kind{PubKind::SENSOR_SINGLE};
  sensor::Sensor *sensor{nullptr};
  switch_::Switch *sw{nullptr};
  binary_sensor::BinarySensor *bsensor{nullptr};
  std::array<JointSource, ROS2_MAX_TARGETS> joints{};
  size_t num_joints{0};
  uint32_t interval_ms{1000};
  uint32_t last_pub{0};
};

class Ros2Component : public Component {
 public:
  void set_middleware_name(const std::string &name) { this->middleware_name_ = name; }
  void set_default_publish_interval(uint32_t ms) { this->default_interval_ms_ = ms; }
  void set_status_sensor(binary_sensor::BinarySensor *s) { this->status_sensor_ = s; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void add_servo_subscription(const char *topic, const char *type, servo::Servo *servo, float min_rad,
                              float max_rad);
  void add_joint_subscription(const char *topic, const char *type, servo::Servo *servo, const char *joint_name,
                              float min_rad, float max_rad);
  void add_joint_state_source(servo::Servo *servo, const char *joint_name, float min_rad, float max_rad);
  void add_switch_subscription(const char *topic, const char *type, switch_::Switch *sw);
  uint8_t add_switch_publication(const char *topic, const char *type, switch_::Switch *sw, uint32_t interval_ms);
  uint8_t add_sensor_publication(const char *topic, const char *type, sensor::Sensor *sensor, uint32_t interval_ms);
  uint8_t add_binary_sensor_publication(const char *topic, const char *type, binary_sensor::BinarySensor *bs,
                                        uint32_t interval_ms);
  uint8_t add_joint_state_publication(const char *topic, const char *type, uint32_t interval_ms);

 protected:
  void try_subscribe_();
  void dispatch_scalar_switch_(const Subscription &sub, const void *sample);
  void dispatch_scalar_servo_(const Subscription &sub, const void *sample);
  void dispatch_joints_(const Subscription &sub, const void *sample);
  static float rad_to_level_(float rad, float min_rad, float max_rad);
  static float level_to_rad_(float level, float min_rad, float max_rad);
  void remember_level_(servo::Servo *servo, float level);
  float recalled_level_(servo::Servo *servo);
  void poll_publication_(Publication &pub);

  std::string middleware_name_{"mqtt"};
  Ros2Middleware *mw_{nullptr};
  bool subscribed_{false};
  uint32_t mw_retry_at_{0};
  std::array<Subscription, ROS2_MAX_SUBSCRIPTIONS> subs_{};
  size_t num_subs_{0};
  std::array<Publication, ROS2_MAX_PUBLICATIONS> pubs_{};
  size_t num_pubs_{0};
  uint32_t default_interval_ms_{1000};
  binary_sensor::BinarySensor *status_sensor_{nullptr};
  struct ServoLevel {
    servo::Servo *servo{nullptr};
    float level{0.0f};
  };
  std::array<ServoLevel, ROS2_MAX_TARGETS * ROS2_MAX_SUBSCRIPTIONS> levels_{};
  size_t num_levels_{0};
};

}  // namespace ros2
}  // namespace esphome

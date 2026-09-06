#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "esphome/core/component.h"
#ifdef __has_include
#if __has_include("esphome/core/defines.h")
#include "esphome/core/defines.h"
#endif
#endif

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/servo/servo.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/time/real_time_clock.h"

#include "ros2_middleware.h"
#include "ros2_types.h"

namespace esphome {
namespace ros2 {

// Static-only bounds: each Subscription carries a full joint table, so keep
// these small (ESP32 SRAM). Raise only with a measured RAM budget.
constexpr size_t ROS2_MAX_SUBSCRIPTIONS = 16;
constexpr size_t ROS2_MAX_PUBLICATIONS = 16;
constexpr size_t ROS2_MAX_TARGETS = 16;

enum class SubKind : uint8_t {
  SERVO_SINGLE,
  SWITCH_SINGLE,
  JOINT_MULTI,
  LIGHT_SINGLE,
};

enum class PubKind : uint8_t {
  SENSOR_SINGLE,
  SWITCH_SINGLE,
  BINARY_SENSOR_SINGLE,
  JOINT_MULTI,
  IMAGE_SINGLE,
  LIGHT_SINGLE,
};

enum class LightField : uint8_t {
  RGB,
  BRIGHTNESS,
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
  light::LightState *light{nullptr};
  LightField light_field{LightField::RGB};
  float min_rad{-3.14159265f};
  float max_rad{3.14159265f};
  std::array<JointTarget, ROS2_MAX_TARGETS> joints{};
  size_t num_joints{0};
  bool reliable{true};
  bool qos_explicit{false};
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
  camera::Camera *camera{nullptr};
  light::LightState *light{nullptr};
  LightField light_field{LightField::RGB};
  std::array<JointSource, ROS2_MAX_TARGETS> joints{};
  size_t num_joints{0};
  uint32_t interval_ms{1000};
  uint32_t last_pub{0};
  char frame_id[ROS2_FRAME_ID_LEN]{0};
  bool reliable{true};
  bool qos_explicit{false};
  // sensor_msgs/Range static geometry (range itself comes from the sensor).
  uint8_t radiation_type{0};
  float field_of_view{0.0f};
  float min_range{0.0f};
  float max_range{0.0f};
  float range_variance{0.0f};
  // sensor_msgs/BatteryState: percentage maps voltage across [min_v, max_v].
  float min_voltage{3.0f};
  float max_voltage{4.2f};
  float design_capacity{0.0f};
  uint8_t battery_technology{0};
  char battery_location[ROS2_NAME_LEN]{0};
};

class Ros2Component : public Component, public camera::CameraListener {
 public:
  void set_middleware_name(const std::string &name) { this->middleware_name_ = name; }
  void set_default_publish_interval(uint32_t ms) { this->default_interval_ms_ = ms; }
  void set_status_sensor(binary_sensor::BinarySensor *s) { this->status_sensor_ = s; }
  void set_time(time::RealTimeClock *t) { this->time_ = t; }

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
  void add_light_subscription(const char *topic, const char *type, light::LightState *light, const char *field);
  uint8_t add_light_publication(const char *topic, const char *type, light::LightState *light, uint32_t interval_ms);
  uint8_t add_switch_publication(const char *topic, const char *type, switch_::Switch *sw, uint32_t interval_ms);
  uint8_t add_sensor_publication(const char *topic, const char *type, sensor::Sensor *sensor, uint32_t interval_ms);
  uint8_t add_range_publication(const char *topic, sensor::Sensor *sensor, uint32_t interval_ms);
  uint8_t add_battery_publication(const char *topic, sensor::Sensor *sensor, uint32_t interval_ms);
  uint8_t add_binary_sensor_publication(const char *topic, const char *type, binary_sensor::BinarySensor *bs,
                                        uint32_t interval_ms);
  uint8_t add_joint_state_publication(const char *topic, const char *type, uint32_t interval_ms);
  uint8_t add_image_publication(const char *topic, const char *type, camera::Camera *camera, uint32_t interval_ms);
  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;
  // Post-hoc per-topic configuration from codegen (keeps add_* signatures
  // stable across single- and multi-entity topics).
  void set_subscription_qos(const char *topic, const char *qos);
  void set_publication_qos(const char *topic, const char *qos);
  void set_publication_frame_id(const char *topic, const char *frame_id);
  void set_range_params(const char *topic, uint8_t radiation_type, float field_of_view, float min_range,
                        float max_range, float variance);
  void set_battery_params(const char *topic, float min_voltage, float max_voltage, float design_capacity,
                          uint8_t technology, const char *location);

 protected:
  void try_subscribe_();
  void register_camera_listener_();
  void publish_compressed_image_(Publication &pub, const uint8_t *jpeg, size_t len);
  // Stamp from the time source (zeros when unset/unsynced) + frame_id copy.
  void fill_header_(HeaderMsg &header, const char *frame_id);
  static void apply_qos_(bool &reliable, bool &explicit_out, const char *qos);
  void dispatch_scalar_switch_(const Subscription &sub, const void *sample);
  void dispatch_scalar_servo_(const Subscription &sub, const void *sample);
  void dispatch_joints_(const Subscription &sub, const void *sample);
  void dispatch_light_(const Subscription &sub, const void *sample);
  static float clamp01_(float v);
  static LightField parse_light_field_(const char *field);
  static float rad_to_level_(float rad, float min_rad, float max_rad);
  static float level_to_rad_(float level, float min_rad, float max_rad);
  void remember_level_(servo::Servo *servo, float level);
  float recalled_level_(servo::Servo *servo);
  void poll_publication_(Publication &pub);

  std::string middleware_name_{"mqtt"};
  Ros2Middleware *mw_{nullptr};
  time::RealTimeClock *time_{nullptr};
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

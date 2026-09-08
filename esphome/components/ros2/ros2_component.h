#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esphome/core/component.h"
#ifdef __has_include
#if __has_include("esphome/core/defines.h")
#include "esphome/core/defines.h"
#endif
#endif

// All entity dependencies are optional: AUTO_LOAD pulls only the libs the
// YAML actually uses, so any of these headers may be absent from the build.
// Guard every include with its USE_* macro and forward-declare the classes
// below (pointer members/parameters never need the complete type).
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_CAMERA
#include "esphome/components/camera/camera.h"
#endif
#ifdef USE_LIGHT
#include "esphome/components/light/light_state.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_SERVO
#include "esphome/components/servo/servo.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

#include "ros2_middleware.h"
#include "ros2_types.h"
namespace esphome
{
// Forward declarations matching the guarded includes above. Only pointers
// to these appear in this header, so incomplete types are enough; the .cpp
// includes the full header under the same USE_* guard wherever it calls
// methods on them.
namespace binary_sensor
{
class BinarySensor;
}  // namespace binary_sensor
namespace camera
{
class Camera;
}  // namespace camera
namespace light
{
class LightState;
}  // namespace light
namespace sensor
{
class Sensor;
}  // namespace sensor
namespace servo
{
class Servo;
}  // namespace servo
namespace switch_
{
class Switch;
}  // namespace switch_
namespace time
{
class RealTimeClock;
}  // namespace time
}  // namespace esphome

namespace esphome
{
namespace ros2
{
// Static-only bounds: each Subscription carries a full joint table, so keep
// these small (ESP32 SRAM). Raise only with a measured RAM budget.
constexpr size_t ROS2_MAX_SUBSCRIPTIONS = 16;
constexpr size_t ROS2_MAX_PUBLICATIONS = 16;
constexpr size_t ROS2_MAX_TARGETS = 16;
constexpr size_t ROS2_MAX_SERVICES = 4;

enum class SubKind : uint8_t {
  SERVO_SINGLE,
  SWITCH_SINGLE,
  JOINT_MULTI,
  LIGHT_SINGLE,
  DIFF_DRIVE,
};

enum class PubKind : uint8_t {
  SENSOR_SINGLE,
  SWITCH_SINGLE,
  BINARY_SENSOR_SINGLE,
  JOINT_MULTI,
  IMAGE_SINGLE,
  LIGHT_SINGLE,
  ODOM,
  TF,
  IMU,
  NAVSAT,
};

enum class LightField : uint8_t {
  RGB,
  BRIGHTNESS,
};

// Inbound dispatch queue (Phase 2): subscription callbacks may fire on a
// middleware worker thread, so they only memcpy the decoded sample here.
// loop() drains the queue and runs the dispatch_* bodies on the loop thread
// where entity writes are safe. Depth 8, drop-oldest + counter.
constexpr size_t ROS2_INBOUND_DEPTH = 8;
constexpr size_t ROS2_INBOUND_SAMPLE_MAX = sizeof(JointTrajectoryMsg);

struct InboundItem {
  uint8_t sub_idx{0};
  uint16_t len{0};
  uint8_t data[ROS2_INBOUND_SAMPLE_MAX]{0};
};

struct ServiceClient {
  std::string service;
  const ServiceDef *type{nullptr};
  switch_::Switch *trigger{nullptr};
  bool last_state{false};
  bool pending{false};
  uint32_t timeout_ms{5000};
};

static_assert(sizeof(JointStateMsg) <= ROS2_INBOUND_SAMPLE_MAX, "inbound payload too small");
static_assert(sizeof(TFMessageMsg) <= ROS2_INBOUND_SAMPLE_MAX, "inbound payload too small");

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
  // Diff-drive (planar) parameters; wheels are velocity-driven servos.
  servo::Servo *left_wheel{nullptr};
  servo::Servo *right_wheel{nullptr};
  float wheel_separation{0.2f};
  float max_linear_speed{0.5f};
  float max_angular_speed{2.0f};
  uint32_t cmd_timeout_ms{500};
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
  // nav_msgs/Odometry dead-reckoning state (open-loop, commanded velocity).
  char child_frame_id[ROS2_FRAME_ID_LEN]{0};
  float odom_wheel_separation{0.2f};
  std::string tf_topic;
  float odom_x{0.0f};
  float odom_y{0.0f};
  float odom_theta{0.0f};
  uint32_t odom_last_ms{0};
  // tf2_msgs/TFMessage static transforms.
  TFTransformMsg tf_transforms[ROS2_MAX_TF_TRANSFORMS]{};
  uint8_t num_tf_transforms{0};
  // sensor_msgs/Imu multi-sensor source: accel + gyro (m/s^2, rad/s),
  // optional orientation quaternion. Null orientation entries mean
  // unbound (publish covariances[0] = -1 per the IDL).
  sensor::Sensor *imu_accel[3]{nullptr};
  sensor::Sensor *imu_gyro[3]{nullptr};
  sensor::Sensor *imu_orientation[4]{nullptr};
  bool imu_has_orientation{false};
  // sensor_msgs/NavSatFix source: latitude + longitude (degrees, required),
  // altitude (m, optional; NaN on the wire when unbound). Status is fixed
  // at STATUS_FIX while lat/lon have state; otherwise the poll is skipped.
  sensor::Sensor *navsat_lat{nullptr};
  sensor::Sensor *navsat_lon{nullptr};
  sensor::Sensor *navsat_alt{nullptr};
  // Decoupled sampling cache: sensor callbacks (raw = pre-filter, filt =
  // post-filter) copy the latest value + tick here so poll_publication_ can
  // publish at the ROS interval while Home Assistant keeps its own
  // update_interval / throttle cadence. Static-only, loop-thread only.
  bool use_raw{false};
  // Image encoding for IMAGE_SINGLE: true (default) base64-in-JSON,
  // false raw JPEG bytes on the MQTT payload.
  bool use_b64{true};
  float raw_state{0.0f};
  bool has_raw{false};
  uint32_t raw_ms{0};
  float filt_state{0.0f};
  bool has_filt{false};
  uint32_t filt_ms{0};
  uint32_t last_sent_sample_ms{0};
  bool has_sent{false};
  uint32_t stale_skips{0};
  // IMU caches: 0-2 accel, 3-5 gyro, 6-9 orientation.
  float imu_raw[10]{0.0f};
  bool imu_has_raw[10]{false};
  uint32_t imu_raw_ms[10]{0};
  float imu_filt[10]{0.0f};
  bool imu_has_filt[10]{false};
  uint32_t imu_filt_ms[10]{0};
  // NavSat caches: 0 lat, 1 lon, 2 alt.
  float navsat_raw_v[3]{0.0f};
  bool navsat_has_raw[3]{false};
  uint32_t navsat_raw_ms[3]{0};
  float navsat_filt_v[3]{0.0f};
  bool navsat_has_filt[3]{false};
  uint32_t navsat_filt_ms[3]{0};
};

#ifdef USE_CAMERA
class Ros2Component : public Component, public camera::CameraListener {
#else
class Ros2Component : public Component {
#endif
 public:
  void set_middleware_name(const std::string &name) { this->middleware_name_ = name; }
  void set_default_publish_interval(uint32_t ms) { this->default_interval_ms_ = ms; }
  void set_status_sensor(binary_sensor::BinarySensor *s) { this->status_sensor_ = s; }
  void set_time(time::RealTimeClock *t) { this->time_ = t; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void add_service_client(const char *service, const char *type, switch_::Switch *trigger,
                            uint32_t timeout_ms);
  void add_servo_subscription(const char *topic, const char *type, servo::Servo *servo, float min_rad,
                              float max_rad);
  void add_joint_subscription(const char *topic, const char *type, servo::Servo *servo, const char *joint_name,
                              float min_rad, float max_rad);
  void add_joint_state_source(servo::Servo *servo, const char *joint_name, float min_rad, float max_rad);
  void add_switch_subscription(const char *topic, const char *type, switch_::Switch *sw);
  void add_light_subscription(const char *topic, const char *type, light::LightState *light, const char *field);
  void add_diff_drive_subscription(const char *topic, servo::Servo *left, servo::Servo *right,
                                   float wheel_separation, float max_linear_speed,
                                   float max_angular_speed, uint32_t cmd_timeout_ms);
  uint8_t add_light_publication(const char *topic, const char *type, light::LightState *light, uint32_t interval_ms);
  uint8_t add_switch_publication(const char *topic, const char *type, switch_::Switch *sw, uint32_t interval_ms);
  uint8_t add_sensor_publication(const char *topic, const char *type, sensor::Sensor *sensor, uint32_t interval_ms);
  uint8_t add_range_publication(const char *topic, sensor::Sensor *sensor, uint32_t interval_ms);
  uint8_t add_battery_publication(const char *topic, sensor::Sensor *sensor, uint32_t interval_ms);
  uint8_t add_binary_sensor_publication(const char *topic, const char *type, binary_sensor::BinarySensor *bs,
                                        uint32_t interval_ms);
  uint8_t add_joint_state_publication(const char *topic, const char *type, uint32_t interval_ms);
  uint8_t add_image_publication(const char *topic, const char *type, camera::Camera *camera, uint32_t interval_ms);
  uint8_t add_odom_publication(const char *topic, uint32_t interval_ms);
  uint8_t add_tf_publication(const char *topic, uint32_t interval_ms);
  uint8_t add_imu_publication(const char *topic, uint32_t interval_ms);
  uint8_t add_navsat_publication(const char *topic, uint32_t interval_ms);
  void set_navsat_sources(const char *topic, sensor::Sensor *lat, sensor::Sensor *lon);
  void set_navsat_altitude(const char *topic, sensor::Sensor *alt);
  void set_imu_sources(const char *topic, sensor::Sensor *ax, sensor::Sensor *ay, sensor::Sensor *az,
                       sensor::Sensor *gx, sensor::Sensor *gy, sensor::Sensor *gz);
  void set_imu_orientation(const char *topic, sensor::Sensor *ox, sensor::Sensor *oy, sensor::Sensor *oz,
                           sensor::Sensor *ow);
  void add_tf_transform(const char *topic, const char *frame_id, const char *child_frame_id,
                        float tx, float ty, float tz, float qx, float qy, float qz, float qw);
#ifdef USE_CAMERA
  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;
#endif
  // Post-hoc per-topic configuration from codegen (keeps add_* signatures
  // stable across single- and multi-entity topics).
  void set_subscription_qos(const char *topic, const char *qos);
  void set_publication_qos(const char *topic, const char *qos);
  void set_publication_raw(const char *topic, bool raw);
  void set_publication_use_b64(const char *topic, bool use_b64);
  void set_publication_frame_id(const char *topic, const char *frame_id);
  void set_range_params(const char *topic, uint8_t radiation_type, float field_of_view, float min_range,
                        float max_range, float variance);
  void set_battery_params(const char *topic, float min_voltage, float max_voltage, float design_capacity,
                          uint8_t technology, const char *location);
  void set_odom_params(const char *topic, float wheel_separation, const char *child_frame_id,
                       const char *tf_topic);

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
  void dispatch_diff_drive_(const Subscription &sub, const void *sample);
  static float clamp01_(float v);
  static LightField parse_light_field_(const char *field);
  static float rad_to_level_(float rad, float min_rad, float max_rad);
  static float level_to_rad_(float level, float min_rad, float max_rad);
  void remember_level_(servo::Servo *servo, float level);
  float recalled_level_(servo::Servo *servo);
  void poll_publication_(Publication &pub);
  void register_single_sensor_cache_(size_t idx);
  void register_imu_cache_(size_t idx);
  void register_navsat_cache_(size_t idx);
  bool single_sample_(Publication &pub, float &out, uint32_t &sample_ms);
  bool imu_sample_(Publication &pub, float *accel, float *gyro, float *orient, bool &have_orientation,
                   uint32_t &sample_ms);
  bool navsat_sample_(Publication &pub, float &lat, float &lon, float &alt, bool &have_alt,
                      uint32_t &sample_ms);
  void poll_odom_(Publication &pub, const MiddlewareOptions &opts, uint32_t now);
  void poll_tf_(Publication &pub, const MiddlewareOptions &opts);
  void poll_imu_(Publication &pub, const MiddlewareOptions &opts, uint32_t now);
  void poll_navsat_(Publication &pub, const MiddlewareOptions &opts, uint32_t now);
  void publish_tf_transform_(const std::string &topic, const MiddlewareOptions &opts, int32_t sec,
                             const char *frame_id, const char *child_frame_id, float x, float y,
                             float qz, float qw);
  // Zero stale wheel commands (cmd_vel timeout). Runs on loop(), not hot path.
  void stop_stale_diff_drive_(uint32_t now);
  // Inbound queue edge: safe to call from any thread (middleware callbacks).
  void enqueue_inbound_(size_t sub_idx, const void *sample, size_t len);
  // Loop thread only: replay queued samples through the dispatch_* bodies.
  void drain_inbound_();
  void dispatch_sub_(size_t sub_idx, const void *sample);

  std::string middleware_name_{"mqtt"};
  Ros2Middleware *mw_{nullptr};
  time::RealTimeClock *time_{nullptr};
  // Last commanded planar velocity (single diff-drive base per MCU).
  float cmd_vl_{0.0f};
  float cmd_vr_{0.0f};
  uint32_t cmd_time_{0};
  uint32_t cmd_timeout_ms_{500};
  bool cmd_active_{false};
  bool subscribed_{false};
  uint32_t mw_retry_at_{0};
  std::array<Subscription, ROS2_MAX_SUBSCRIPTIONS> subs_{};
  size_t num_subs_{0};
  std::array<Publication, ROS2_MAX_PUBLICATIONS> pubs_{};
  size_t num_pubs_{0};
  std::array<ServiceClient, ROS2_MAX_SERVICES> svcs_{};
  size_t num_svcs_{0};
  uint32_t svc_ok_{0};
  uint32_t svc_fail_{0};
  uint32_t default_interval_ms_{1000};
  binary_sensor::BinarySensor *status_sensor_{nullptr};
  struct ServoLevel {
    servo::Servo *servo{nullptr};
    float level{0.0f};
  };
  std::array<ServoLevel, ROS2_MAX_TARGETS * ROS2_MAX_SUBSCRIPTIONS> levels_{};
  size_t num_levels_{0};
  // Inbound dispatch queue (see InboundItem): created in setup(), drained
  // in loop(). Callbacks only enqueue; dispatch runs on the loop thread.
  QueueHandle_t inbound_{nullptr};
  StaticQueue_t inbound_ctrl_{};
  uint8_t inbound_storage_[ROS2_INBOUND_DEPTH * sizeof(InboundItem)]{};
  std::atomic<uint32_t> inbound_drop_{0};
  // Main-loop cadence (Phase 2 exit signal): worst gap between loop()
  // passes + passes slower than 100 ms. Loop-only.
  uint32_t loop_last_ms_{0};
  uint32_t loop_max_gap_ms_{0};
  uint32_t loop_slow_passes_{0};
};

}  // namespace ros2
}  // namespace esphome

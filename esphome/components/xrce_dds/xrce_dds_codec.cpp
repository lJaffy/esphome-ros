#include "xrce_dds_codec.h"

#include <cstring>

#include "../ros2/ros2_types.h"

namespace esphome {
namespace xrce_dds {

namespace {

uint32_t str_size(uint32_t size, const char *s) {
  size += (uint32_t) (ucdr_alignment(size, 4) + 4 + strlen(s) + 1);
  return size;
}

bool ser_header(ucdrBuffer *ub, const ros2::HeaderMsg *h) {
  return ucdr_serialize_int32_t(ub, h->stamp_sec) && ucdr_serialize_uint32_t(ub, h->stamp_nsec) &&
         ucdr_serialize_string(ub, h->frame_id);
}

bool de_header(ucdrBuffer *ub, ros2::HeaderMsg *h) {
  return ucdr_deserialize_int32_t(ub, &h->stamp_sec) &&
         ucdr_deserialize_uint32_t(ub, &h->stamp_nsec) &&
         ucdr_deserialize_string(ub, h->frame_id, sizeof(h->frame_id));
}

uint32_t header_size(uint32_t size, const ros2::HeaderMsg *h) {
  size += (uint32_t) (ucdr_alignment(size, 4) + 4);  // stamp_sec
  size += 4;                                        // stamp_nanosec (already 4-aligned)
  return str_size(size, h->frame_id);
}

// float64[] on the wire, float[] in the struct.
bool ser_f64_seq(ucdrBuffer *ub, const float *vals, uint8_t n) {
  if (!ucdr_serialize_uint32_t(ub, n))
    return false;
  for (uint8_t i = 0; i < n; i++)
    if (!ucdr_serialize_double(ub, (double) vals[i]))
      return false;
  return true;
}

bool de_f64_seq(ucdrBuffer *ub, float *vals, uint8_t capacity, uint8_t *n) {
  uint32_t len = 0;
  // Bound the wire length before reading elements.
  ucdrBuffer probe = *ub;
  if (!ucdr_deserialize_uint32_t(&probe, &len) || len > capacity)
    return false;
  *ub = probe;
  for (uint32_t i = 0; i < len; i++) {
    double d = 0;
    if (!ucdr_deserialize_double(ub, &d))
      return false;
    vals[i] = (float) d;
  }
  *n = (uint8_t) len;
  return true;
}

uint32_t f64_seq_size(uint32_t size, uint8_t n) {
  size += (uint32_t) (ucdr_alignment(size, 4) + 4);  // length
  size += (uint32_t) (ucdr_alignment(size, 8) + (uint32_t) n * 8);
  return size;
}

bool ser_names(ucdrBuffer *ub, const char (*names)[ros2::ROS2_NAME_LEN], uint8_t n) {
  if (!ucdr_serialize_uint32_t(ub, n))
    return false;
  for (uint8_t i = 0; i < n; i++)
    if (!ucdr_serialize_string(ub, names[i]))
      return false;
  return true;
}

bool de_names(ucdrBuffer *ub, char (*names)[ros2::ROS2_NAME_LEN], uint8_t capacity, uint8_t *n) {
  uint32_t len = 0;
  ucdrBuffer probe = *ub;
  if (!ucdr_deserialize_uint32_t(&probe, &len) || len > capacity)
    return false;
  *ub = probe;
  for (uint32_t i = 0; i < len; i++)
    if (!ucdr_deserialize_string(ub, names[i], ros2::ROS2_NAME_LEN))
      return false;
  *n = (uint8_t) len;
  return true;
}

uint32_t names_size(uint32_t size, const char (*names)[ros2::ROS2_NAME_LEN], uint8_t n) {
  size += (uint32_t) (ucdr_alignment(size, 4) + 4);
  for (uint8_t i = 0; i < n; i++)
    size = str_size(size, names[i]);
  return size;
}

// Fixed float64 vector on the wire, floats in the struct.
bool ser_f64_vec(ucdrBuffer *ub, const float *vals, uint8_t n) {
  for (uint8_t i = 0; i < n; i++)
    if (!ucdr_serialize_double(ub, (double) vals[i]))
      return false;
  return true;
}

bool de_f64_vec(ucdrBuffer *ub, float *vals, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    double d = 0;
    if (!ucdr_deserialize_double(ub, &d))
      return false;
    vals[i] = (float) d;
  }
  return true;
}

uint32_t f64_vec_size(uint32_t size, uint8_t n) {
  for (uint8_t i = 0; i < n; i++)
    size += (uint32_t) (ucdr_alignment(size, 8) + 8);
  return size;
}

bool ser_cov36(ucdrBuffer *ub) {
  for (uint8_t i = 0; i < 36; i++)
    if (!ucdr_serialize_double(ub, 0.0))
      return false;
  return true;
}

bool de_cov36(ucdrBuffer *ub) {
  double d = 0;
  for (uint8_t i = 0; i < 36; i++)
    if (!ucdr_deserialize_double(ub, &d))
      return false;
  return true;
}

uint32_t cov36_size(uint32_t size) {
  for (uint8_t i = 0; i < 36; i++)
    size += (uint32_t) (ucdr_alignment(size, 8) + 8);
  return size;
}

}  // namespace

bool dds_topic_name(const char *ros_topic, char *out, size_t cap) {
  if (ros_topic == nullptr || out == nullptr || cap == 0)
    return false;
  size_t n = strlen(ros_topic);
  if (n + 3 > cap)
    return false;
  out[0] = 'r';
  out[1] = 't';
  memcpy(out + 2, ros_topic, n + 1);
  return true;
}

namespace {
const char *dds_type_suffix(const char *ros_name) {
  if (strcmp(ros_name, "std_msgs/Bool") == 0)
    return "std_msgs::msg::dds_::Bool_";
  if (strcmp(ros_name, "std_msgs/Float32") == 0)
    return "std_msgs::msg::dds_::Float32_";
  if (strcmp(ros_name, "std_msgs/Int32") == 0)
    return "std_msgs::msg::dds_::Int32_";
  if (strcmp(ros_name, "std_msgs/String") == 0)
    return "std_msgs::msg::dds_::String_";
  if (strcmp(ros_name, "sensor_msgs/JointState") == 0)
    return "sensor_msgs::msg::dds_::JointState_";
  if (strcmp(ros_name, "trajectory_msgs/JointTrajectory") == 0)
    return "trajectory_msgs::msg::dds_::JointTrajectory_";
  if (strcmp(ros_name, "std_msgs/ColorRGBA") == 0)
    return "std_msgs::msg::dds_::ColorRGBA_";
  if (strcmp(ros_name, "sensor_msgs/Joy") == 0)
    return "sensor_msgs::msg::dds_::Joy_";
  if (strcmp(ros_name, "sensor_msgs/Range") == 0)
    return "sensor_msgs::msg::dds_::Range_";
  if (strcmp(ros_name, "sensor_msgs/BatteryState") == 0)
    return "sensor_msgs::msg::dds_::BatteryState_";
  if (strcmp(ros_name, "geometry_msgs/Twist") == 0)
    return "geometry_msgs::msg::dds_::Twist_";
  if (strcmp(ros_name, "nav_msgs/Odometry") == 0)
    return "nav_msgs::msg::dds_::Odometry_";
  if (strcmp(ros_name, "tf2_msgs/TFMessage") == 0)
    return "tf2_msgs::msg::dds_::TFMessage_";
  if (strcmp(ros_name, "sensor_msgs/Imu") == 0)
    return "sensor_msgs::msg::dds_::Imu_";
  if (strcmp(ros_name, "sensor_msgs/NavSatFix") == 0)
    return "sensor_msgs::msg::dds_::NavSatFix_";
  if (strcmp(ros_name, "sensor_msgs/CompressedImage") == 0)
    return "sensor_msgs::msg::dds_::CompressedImage_";
  return "";
}
}  // namespace

const char *dds_type_name(const ros2::TypeDef *type) {
  if (type == nullptr)
    return "";
  return dds_type_suffix(type->name);
}

bool dds_service_request_names(const char *ros_service, char *req_topic, size_t req_cap, char *req_type,
                               size_t req_type_cap, char *rep_topic, size_t rep_cap, char *rep_type,
                               size_t rep_type_cap) {
  if (ros_service == nullptr || req_topic == nullptr || req_type == nullptr || rep_topic == nullptr ||
      rep_type == nullptr)
    return false;
  size_t n = strlen(ros_service);
  if (n == 0 || n + 12 > req_cap || n + 12 > rep_cap)
    return false;
  memcpy(req_topic, "rq", 2);
  memcpy(req_topic + 2, ros_service, n + 1);
  memcpy(rep_topic, "rr", 2);
  memcpy(rep_topic + 2, ros_service, n + 1);
  const char *req = "std_srvs::srv::dds_::Trigger_Request_";
  const char *rep = "std_srvs::srv::dds_::Trigger_Response_";
  if (strlen(req) + 1 > req_type_cap || strlen(rep) + 1 > rep_type_cap)
    return false;
  memcpy(req_type, req, strlen(req) + 1);
  memcpy(rep_type, rep, strlen(rep) + 1);
  return true;
}

uint32_t service_request_size(const ros2::ServiceDef *service) {
  if (service == nullptr || strcmp(service->name, "std_srvs/Trigger") != 0)
    return 0;
  return 0;
}

uint32_t service_reply_size(const ros2::ServiceDef *service, const void *reply, size_t len) {
  if (service == nullptr || strcmp(service->name, "std_srvs/Trigger") != 0 || reply == nullptr)
    return 0;
  if (len < sizeof(ros2::TriggerResMsg))
    return 0;
  auto *msg = static_cast<const ros2::TriggerResMsg *>(reply);
  uint32_t size = 1;
  return str_size(size, msg->message);
}

uint32_t XcdrCodec::size_of(const ros2::TypeDef *type, const void *sample, size_t len) {
  if (type == nullptr || sample == nullptr)
    return 0;
  uint32_t size = 0;
  if (strcmp(type->name, "std_msgs/Bool") == 0 && len >= sizeof(ros2::BoolMsg)) {
    return 1;
  }
  if (strcmp(type->name, "std_msgs/Float32") == 0 && len >= sizeof(ros2::Float32Msg)) {
    return 4;
  }
  if (strcmp(type->name, "std_msgs/Int32") == 0 && len >= sizeof(ros2::Int32Msg)) {
    return 4;
  }
  if (strcmp(type->name, "std_msgs/String") == 0 && len >= sizeof(ros2::StringMsg)) {
    auto *msg = static_cast<const ros2::StringMsg *>(sample);
    return str_size(0, msg->data);
  }
  if (strcmp(type->name, "std_msgs/ColorRGBA") == 0 && len >= sizeof(ros2::ColorRGBAMsg)) {
    return 16;
  }
  if (strcmp(type->name, "sensor_msgs/JointState") == 0 && len >= sizeof(ros2::JointStateMsg)) {
    auto *msg = static_cast<const ros2::JointStateMsg *>(sample);
    uint8_t n = msg->num_joints > ros2::ROS2_MAX_JOINTS ? ros2::ROS2_MAX_JOINTS : msg->num_joints;
    // Must match serialize() byte-for-byte: velocity/effort go out as
    // zero-length (length field only), so they contribute just align4+4.
    size = header_size(0, &msg->header);
    size = names_size(size, msg->name, n);
    size = f64_seq_size(size, n);
    size += (uint32_t) (ucdr_alignment(size, 4) + 4);  // velocity: empty
    size += (uint32_t) (ucdr_alignment(size, 4) + 4);  // effort: empty
    return size;
  }
  if (strcmp(type->name, "trajectory_msgs/JointTrajectory") == 0 &&
      len >= sizeof(ros2::JointTrajectoryMsg)) {
    auto *msg = static_cast<const ros2::JointTrajectoryMsg *>(sample);
    uint8_t nj = msg->num_joints > ros2::ROS2_MAX_JOINTS ? ros2::ROS2_MAX_JOINTS : msg->num_joints;
    (void) msg->num_points;  // serialize() always emits a single setpoint
    size = header_size(0, &msg->header);
    size = names_size(size, msg->joint_names, nj);
    // Must match serialize(): single point, positions full, the other
    // three arrays empty, then duration.
    size += (uint32_t) (ucdr_alignment(size, 4) + 4);  // points sequence length (1)
    size = f64_seq_size(size, nj);                     // positions
    size += (uint32_t) (ucdr_alignment(size, 4) + 4);  // velocities: empty
    size += (uint32_t) (ucdr_alignment(size, 4) + 4);  // accelerations: empty
    size += (uint32_t) (ucdr_alignment(size, 4) + 4);  // effort: empty
    size += (uint32_t) (ucdr_alignment(size, 4) + 8);  // time_from_start sec+nsec
    return size;
  }
  if (strcmp(type->name, "sensor_msgs/Joy") == 0 && len >= sizeof(ros2::JoyMsg)) {
    auto *msg = static_cast<const ros2::JoyMsg *>(sample);
    size = header_size(0, &msg->header);
    size += (uint32_t) (ucdr_alignment(size, 4) + 4 + (uint32_t) msg->num_axes * 4);
    size += (uint32_t) (ucdr_alignment(size, 4) + 4 + (uint32_t) msg->num_buttons * 4);
    return size;
  }
  if (strcmp(type->name, "sensor_msgs/Range") == 0 && len >= sizeof(ros2::RangeMsg)) {
    auto *msg = static_cast<const ros2::RangeMsg *>(sample);
    size = header_size(0, &msg->header);
    size += 1;  // radiation_type
    // field_of_view, min_range, max_range, range, variance.
    size += (uint32_t) (ucdr_alignment(size, 4) + 4 * 5);
    return size;
  }
  if (strcmp(type->name, "sensor_msgs/BatteryState") == 0 && len >= sizeof(ros2::BatteryStateMsg)) {
    auto *msg = static_cast<const ros2::BatteryStateMsg *>(sample);
    size = header_size(0, &msg->header);
    // voltage, temperature, current, charge, capacity, design_capacity,
    // percentage.
    size += (uint32_t) (ucdr_alignment(size, 4) + 7 * 4);
    size += 4;  // power_supply_status, health, technology, present
    size += (uint32_t) (ucdr_alignment(size, 4) + 4);  // cell_voltage: empty
    size += (uint32_t) (ucdr_alignment(size, 4) + 4);  // cell_temperature: empty
    size = str_size(size, msg->location);
    size = str_size(size, "");  // serial_number: empty
    return size;
  }
  if (strcmp(type->name, "geometry_msgs/Twist") == 0 && len >= sizeof(ros2::TwistMsg)) {
    // linear xyz + angular xyz, all float64 on the wire.
    return f64_vec_size(0, 6);
  }
  if (strcmp(type->name, "nav_msgs/Odometry") == 0 && len >= sizeof(ros2::OdometryMsg)) {
    auto *msg = static_cast<const ros2::OdometryMsg *>(sample);
    size = header_size(0, &msg->header);
    size = str_size(size, msg->child_frame_id);
    size = f64_vec_size(size, 3);  // pose position
    size = f64_vec_size(size, 4);  // pose orientation
    size = cov36_size(size);       // pose covariance: zeros
    size = f64_vec_size(size, 3);  // twist linear
    size = f64_vec_size(size, 3);  // twist angular
    size = cov36_size(size);       // twist covariance: zeros
    return size;
  }
  if (strcmp(type->name, "tf2_msgs/TFMessage") == 0 && len >= sizeof(ros2::TFMessageMsg)) {
    auto *msg = static_cast<const ros2::TFMessageMsg *>(sample);
    uint8_t n = msg->num_transforms > ros2::ROS2_MAX_TF_TRANSFORMS ? ros2::ROS2_MAX_TF_TRANSFORMS
                                                                   : msg->num_transforms;
    size = (uint32_t) (ucdr_alignment(size, 4) + 4);  // transforms sequence length
    for (uint8_t i = 0; i < n; i++) {
      size = header_size(size, &msg->transforms[i].header);
      size = str_size(size, msg->transforms[i].child_frame_id);
      size = f64_vec_size(size, 3);  // translation
      size = f64_vec_size(size, 4);  // rotation
    }
    return size;
  }
  if (strcmp(type->name, "sensor_msgs/Imu") == 0 && len >= sizeof(ros2::ImuMsg)) {
    size = header_size(0, &static_cast<const ros2::ImuMsg *>(sample)->header);
    size = f64_vec_size(size, 4);  // orientation
    size = f64_vec_size(size, 9);  // orientation_covariance
    size = f64_vec_size(size, 3);  // angular_velocity
    size = f64_vec_size(size, 9);  // angular_velocity_covariance
    size = f64_vec_size(size, 3);  // linear_acceleration
    size = f64_vec_size(size, 9);  // linear_acceleration_covariance
    return size;
  }
  if (strcmp(type->name, "sensor_msgs/NavSatFix") == 0 && len >= sizeof(ros2::NavSatFixMsg)) {
    size = header_size(0, &static_cast<const ros2::NavSatFixMsg *>(sample)->header);
    size += 1;  // status (int8)
    size += (uint32_t) (ucdr_alignment(size, 2) + 2);  // service (uint16)
    size = f64_vec_size(size, 3);  // latitude, longitude, altitude
    size = f64_vec_size(size, 9);  // position_covariance
    size += 1;  // position_covariance_type (uint8)
    return size;
  }
  return 0;
}

bool XcdrCodec::serialize(ucdrBuffer *ub, const ros2::TypeDef *type, const void *sample,
                          size_t len) {
  if (ub == nullptr || type == nullptr || sample == nullptr)
    return false;
  if (strcmp(type->name, "std_msgs/Bool") == 0 && len >= sizeof(ros2::BoolMsg)) {
    return ucdr_serialize_bool(ub, static_cast<const ros2::BoolMsg *>(sample)->data);
  }
  if (strcmp(type->name, "std_msgs/Float32") == 0 && len >= sizeof(ros2::Float32Msg)) {
    return ucdr_serialize_float(ub, static_cast<const ros2::Float32Msg *>(sample)->data);
  }
  if (strcmp(type->name, "std_msgs/Int32") == 0 && len >= sizeof(ros2::Int32Msg)) {
    return ucdr_serialize_int32_t(ub, static_cast<const ros2::Int32Msg *>(sample)->data);
  }
  if (strcmp(type->name, "std_msgs/String") == 0 && len >= sizeof(ros2::StringMsg)) {
    return ucdr_serialize_string(ub, static_cast<const ros2::StringMsg *>(sample)->data);
  }
  if (strcmp(type->name, "std_msgs/ColorRGBA") == 0 && len >= sizeof(ros2::ColorRGBAMsg)) {
    auto *msg = static_cast<const ros2::ColorRGBAMsg *>(sample);
    return ucdr_serialize_float(ub, msg->r) && ucdr_serialize_float(ub, msg->g) &&
           ucdr_serialize_float(ub, msg->b) && ucdr_serialize_float(ub, msg->a);
  }
  if (strcmp(type->name, "sensor_msgs/JointState") == 0 && len >= sizeof(ros2::JointStateMsg)) {
    auto *msg = static_cast<const ros2::JointStateMsg *>(sample);
    uint8_t n = msg->num_joints > ros2::ROS2_MAX_JOINTS ? ros2::ROS2_MAX_JOINTS : msg->num_joints;
    if (!ser_header(ub, &msg->header))
      return false;
    if (!ser_names(ub, msg->name, n))
      return false;
    // Velocity/effort publish as zero-length: subscribers treat missing
    // arrays as unknown, and it halves the on-wire size. Position is the
    // only array the bridge itself ever reads.
    if (!ser_f64_seq(ub, msg->position, n))
      return false;
    if (!ser_f64_seq(ub, msg->velocity, 0))
      return false;
    return ser_f64_seq(ub, msg->effort, 0);
  }
  if (strcmp(type->name, "trajectory_msgs/JointTrajectory") == 0 &&
      len >= sizeof(ros2::JointTrajectoryMsg)) {
    auto *msg = static_cast<const ros2::JointTrajectoryMsg *>(sample);
    uint8_t nj = msg->num_joints > ros2::ROS2_MAX_JOINTS ? ros2::ROS2_MAX_JOINTS : msg->num_joints;
    if (!ser_header(ub, &msg->header))
      return false;
    if (!ser_names(ub, msg->joint_names, nj))
      return false;
    // The bridge only publishes single-setpoint trajectories (point[0]).
    if (!ucdr_serialize_uint32_t(ub, 1))
      return false;
    return ser_f64_seq(ub, msg->points[0].positions, nj) &&
           ser_f64_seq(ub, msg->points[0].velocities, 0) &&
           ser_f64_seq(ub, msg->points[0].accelerations, 0) &&
           ser_f64_seq(ub, msg->points[0].effort, 0) &&
           ucdr_serialize_uint32_t(ub, msg->points[0].time_sec) &&
           ucdr_serialize_uint32_t(ub, msg->points[0].time_nsec);
  }
  if (strcmp(type->name, "sensor_msgs/Joy") == 0 && len >= sizeof(ros2::JoyMsg)) {
    // Subscribe-only on this bridge; no publish path serializes Joy.
    (void) ub;
    return false;
  }
  if (strcmp(type->name, "sensor_msgs/Range") == 0 && len >= sizeof(ros2::RangeMsg)) {
    auto *msg = static_cast<const ros2::RangeMsg *>(sample);
    if (!ser_header(ub, &msg->header))
      return false;
    if (!ucdr_serialize_uint8_t(ub, msg->radiation_type))
      return false;
    return ucdr_serialize_float(ub, msg->field_of_view) &&
           ucdr_serialize_float(ub, msg->min_range) && ucdr_serialize_float(ub, msg->max_range) &&
           ucdr_serialize_float(ub, msg->range) && ucdr_serialize_float(ub, msg->variance);
  }
  if (strcmp(type->name, "sensor_msgs/BatteryState") == 0 &&
      len >= sizeof(ros2::BatteryStateMsg)) {
    auto *msg = static_cast<const ros2::BatteryStateMsg *>(sample);
    if (!ser_header(ub, &msg->header))
      return false;
    // NaN floats go out as NaN doubles, matching the IDL's unmeasured
    // convention; cell arrays and serial_number publish empty.
    if (!ucdr_serialize_float(ub, msg->voltage) || !ucdr_serialize_float(ub, msg->temperature) ||
        !ucdr_serialize_float(ub, msg->current) || !ucdr_serialize_float(ub, msg->charge) ||
        !ucdr_serialize_float(ub, msg->capacity) ||
        !ucdr_serialize_float(ub, msg->design_capacity) ||
        !ucdr_serialize_float(ub, msg->percentage))
      return false;
    if (!ucdr_serialize_uint8_t(ub, msg->power_supply_status) ||
        !ucdr_serialize_uint8_t(ub, msg->power_supply_health) ||
        !ucdr_serialize_uint8_t(ub, msg->power_supply_technology) ||
        !ucdr_serialize_bool(ub, msg->present))
      return false;
    if (!ucdr_serialize_uint32_t(ub, 0) || !ucdr_serialize_uint32_t(ub, 0))
      return false;
    return ucdr_serialize_string(ub, msg->location) && ucdr_serialize_string(ub, "");
  }
  if (strcmp(type->name, "geometry_msgs/Twist") == 0 && len >= sizeof(ros2::TwistMsg)) {
    // Subscribe-only on this bridge; no publish path serializes Twist.
    (void) ub;
    return false;
  }
  if (strcmp(type->name, "nav_msgs/Odometry") == 0 && len >= sizeof(ros2::OdometryMsg)) {
    auto *msg = static_cast<const ros2::OdometryMsg *>(sample);
    if (!ser_header(ub, &msg->header))
      return false;
    if (!ucdr_serialize_string(ub, msg->child_frame_id))
      return false;
    if (!ser_f64_vec(ub, msg->pose_position, 3) || !ser_f64_vec(ub, msg->pose_orientation, 4))
      return false;
    if (!ser_cov36(ub))
      return false;
    if (!ser_f64_vec(ub, msg->twist_linear, 3) || !ser_f64_vec(ub, msg->twist_angular, 3))
      return false;
    return ser_cov36(ub);
  }
  if (strcmp(type->name, "tf2_msgs/TFMessage") == 0 && len >= sizeof(ros2::TFMessageMsg)) {
    auto *msg = static_cast<const ros2::TFMessageMsg *>(sample);
    uint8_t n = msg->num_transforms > ros2::ROS2_MAX_TF_TRANSFORMS ? ros2::ROS2_MAX_TF_TRANSFORMS
                                                                   : msg->num_transforms;
    if (!ucdr_serialize_uint32_t(ub, n))
      return false;
    for (uint8_t i = 0; i < n; i++) {
      const ros2::TFTransformMsg *t = &msg->transforms[i];
      if (!ser_header(ub, &t->header))
        return false;
      if (!ucdr_serialize_string(ub, t->child_frame_id))
        return false;
      if (!ser_f64_vec(ub, t->translation, 3) || !ser_f64_vec(ub, t->rotation, 4))
        return false;
    }
    return true;
  }
  if (strcmp(type->name, "sensor_msgs/Imu") == 0 && len >= sizeof(ros2::ImuMsg)) {
    auto *msg = static_cast<const ros2::ImuMsg *>(sample);
    if (!ser_header(ub, &msg->header))
      return false;
    // NaN floats (unused axes read as absent) go out as NaN doubles;
    // orientation_covariance[0] = -1 marks "no orientation estimate".
    if (!ser_f64_vec(ub, msg->orientation, 4))
      return false;
    if (!ser_f64_vec(ub, msg->orientation_covariance, 9))
      return false;
    if (!ser_f64_vec(ub, msg->angular_velocity, 3))
      return false;
    if (!ser_f64_vec(ub, msg->angular_velocity_covariance, 9))
      return false;
    if (!ser_f64_vec(ub, msg->linear_acceleration, 3))
      return false;
    return ser_f64_vec(ub, msg->linear_acceleration_covariance, 9);
  }
  if (strcmp(type->name, "sensor_msgs/NavSatFix") == 0 && len >= sizeof(ros2::NavSatFixMsg)) {
    auto *msg = static_cast<const ros2::NavSatFixMsg *>(sample);
    if (!ser_header(ub, &msg->header))
      return false;
    // Unavailable altitude is NaN (no altimeter bound); NaN floats go out
    // as NaN doubles. Latitude/longitude/altitude are consecutive in the
    // struct, so one vector write covers all three.
    if (!ucdr_serialize_int8_t(ub, msg->status))
      return false;
    if (!ucdr_serialize_uint16_t(ub, msg->service))
      return false;
    if (!ser_f64_vec(ub, &msg->latitude, 3))
      return false;
    if (!ser_f64_vec(ub, msg->position_covariance, 9))
      return false;
    return ucdr_serialize_uint8_t(ub, msg->position_covariance_type);
  }
  return false;
}

uint32_t XcdrCodec::request_size(const ros2::ServiceDef *service) {
  return service_request_size(service);
}

bool XcdrCodec::serialize_request(ucdrBuffer *ub, const ros2::ServiceDef *service, const void *req,
                                  size_t len) {
  if (ub == nullptr || service == nullptr)
    return false;
  if (strcmp(service->name, "std_srvs/Trigger") != 0)
    return false;
  (void) req;
  (void) len;
  return true;
}

bool XcdrCodec::deserialize_reply(ucdrBuffer *ub, const ros2::ServiceDef *service, void *out,
                                  size_t out_len) {
  if (ub == nullptr || service == nullptr || out == nullptr)
    return false;
  if (strcmp(service->name, "std_srvs/Trigger") != 0 || out_len < sizeof(ros2::TriggerResMsg))
    return false;
  auto *msg = static_cast<ros2::TriggerResMsg *>(out);
  memset(msg, 0, sizeof(*msg));
  return ucdr_deserialize_bool(ub, &msg->success) &&
         ucdr_deserialize_string(ub, msg->message, sizeof(msg->message));
}

uint32_t XcdrCodec::reply_size(const ros2::ServiceDef *service, const void *reply, size_t len) {
  return service_reply_size(service, reply, len);
}

bool XcdrCodec::deserialize(ucdrBuffer *ub, const ros2::TypeDef *type, void *out, size_t out_len) {
  if (ub == nullptr || type == nullptr || out == nullptr)
    return false;
  if (strcmp(type->name, "std_msgs/Bool") == 0 && out_len >= sizeof(ros2::BoolMsg)) {
    return ucdr_deserialize_bool(ub, &static_cast<ros2::BoolMsg *>(out)->data);
  }
  if (strcmp(type->name, "std_msgs/Float32") == 0 && out_len >= sizeof(ros2::Float32Msg)) {
    return ucdr_deserialize_float(ub, &static_cast<ros2::Float32Msg *>(out)->data);
  }
  if (strcmp(type->name, "std_msgs/Int32") == 0 && out_len >= sizeof(ros2::Int32Msg)) {
    return ucdr_deserialize_int32_t(ub, &static_cast<ros2::Int32Msg *>(out)->data);
  }
  if (strcmp(type->name, "std_msgs/String") == 0 && out_len >= sizeof(ros2::StringMsg)) {
    auto *msg = static_cast<ros2::StringMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    return ucdr_deserialize_string(ub, msg->data, sizeof(msg->data));
  }
  if (strcmp(type->name, "std_msgs/ColorRGBA") == 0 && out_len >= sizeof(ros2::ColorRGBAMsg)) {
    auto *msg = static_cast<ros2::ColorRGBAMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    return ucdr_deserialize_float(ub, &msg->r) && ucdr_deserialize_float(ub, &msg->g) &&
           ucdr_deserialize_float(ub, &msg->b) && ucdr_deserialize_float(ub, &msg->a);
  }
  if (strcmp(type->name, "sensor_msgs/JointState") == 0 && out_len >= sizeof(ros2::JointStateMsg)) {
    auto *msg = static_cast<ros2::JointStateMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    uint8_t n = 0;
    if (!de_header(ub, &msg->header))
      return false;
    if (!de_names(ub, msg->name, ros2::ROS2_MAX_JOINTS, &n))
      return false;
    msg->num_joints = n;
    uint8_t m = 0;
    // Velocity/effort are optional on the wire; absent (zero-length) arrays
    // leave the struct fields zeroed from the memset above.
    if (!de_f64_seq(ub, msg->position, ros2::ROS2_MAX_JOINTS, &m) || m != n)
      return false;
    if (!de_f64_seq(ub, msg->velocity, ros2::ROS2_MAX_JOINTS, &m))
      return false;
    return de_f64_seq(ub, msg->effort, ros2::ROS2_MAX_JOINTS, &m);
  }
  if (strcmp(type->name, "trajectory_msgs/JointTrajectory") == 0 &&
      out_len >= sizeof(ros2::JointTrajectoryMsg)) {
    auto *msg = static_cast<ros2::JointTrajectoryMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    uint8_t nj = 0;
    if (!de_header(ub, &msg->header))
      return false;
    if (!de_names(ub, msg->joint_names, ros2::ROS2_MAX_JOINTS, &nj))
      return false;
    msg->num_joints = nj;
    uint32_t np = 0;
    if (!ucdr_deserialize_uint32_t(ub, &np) || np == 0)
      return false;
    // Single-setpoint model: only point[0] is kept; the dispatch layer
    // ignores the rest. Still, the wire cursor must consume point[0]
    // fully (all four arrays + duration) to stay aligned.
    msg->num_points = 1;
    uint8_t m = 0;
    if (!de_f64_seq(ub, msg->points[0].positions, ros2::ROS2_MAX_JOINTS, &m) || m != nj)
      return false;
    if (!de_f64_seq(ub, msg->points[0].velocities, ros2::ROS2_MAX_JOINTS, &m))
      return false;
    if (!de_f64_seq(ub, msg->points[0].accelerations, ros2::ROS2_MAX_JOINTS, &m))
      return false;
    if (!de_f64_seq(ub, msg->points[0].effort, ros2::ROS2_MAX_JOINTS, &m))
      return false;
    return ucdr_deserialize_uint32_t(ub, &msg->points[0].time_sec) &&
           ucdr_deserialize_uint32_t(ub, &msg->points[0].time_nsec);
  }
  if (strcmp(type->name, "sensor_msgs/Joy") == 0 && out_len >= sizeof(ros2::JoyMsg)) {
    auto *msg = static_cast<ros2::JoyMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    uint32_t na = 0, nb = 0;
    if (!de_header(ub, &msg->header))
      return false;
    if (!ucdr_deserialize_sequence_float(ub, msg->axes, ros2::ROS2_MAX_JOY_AXES, &na))
      return false;
    msg->num_axes = (uint8_t) na;
    if (!ucdr_deserialize_sequence_int32_t(ub, msg->buttons, ros2::ROS2_MAX_JOY_BUTTONS, &nb))
      return false;
    msg->num_buttons = (uint8_t) nb;
    return true;
  }
  if (strcmp(type->name, "sensor_msgs/Range") == 0 && out_len >= sizeof(ros2::RangeMsg)) {
    auto *msg = static_cast<ros2::RangeMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    if (!de_header(ub, &msg->header))
      return false;
    return ucdr_deserialize_uint8_t(ub, &msg->radiation_type) &&
           ucdr_deserialize_float(ub, &msg->field_of_view) &&
           ucdr_deserialize_float(ub, &msg->min_range) &&
           ucdr_deserialize_float(ub, &msg->max_range) && ucdr_deserialize_float(ub, &msg->range) &&
           ucdr_deserialize_float(ub, &msg->variance);
  }
  if (strcmp(type->name, "sensor_msgs/BatteryState") == 0 &&
      out_len >= sizeof(ros2::BatteryStateMsg)) {
    auto *msg = static_cast<ros2::BatteryStateMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    if (!de_header(ub, &msg->header))
      return false;
    if (!ucdr_deserialize_float(ub, &msg->voltage) ||
        !ucdr_deserialize_float(ub, &msg->temperature) ||
        !ucdr_deserialize_float(ub, &msg->current) || !ucdr_deserialize_float(ub, &msg->charge) ||
        !ucdr_deserialize_float(ub, &msg->capacity) ||
        !ucdr_deserialize_float(ub, &msg->design_capacity) ||
        !ucdr_deserialize_float(ub, &msg->percentage))
      return false;
    if (!ucdr_deserialize_uint8_t(ub, &msg->power_supply_status) ||
        !ucdr_deserialize_uint8_t(ub, &msg->power_supply_health) ||
        !ucdr_deserialize_uint8_t(ub, &msg->power_supply_technology) ||
        !ucdr_deserialize_bool(ub, &msg->present))
      return false;
    // The bridge keeps no cell storage, so inbound cell data is rejected
    // instead of overflowing the fixed struct.
    uint32_t n = 0;
    ucdrBuffer probe = *ub;
    if (!ucdr_deserialize_uint32_t(&probe, &n) || n != 0)
      return false;
    *ub = probe;
    probe = *ub;
    if (!ucdr_deserialize_uint32_t(&probe, &n) || n != 0)
      return false;
    *ub = probe;
    char serial[ros2::ROS2_NAME_LEN];
    return ucdr_deserialize_string(ub, msg->location, sizeof(msg->location)) &&
           ucdr_deserialize_string(ub, serial, sizeof(serial));
  }
  if (strcmp(type->name, "geometry_msgs/Twist") == 0 && out_len >= sizeof(ros2::TwistMsg)) {
    auto *msg = static_cast<ros2::TwistMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    if (!de_f64_vec(ub, &msg->linear_x, 3))
      return false;
    return de_f64_vec(ub, &msg->angular_x, 3);
  }
  if (strcmp(type->name, "nav_msgs/Odometry") == 0 && out_len >= sizeof(ros2::OdometryMsg)) {
    auto *msg = static_cast<ros2::OdometryMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    if (!de_header(ub, &msg->header))
      return false;
    if (!ucdr_deserialize_string(ub, msg->child_frame_id, sizeof(msg->child_frame_id)))
      return false;
    if (!de_f64_vec(ub, msg->pose_position, 3) || !de_f64_vec(ub, msg->pose_orientation, 4))
      return false;
    if (!de_cov36(ub))
      return false;
    if (!de_f64_vec(ub, msg->twist_linear, 3) || !de_f64_vec(ub, msg->twist_angular, 3))
      return false;
    return de_cov36(ub);
  }
  if (strcmp(type->name, "tf2_msgs/TFMessage") == 0 && out_len >= sizeof(ros2::TFMessageMsg)) {
    auto *msg = static_cast<ros2::TFMessageMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    uint32_t n = 0;
    // Bound the wire length before reading transforms.
    ucdrBuffer probe = *ub;
    if (!ucdr_deserialize_uint32_t(&probe, &n) || n > ros2::ROS2_MAX_TF_TRANSFORMS)
      return false;
    *ub = probe;
    msg->num_transforms = (uint8_t) n;
    for (uint32_t i = 0; i < n; i++) {
      ros2::TFTransformMsg *t = &msg->transforms[i];
      if (!de_header(ub, &t->header))
        return false;
      if (!ucdr_deserialize_string(ub, t->child_frame_id, sizeof(t->child_frame_id)))
        return false;
      if (!de_f64_vec(ub, t->translation, 3) || !de_f64_vec(ub, t->rotation, 4))
        return false;
    }
    return true;
  }
  if (strcmp(type->name, "sensor_msgs/Imu") == 0 && out_len >= sizeof(ros2::ImuMsg)) {
    auto *msg = static_cast<ros2::ImuMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    if (!de_header(ub, &msg->header))
      return false;
    if (!de_f64_vec(ub, msg->orientation, 4))
      return false;
    if (!de_f64_vec(ub, msg->orientation_covariance, 9))
      return false;
    if (!de_f64_vec(ub, msg->angular_velocity, 3))
      return false;
    if (!de_f64_vec(ub, msg->angular_velocity_covariance, 9))
      return false;
    if (!de_f64_vec(ub, msg->linear_acceleration, 3))
      return false;
    return de_f64_vec(ub, msg->linear_acceleration_covariance, 9);
  }
  if (strcmp(type->name, "sensor_msgs/NavSatFix") == 0 && out_len >= sizeof(ros2::NavSatFixMsg)) {
    auto *msg = static_cast<ros2::NavSatFixMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    if (!de_header(ub, &msg->header))
      return false;
    if (!ucdr_deserialize_int8_t(ub, &msg->status))
      return false;
    if (!ucdr_deserialize_uint16_t(ub, &msg->service))
      return false;
    if (!de_f64_vec(ub, &msg->latitude, 3))
      return false;
    if (!de_f64_vec(ub, msg->position_covariance, 9))
      return false;
    return ucdr_deserialize_uint8_t(ub, &msg->position_covariance_type);
  }
  return false;
}

}  // namespace xrce_dds
}  // namespace esphome

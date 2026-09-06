#include "xrce_dds_codec.h"

#include <cstring>

#include "../ros2/ros2_types.h"

namespace esphome {
namespace xrce_dds {

namespace {

uint32_t str_size(uint32_t size, const char *s) {
  return (uint32_t) (ucdr_alignment(size, 4) + 4 + strlen(s) + 1);
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
  return false;
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
  return false;
}

}  // namespace xrce_dds
}  // namespace esphome

#pragma once

#include <cstddef>
#include <cstdint>

#include "xrce_dds_vendor.h"

#include "../ros2/ros2_middleware.h"

namespace esphome {
namespace xrce_dds {

// Hand-written XCDR (CDR LE) encode/decode for the ros2 fixed structs,
// built on the vendored Micro-CDR primitives (alignment, endianness and
// bounds checks). No encapsulation header: XRCE-DDS carries bare CDR
// members, matching microxrceddsgen output for the same IDL.
//
// ROS IDL ground truth (third_party/common_interfaces):
// - float64 arrays (JointState position/velocity/effort, all
//   JointTrajectoryPoint arrays) are 8-byte doubles on the wire; the MCU
//   structs hold float, so the codec widens/narrows per element.
// - Everything else maps 1:1 (bool/char=1B, float/int32/uint32=4B,
//   string/sequence = uint32 length + bytes).
// - ucdr sequence/string reads fail when the wire length exceeds the
//   destination capacity, so oversize ROS strings drop the sample instead
//   of overflowing the fixed buffers.
class XcdrCodec {
 public:
  uint32_t size_of(const ros2::TypeDef *type, const void *sample, size_t len);
  bool serialize(ucdrBuffer *ub, const ros2::TypeDef *type, const void *sample, size_t len);
  bool deserialize(ucdrBuffer *ub, const ros2::TypeDef *type, void *out, size_t out_len);
  uint32_t request_size(const ros2::ServiceDef *service);
  bool serialize_request(ucdrBuffer *ub, const ros2::ServiceDef *service, const void *req, size_t len);
  bool deserialize_reply(ucdrBuffer *ub, const ros2::ServiceDef *service, void *out, size_t out_len);
  uint32_t reply_size(const ros2::ServiceDef *service, const void *reply, size_t len);
};

// ROS <-> DDS name mapping, matching rmw_microxrcedds (utils.c):
// - topic "/joint_states" -> "rt/joint_states" (ros_topic_prefix "rt")
// - type "sensor_msgs/JointState" -> "sensor_msgs::msg::dds_::JointState_"
//   ("{ns}::dds_::{name}_" with ns = "pkg::msg").
// All nine bridge types are msg types, so the pkg/Type split is enough.
// Mismatched names create valid DDS entities that ROS 2 never matches,
// so a failed `ros2 topic echo` after connect points here first.
bool dds_topic_name(const char *ros_topic, char *out, size_t cap);
const char *dds_type_name(const ros2::TypeDef *type);
bool dds_service_request_names(const char *ros_service, char *req_topic, size_t req_cap, char *req_type,
                               size_t req_type_cap, char *rep_topic, size_t rep_cap, char *rep_type,
                               size_t rep_type_cap);
uint32_t service_request_size(const ros2::ServiceDef *service);
uint32_t service_reply_size(const ros2::ServiceDef *service, const void *reply, size_t len);

}  // namespace xrce_dds
}  // namespace esphome

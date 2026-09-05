#include "ros2_json.h"

#include <cstring>

#include "esphome/components/json/json_util.h"
#include "ros2_types.h"

namespace esphome {
namespace ros2 {

static void copy_name(char *dst, const char *src) {
  strncpy(dst, src, ROS2_NAME_LEN - 1);
  dst[ROS2_NAME_LEN - 1] = '\0';
}

bool JsonCodec::deserialize(const TypeDef *type, const std::string &payload, void *out, size_t out_len) {
  if (type == nullptr || out == nullptr)
    return false;
  bool ok = false;
  if (strcmp(type->name, "std_msgs/Bool") == 0) {
    if (out_len < sizeof(BoolMsg))
      return false;
    auto *msg = static_cast<BoolMsg *>(out);
    if (payload == "true") {
      msg->data = true;
      return true;
    }
    if (payload == "false") {
      msg->data = false;
      return true;
    }
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      if (root["data"].is<bool>()) {
        msg->data = root["data"].as<bool>();
        ok = true;
      }
      return true;
    }) && ok;
  }
  if (strcmp(type->name, "std_msgs/Float32") == 0) {
    if (out_len < sizeof(Float32Msg))
      return false;
    auto *msg = static_cast<Float32Msg *>(out);
    char *end = nullptr;
    float v = strtof(payload.c_str(), &end);
    if (end != nullptr && *end == '\0') {
      msg->data = v;
      return true;
    }
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      if (root["data"].is<float>()) {
        msg->data = root["data"].as<float>();
        ok = true;
      }
      return true;
    }) && ok;
  }
  if (strcmp(type->name, "std_msgs/Int32") == 0) {
    if (out_len < sizeof(Int32Msg))
      return false;
    auto *msg = static_cast<Int32Msg *>(out);
    char *end = nullptr;
    long v = strtol(payload.c_str(), &end, 10);
    if (end != nullptr && *end == '\0') {
      msg->data = static_cast<int32_t>(v);
      return true;
    }
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      if (root["data"].is<int>()) {
        msg->data = root["data"].as<int>();
        ok = true;
      }
      return true;
    }) && ok;
  }
  if (strcmp(type->name, "std_msgs/String") == 0) {
    if (out_len < sizeof(StringMsg))
      return false;
    auto *msg = static_cast<StringMsg *>(out);
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      if (root["data"].is<const char *>()) {
        strncpy(msg->data, root["data"].as<const char *>(), ROS2_STRING_LEN - 1);
        msg->data[ROS2_STRING_LEN - 1] = '\0';
        ok = true;
      }
      return true;
    }) && ok;
  }
  if (strcmp(type->name, "sensor_msgs/JointState") == 0) {
    if (out_len < sizeof(JointStateMsg))
      return false;
    auto *msg = static_cast<JointStateMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      JsonArrayConst names = root["name"].as<JsonArrayConst>();
      JsonArrayConst pos = root["position"].as<JsonArrayConst>();
      if (names.isNull() || pos.isNull())
        return true;
      size_t n = names.size();
      if (n > ROS2_MAX_JOINTS)
        n = ROS2_MAX_JOINTS;
      msg->num_joints = n;
      size_t i = 0;
      for (JsonVariantConst v : names) {
        if (i >= n)
          break;
        copy_name(msg->name[i], v.as<const char *>());
        i++;
      }
      i = 0;
      for (JsonVariantConst v : pos) {
        if (i >= n)
          break;
        msg->position[i] = v.as<float>();
        i++;
      }
      JsonArrayConst vel = root["velocity"].as<JsonArrayConst>();
      if (!vel.isNull()) {
        i = 0;
        for (JsonVariantConst v : vel) {
          if (i >= n)
            break;
          msg->velocity[i++] = v.as<float>();
        }
      }
      JsonArrayConst eff = root["effort"].as<JsonArrayConst>();
      if (!eff.isNull()) {
        i = 0;
        for (JsonVariantConst v : eff) {
          if (i >= n)
            break;
          msg->effort[i++] = v.as<float>();
        }
      }
      ok = true;
      return true;
    }) && ok;
  }
  if (strcmp(type->name, "trajectory_msgs/JointTrajectory") == 0) {
    if (out_len < sizeof(JointTrajectoryMsg))
      return false;
    auto *msg = static_cast<JointTrajectoryMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      JsonArrayConst names = root["joint_names"].as<JsonArrayConst>();
      JsonArrayConst points = root["points"].as<JsonArrayConst>();
      if (names.isNull() || points.isNull() || points.size() == 0)
        return true;
      size_t n = names.size();
      if (n > ROS2_MAX_JOINTS)
        n = ROS2_MAX_JOINTS;
      msg->num_joints = n;
      size_t i = 0;
      for (JsonVariantConst v : names) {
        if (i >= n)
          break;
        copy_name(msg->joint_names[i++], v.as<const char *>());
      }
      JsonObjectConst p0 = points[0].as<JsonObjectConst>();
      if (!p0.isNull()) {
        JsonArrayConst pos = p0["positions"].as<JsonArrayConst>();
        if (!pos.isNull()) {
          i = 0;
          for (JsonVariantConst v : pos) {
            if (i >= n)
              break;
            msg->points[0].positions[i++] = v.as<float>();
          }
        }
      }
      msg->num_points = 1;
      ok = true;
      return true;
    }) && ok;
  }
  return false;
}

std::string JsonCodec::serialize(const TypeDef *type, const void *sample, size_t len) {
  if (type == nullptr || sample == nullptr)
    return "";
  if (strcmp(type->name, "std_msgs/Bool") == 0 && len >= sizeof(BoolMsg)) {
    return static_cast<const BoolMsg *>(sample)->data ? "true" : "false";
  }
  if (strcmp(type->name, "std_msgs/Float32") == 0 && len >= sizeof(Float32Msg)) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%f", static_cast<const Float32Msg *>(sample)->data);
    return buf;
  }
  if (strcmp(type->name, "std_msgs/Int32") == 0 && len >= sizeof(Int32Msg)) {
    return std::to_string(static_cast<const Int32Msg *>(sample)->data);
  }
  if (strcmp(type->name, "std_msgs/String") == 0 && len >= sizeof(StringMsg)) {
    auto msg = json::build_json([&](JsonObject root) {
      root["data"] = static_cast<const StringMsg *>(sample)->data;
    });
    return std::string(msg.c_str(), msg.size());
  }
  if (strcmp(type->name, "sensor_msgs/JointState") == 0 && len >= sizeof(JointStateMsg)) {
    auto *msg = static_cast<const JointStateMsg *>(sample);
    auto buf = json::build_json([&](JsonObject root) {
      JsonArray names = root["name"].to<JsonArray>();
      JsonArray pos = root["position"].to<JsonArray>();
      for (uint8_t i = 0; i < msg->num_joints; i++) {
        names.add(msg->name[i]);
        pos.add(msg->position[i]);
      }
    });
    return std::string(buf.c_str(), buf.size());
  }
  return "";
}

}  // namespace ros2
}  // namespace esphome

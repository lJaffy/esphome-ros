#include "ros2_json.h"

#include <cmath>
#include <cstring>

#include "esphome/components/json/json_util.h"
#include "ros2_types.h"

namespace esphome {
namespace ros2 {

static void copy_name(char *dst, const char *src) {
  strncpy(dst, src, ROS2_NAME_LEN - 1);
  dst[ROS2_NAME_LEN - 1] = '\0';
}

// rosbridge-style header: {"stamp": {"sec": N, "nanosec": N}, "frame_id": ""}.
static void write_header(JsonObject root, const HeaderMsg &h) {
  JsonObject header = root["header"].to<JsonObject>();
  JsonObject stamp = header["stamp"].to<JsonObject>();
  stamp["sec"] = h.stamp_sec;
  stamp["nanosec"] = h.stamp_nsec;
  header["frame_id"] = h.frame_id;
}

static void read_header(JsonObjectConst root, HeaderMsg *h) {
  JsonObjectConst header = root["header"].as<JsonObjectConst>();
  if (header.isNull())
    return;
  JsonObjectConst stamp = header["stamp"].as<JsonObjectConst>();
  if (!stamp.isNull()) {
    if (stamp["sec"].is<int>())
      h->stamp_sec = stamp["sec"].as<int>();
    if (stamp["nanosec"].is<unsigned int>())
      h->stamp_nsec = stamp["nanosec"].as<unsigned int>();
  }
  if (header["frame_id"].is<const char *>()) {
    strncpy(h->frame_id, header["frame_id"].as<const char *>(), ROS2_FRAME_ID_LEN - 1);
    h->frame_id[ROS2_FRAME_ID_LEN - 1] = '\0';
  }
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
      read_header(root, &msg->header);
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
      read_header(root, &msg->header);
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
  if (strcmp(type->name, "std_msgs/ColorRGBA") == 0) {
    if (out_len < sizeof(ColorRGBAMsg))
      return false;
    auto *msg = static_cast<ColorRGBAMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      if (root["r"].is<float>())
        msg->r = root["r"].as<float>();
      if (root["g"].is<float>())
        msg->g = root["g"].as<float>();
      if (root["b"].is<float>())
        msg->b = root["b"].as<float>();
      if (root["a"].is<float>())
        msg->a = root["a"].as<float>();
      ok = true;
      return true;
    }) && ok;
  }
  if (strcmp(type->name, "sensor_msgs/Joy") == 0) {
    if (out_len < sizeof(JoyMsg))
      return false;
    auto *msg = static_cast<JoyMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      read_header(root, &msg->header);
      JsonArrayConst axes = root["axes"].as<JsonArrayConst>();
      if (!axes.isNull()) {
        size_t n = axes.size();
        if (n > ROS2_MAX_JOY_AXES)
          n = ROS2_MAX_JOY_AXES;
        msg->num_axes = n;
        size_t i = 0;
        for (JsonVariantConst v : axes) {
          if (i >= n)
            break;
          msg->axes[i++] = v.as<float>();
        }
      }
      JsonArrayConst buttons = root["buttons"].as<JsonArrayConst>();
      if (!buttons.isNull()) {
        size_t n = buttons.size();
        if (n > ROS2_MAX_JOY_BUTTONS)
          n = ROS2_MAX_JOY_BUTTONS;
        msg->num_buttons = n;
        size_t i = 0;
        for (JsonVariantConst v : buttons) {
          if (i >= n)
            break;
          msg->buttons[i++] = v.as<int>();
        }
      }
      ok = true;
      return true;
    }) && ok;
  }
  if (strcmp(type->name, "sensor_msgs/Range") == 0) {
    if (out_len < sizeof(RangeMsg))
      return false;
    auto *msg = static_cast<RangeMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      if (!root["range"].is<float>())
        return true;
      read_header(root, &msg->header);
      msg->range = root["range"].as<float>();
      if (root["radiation_type"].is<int>())
        msg->radiation_type = (uint8_t) root["radiation_type"].as<int>();
      if (root["field_of_view"].is<float>())
        msg->field_of_view = root["field_of_view"].as<float>();
      if (root["min_range"].is<float>())
        msg->min_range = root["min_range"].as<float>();
      if (root["max_range"].is<float>())
        msg->max_range = root["max_range"].as<float>();
      if (root["variance"].is<float>())
        msg->variance = root["variance"].as<float>();
      ok = true;
      return true;
    }) && ok;
  }
  if (strcmp(type->name, "sensor_msgs/BatteryState") == 0) {
    if (out_len < sizeof(BatteryStateMsg))
      return false;
    auto *msg = static_cast<BatteryStateMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    msg->temperature = NAN;
    msg->current = NAN;
    msg->charge = NAN;
    msg->capacity = NAN;
    msg->design_capacity = NAN;
    msg->percentage = NAN;
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      if (!root["voltage"].is<float>())
        return true;
      read_header(root, &msg->header);
      msg->voltage = root["voltage"].as<float>();
      if (root["temperature"].is<float>())
        msg->temperature = root["temperature"].as<float>();
      if (root["current"].is<float>())
        msg->current = root["current"].as<float>();
      if (root["charge"].is<float>())
        msg->charge = root["charge"].as<float>();
      if (root["capacity"].is<float>())
        msg->capacity = root["capacity"].as<float>();
      if (root["design_capacity"].is<float>())
        msg->design_capacity = root["design_capacity"].as<float>();
      if (root["percentage"].is<float>())
        msg->percentage = root["percentage"].as<float>();
      if (root["power_supply_status"].is<int>())
        msg->power_supply_status = (uint8_t) root["power_supply_status"].as<int>();
      if (root["power_supply_health"].is<int>())
        msg->power_supply_health = (uint8_t) root["power_supply_health"].as<int>();
      if (root["power_supply_technology"].is<int>())
        msg->power_supply_technology = (uint8_t) root["power_supply_technology"].as<int>();
      if (root["present"].is<bool>())
        msg->present = root["present"].as<bool>();
      if (root["location"].is<const char *>())
        copy_name(msg->location, root["location"].as<const char *>());
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
      write_header(root, msg->header);
      JsonArray names = root["name"].to<JsonArray>();
      JsonArray pos = root["position"].to<JsonArray>();
      for (uint8_t i = 0; i < msg->num_joints; i++) {
        names.add(msg->name[i]);
        pos.add(msg->position[i]);
      }
    });
    return std::string(buf.c_str(), buf.size());
  }
  if (strcmp(type->name, "std_msgs/ColorRGBA") == 0 && len >= sizeof(ColorRGBAMsg)) {
    auto *msg = static_cast<const ColorRGBAMsg *>(sample);
    auto buf = json::build_json([&](JsonObject root) {
      root["r"] = msg->r;
      root["g"] = msg->g;
      root["b"] = msg->b;
      root["a"] = msg->a;
    });
    return std::string(buf.c_str(), buf.size());
  }
  if (strcmp(type->name, "sensor_msgs/Range") == 0 && len >= sizeof(RangeMsg)) {
    auto *msg = static_cast<const RangeMsg *>(sample);
    auto buf = json::build_json([&](JsonObject root) {
      write_header(root, msg->header);
      root["radiation_type"] = msg->radiation_type;
      root["field_of_view"] = msg->field_of_view;
      root["min_range"] = msg->min_range;
      root["max_range"] = msg->max_range;
      root["range"] = msg->range;
      root["variance"] = msg->variance;
    });
    return std::string(buf.c_str(), buf.size());
  }
  if (strcmp(type->name, "sensor_msgs/BatteryState") == 0 && len >= sizeof(BatteryStateMsg)) {
    auto *msg = static_cast<const BatteryStateMsg *>(sample);
    auto buf = json::build_json([&](JsonObject root) {
      write_header(root, msg->header);
      root["voltage"] = msg->voltage;
      // Unmeasured fields are NaN per the IDL; JSON has no NaN, so absent
      // keys mean unmeasured (CDR encodes them as NaN doubles).
      if (!isnan(msg->temperature))
        root["temperature"] = msg->temperature;
      if (!isnan(msg->current))
        root["current"] = msg->current;
      if (!isnan(msg->charge))
        root["charge"] = msg->charge;
      if (!isnan(msg->capacity))
        root["capacity"] = msg->capacity;
      if (!isnan(msg->design_capacity))
        root["design_capacity"] = msg->design_capacity;
      if (!isnan(msg->percentage))
        root["percentage"] = msg->percentage;
      root["power_supply_status"] = msg->power_supply_status;
      root["power_supply_health"] = msg->power_supply_health;
      root["power_supply_technology"] = msg->power_supply_technology;
      root["present"] = msg->present;
      root["location"] = msg->location;
    });
    return std::string(buf.c_str(), buf.size());
  }
  return "";
}

}  // namespace ros2
}  // namespace esphome

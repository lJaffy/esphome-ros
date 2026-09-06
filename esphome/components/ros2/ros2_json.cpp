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
  if (strcmp(type->name, "sensor_msgs/Range") == 0) {    if (out_len < sizeof(RangeMsg))
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
  if (strcmp(type->name, "geometry_msgs/Twist") == 0) {
    if (out_len < sizeof(TwistMsg))
      return false;
    auto *msg = static_cast<TwistMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    // All-zero Twist is a valid stop command, so parse success never depends
    // on any single key: absent fields read as zero.
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      JsonObjectConst linear = root["linear"].as<JsonObjectConst>();
      if (!linear.isNull()) {
        if (linear["x"].is<float>())
          msg->linear_x = linear["x"].as<float>();
        if (linear["y"].is<float>())
          msg->linear_y = linear["y"].as<float>();
        if (linear["z"].is<float>())
          msg->linear_z = linear["z"].as<float>();
      }
      JsonObjectConst angular = root["angular"].as<JsonObjectConst>();
      if (!angular.isNull()) {
        if (angular["x"].is<float>())
          msg->angular_x = angular["x"].as<float>();
        if (angular["y"].is<float>())
          msg->angular_y = angular["y"].as<float>();
        if (angular["z"].is<float>())
          msg->angular_z = angular["z"].as<float>();
      }
      ok = true;
      return true;
    }) && ok;
  }
  if (strcmp(type->name, "nav_msgs/Odometry") == 0) {
    if (out_len < sizeof(OdometryMsg))
      return false;
    auto *msg = static_cast<OdometryMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      read_header(root, &msg->header);
      if (root["child_frame_id"].is<const char *>())
        copy_name(msg->child_frame_id, root["child_frame_id"].as<const char *>());
      JsonObjectConst pose = root["pose"].as<JsonObjectConst>();
      if (!pose.isNull()) {
        JsonObjectConst pos = pose["position"].as<JsonObjectConst>();
        if (!pos.isNull()) {
          if (pos["x"].is<float>())
            msg->pose_position[0] = pos["x"].as<float>();
          if (pos["y"].is<float>())
            msg->pose_position[1] = pos["y"].as<float>();
          if (pos["z"].is<float>())
            msg->pose_position[2] = pos["z"].as<float>();
        }
        JsonObjectConst ori = pose["orientation"].as<JsonObjectConst>();
        if (!ori.isNull()) {
          if (ori["x"].is<float>())
            msg->pose_orientation[0] = ori["x"].as<float>();
          if (ori["y"].is<float>())
            msg->pose_orientation[1] = ori["y"].as<float>();
          if (ori["z"].is<float>())
            msg->pose_orientation[2] = ori["z"].as<float>();
          if (ori["w"].is<float>())
            msg->pose_orientation[3] = ori["w"].as<float>();
        }
      }
      JsonObjectConst twist = root["twist"].as<JsonObjectConst>();
      if (!twist.isNull()) {
        JsonObjectConst lin = twist["linear"].as<JsonObjectConst>();
        if (!lin.isNull()) {
          if (lin["x"].is<float>())
            msg->twist_linear[0] = lin["x"].as<float>();
          if (lin["y"].is<float>())
            msg->twist_linear[1] = lin["y"].as<float>();
          if (lin["z"].is<float>())
            msg->twist_linear[2] = lin["z"].as<float>();
        }
        JsonObjectConst ang = twist["angular"].as<JsonObjectConst>();
        if (!ang.isNull()) {
          if (ang["x"].is<float>())
            msg->twist_angular[0] = ang["x"].as<float>();
          if (ang["y"].is<float>())
            msg->twist_angular[1] = ang["y"].as<float>();
          if (ang["z"].is<float>())
            msg->twist_angular[2] = ang["z"].as<float>();
        }
      }
      ok = true;
      return true;
    }) && ok;
  }
  if (strcmp(type->name, "tf2_msgs/TFMessage") == 0) {
    if (out_len < sizeof(TFMessageMsg))
      return false;
    auto *msg = static_cast<TFMessageMsg *>(out);
    memset(msg, 0, sizeof(*msg));
    return json::parse_json(payload, [&](JsonObject root) -> bool {
      JsonArrayConst transforms = root["transforms"].as<JsonArrayConst>();
      if (transforms.isNull())
        return true;
      size_t n = transforms.size();
      if (n > ROS2_MAX_TF_TRANSFORMS)
        n = ROS2_MAX_TF_TRANSFORMS;
      msg->num_transforms = n;
      size_t i = 0;
      for (JsonVariantConst v : transforms) {
        if (i >= n)
          break;
        JsonObjectConst t = v.as<JsonObjectConst>();
        if (t.isNull())
          continue;
        TFTransformMsg *dst = &msg->transforms[i];
        read_header(t, &dst->header);
        if (t["child_frame_id"].is<const char *>())
          copy_name(dst->child_frame_id, t["child_frame_id"].as<const char *>());
        JsonObjectConst tf = t["transform"].as<JsonObjectConst>();
        if (!tf.isNull()) {
          JsonObjectConst tr = tf["translation"].as<JsonObjectConst>();
          if (!tr.isNull()) {
            if (tr["x"].is<float>())
              dst->translation[0] = tr["x"].as<float>();
            if (tr["y"].is<float>())
              dst->translation[1] = tr["y"].as<float>();
            if (tr["z"].is<float>())
              dst->translation[2] = tr["z"].as<float>();
          }
          JsonObjectConst rot = tf["rotation"].as<JsonObjectConst>();
          if (!rot.isNull()) {
            if (rot["x"].is<float>())
              dst->rotation[0] = rot["x"].as<float>();
            if (rot["y"].is<float>())
              dst->rotation[1] = rot["y"].as<float>();
            if (rot["z"].is<float>())
              dst->rotation[2] = rot["z"].as<float>();
            if (rot["w"].is<float>())
              dst->rotation[3] = rot["w"].as<float>();
          }
        }
        i++;
      }
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
  if (strcmp(type->name, "nav_msgs/Odometry") == 0 && len >= sizeof(OdometryMsg)) {
    auto *msg = static_cast<const OdometryMsg *>(sample);
    auto buf = json::build_json([&](JsonObject root) {
      write_header(root, msg->header);
      root["child_frame_id"] = msg->child_frame_id;
      JsonObject pose = root["pose"].to<JsonObject>();
      JsonObject pos = pose["position"].to<JsonObject>();
      pos["x"] = msg->pose_position[0];
      pos["y"] = msg->pose_position[1];
      pos["z"] = msg->pose_position[2];
      JsonObject ori = pose["orientation"].to<JsonObject>();
      ori["x"] = msg->pose_orientation[0];
      ori["y"] = msg->pose_orientation[1];
      ori["z"] = msg->pose_orientation[2];
      ori["w"] = msg->pose_orientation[3];
      JsonArray cov = pose["covariance"].to<JsonArray>();
      for (uint8_t i = 0; i < 36; i++)
        cov.add(0.0f);
      JsonObject twist = root["twist"].to<JsonObject>();
      JsonObject lin = twist["linear"].to<JsonObject>();
      lin["x"] = msg->twist_linear[0];
      lin["y"] = msg->twist_linear[1];
      lin["z"] = msg->twist_linear[2];
      JsonObject ang = twist["angular"].to<JsonObject>();
      ang["x"] = msg->twist_angular[0];
      ang["y"] = msg->twist_angular[1];
      ang["z"] = msg->twist_angular[2];
      JsonArray tcov = twist["covariance"].to<JsonArray>();
      for (uint8_t i = 0; i < 36; i++)
        tcov.add(0.0f);
    });
    return std::string(buf.c_str(), buf.size());
  }
  if (strcmp(type->name, "tf2_msgs/TFMessage") == 0 && len >= sizeof(TFMessageMsg)) {
    auto *msg = static_cast<const TFMessageMsg *>(sample);
    auto buf = json::build_json([&](JsonObject root) {
      JsonArray transforms = root["transforms"].to<JsonArray>();
      for (uint8_t i = 0; i < msg->num_transforms; i++) {
        const TFTransformMsg *t = &msg->transforms[i];
        JsonObject entry = transforms.add<JsonObject>();
        write_header(entry, t->header);
        entry["child_frame_id"] = t->child_frame_id;
        JsonObject tf = entry["transform"].to<JsonObject>();
        JsonObject tr = tf["translation"].to<JsonObject>();
        tr["x"] = t->translation[0];
        tr["y"] = t->translation[1];
        tr["z"] = t->translation[2];
        JsonObject rot = tf["rotation"].to<JsonObject>();
        rot["x"] = t->rotation[0];
        rot["y"] = t->rotation[1];
        rot["z"] = t->rotation[2];
        rot["w"] = t->rotation[3];
      }
    });
    return std::string(buf.c_str(), buf.size());
  }
  return "";
}

}  // namespace ros2
}  // namespace esphome

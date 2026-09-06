#pragma once

#include <cstddef>
#include <cstdint>

#include "ros2_middleware.h"

namespace esphome
{
  namespace ros2
  {

    constexpr size_t ROS2_MAX_JOINTS = 16;
    constexpr size_t ROS2_MAX_TRAJ_POINTS = 2;
    constexpr size_t ROS2_MAX_JOY_AXES = 16;
    constexpr size_t ROS2_MAX_JOY_BUTTONS = 16;
    constexpr size_t ROS2_NAME_LEN = 32;
    constexpr size_t ROS2_STRING_LEN = 256;
    constexpr size_t ROS2_FRAME_ID_LEN = 64;

    struct BoolMsg
    {
      bool data{false};
    };
    struct Float32Msg
    {
      float data{0.0f};
    };
    struct Int32Msg
    {
      int32_t data{0};
    };
    struct StringMsg
    {
      char data[ROS2_STRING_LEN]{0};
    };

    // MCU projection of std_msgs/Header (see third_party/common_interfaces).
    // ROS 2 Header has no seq; stamp is builtin_interfaces/Time as int sec + nsec.
    struct HeaderMsg
    {
      int32_t stamp_sec{0};
      uint32_t stamp_nsec{0};
      char frame_id[ROS2_FRAME_ID_LEN]{0};
    };

    struct JointStateMsg
    {
      HeaderMsg header;
      uint8_t num_joints{0};
      char name[ROS2_MAX_JOINTS][ROS2_NAME_LEN]{0};
      float position[ROS2_MAX_JOINTS]{0.0f};
      float velocity[ROS2_MAX_JOINTS]{0.0f};
      float effort[ROS2_MAX_JOINTS]{0.0f};
    };

    struct JointTrajectoryPointMsg
    {
      float positions[ROS2_MAX_JOINTS]{0.0f};
      float velocities[ROS2_MAX_JOINTS]{0.0f};
      float accelerations[ROS2_MAX_JOINTS]{0.0f};
      float effort[ROS2_MAX_JOINTS]{0.0f};
      uint32_t time_sec{0};
      uint32_t time_nsec{0};
    };

    struct JointTrajectoryMsg
    {
      HeaderMsg header;
      uint8_t num_joints{0};
      char joint_names[ROS2_MAX_JOINTS][ROS2_NAME_LEN]{0};
      uint8_t num_points{0};
      JointTrajectoryPointMsg points[ROS2_MAX_TRAJ_POINTS]{};
    };

    struct ColorRGBAMsg
    {
      float r{0.0f};
      float g{0.0f};
      float b{0.0f};
      float a{0.0f};
    };

    struct JoyMsg
    {
      HeaderMsg header;
      uint8_t num_axes{0};
      uint8_t num_buttons{0};
      float axes[ROS2_MAX_JOY_AXES]{0.0f};
      int32_t buttons[ROS2_MAX_JOY_BUTTONS]{0};
    };

    const TypeDef *find_type(const char *name);
    bool is_multi_joint_type(const TypeDef *type);

  } // namespace ros2
} // namespace esphome

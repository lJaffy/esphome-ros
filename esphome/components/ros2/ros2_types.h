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
    constexpr size_t ROS2_MAX_TF_TRANSFORMS = 4;
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

    // MCU projection of sensor_msgs/Range: radiation_type is ULTRASOUND (0)
    // or INFRARED (1); variance 0 means unknown.
    struct RangeMsg
    {
      HeaderMsg header;
      uint8_t radiation_type{0};
      float field_of_view{0.0f};
      float min_range{0.0f};
      float max_range{0.0f};
      float range{0.0f};
      float variance{0.0f};
    };

    // MCU projection of sensor_msgs/BatteryState. Unmeasured float fields
    // use NaN per the IDL (voltage is mandatory). Cell arrays always publish
    // empty; serial_number always publishes empty.
    struct BatteryStateMsg
    {
      HeaderMsg header;
      float voltage{0.0f};
      float temperature{0.0f};
      float current{0.0f};
      float charge{0.0f};
      float capacity{0.0f};
      float design_capacity{0.0f};
      float percentage{0.0f};
      uint8_t power_supply_status{0};
      uint8_t power_supply_health{0};
      uint8_t power_supply_technology{0};
      bool present{false};
      char location[ROS2_NAME_LEN]{0};
    };

    // MCU projection of geometry_msgs/Twist: planar drivers use linear.x
    // and angular.z; the rest pass through the codecs untouched.
    struct TwistMsg
    {
      float linear_x{0.0f};
      float linear_y{0.0f};
      float linear_z{0.0f};
      float angular_x{0.0f};
      float angular_y{0.0f};
      float angular_z{0.0f};
    };

    // MCU projection of nav_msgs/Odometry. Covariances publish as zeros
    // (36 doubles each); the bridge keeps no uncertainty model.
    struct OdometryMsg
    {
      HeaderMsg header;
      char child_frame_id[ROS2_FRAME_ID_LEN]{0};
      float pose_position[3]{0.0f};
      float pose_orientation[4]{0.0f};
      float twist_linear[3]{0.0f};
      float twist_angular[3]{0.0f};
    };

    // MCU projection of geometry_msgs/TransformStamped + tf2_msgs/TFMessage
    // (a sequence of the former). tf2_msgs lives in ros2/geometry2, not in
    // the common_interfaces submodule, so parity covers members/keys only.
    struct TFTransformMsg
    {
      HeaderMsg header;
      char child_frame_id[ROS2_FRAME_ID_LEN]{0};
      float translation[3]{0.0f};
      float rotation[4]{0.0f};
    };

    struct TFMessageMsg
    {
      uint8_t num_transforms{0};
      TFTransformMsg transforms[ROS2_MAX_TF_TRANSFORMS]{};
    };

    // MCU projection of sensor_msgs/Imu. Covariances are float[9] here,
    // float64[9] on the wire. Conventions per the IDL: all-zeros means
    // "covariance unknown"; element 0 = -1 means "no estimate for this
    // element" (used for orientation when no orientation source is bound).
    struct ImuMsg
    {
      HeaderMsg header;
      float orientation[4]{0.0f};
      float orientation_covariance[9]{0.0f};
      float angular_velocity[3]{0.0f};
      float angular_velocity_covariance[9]{0.0f};
      float linear_acceleration[3]{0.0f};
      float linear_acceleration_covariance[9]{0.0f};
    };

    // MCU projection of sensor_msgs/NavSatFix. Position covariance is
    // float[9] here, float64[9] on the wire. Status/service follow the
    // NavSatStatus IDL (STATUS_FIX = 0, SERVICE_GPS = 1); covariance publishes
    // as zeros with type UNKNOWN (no uncertainty model on the MCU).
    struct NavSatFixMsg
    {
      HeaderMsg header;
      int8_t status{0};
      uint16_t service{0};
      float latitude{0.0f};
      float longitude{0.0f};
      float altitude{0.0f};
      float position_covariance[9]{0.0f};
      uint8_t position_covariance_type{0};
    };

    const TypeDef *find_type(const char *name);
    bool is_multi_joint_type(const TypeDef *type);

  } // namespace ros2
} // namespace esphome

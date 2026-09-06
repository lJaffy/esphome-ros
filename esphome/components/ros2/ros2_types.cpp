#include "ros2_types.h"

#include <cstring>

namespace esphome {
namespace ros2 {

static const TypeDef K_TYPE_DEFS[] = {
    {"std_msgs/Bool", "std_msgs__Bool", sizeof(BoolMsg)},
    {"std_msgs/Float32", "std_msgs__Float32", sizeof(Float32Msg)},
    {"std_msgs/Int32", "std_msgs__Int32", sizeof(Int32Msg)},
    {"std_msgs/String", "std_msgs__String", sizeof(StringMsg)},
    {"sensor_msgs/JointState", "sensor_msgs__JointState", sizeof(JointStateMsg)},
    {"trajectory_msgs/JointTrajectory", "trajectory_msgs__JointTrajectory", sizeof(JointTrajectoryMsg)},
    {"std_msgs/ColorRGBA", "std_msgs__ColorRGBA", sizeof(ColorRGBAMsg)},
    {"sensor_msgs/Joy", "sensor_msgs__Joy", sizeof(JoyMsg)},
    {"sensor_msgs/Range", "sensor_msgs__Range", sizeof(RangeMsg)},
    {"sensor_msgs/BatteryState", "sensor_msgs__BatteryState", sizeof(BatteryStateMsg)},
    {"geometry_msgs/Twist", "geometry_msgs__Twist", sizeof(TwistMsg)},
    {"nav_msgs/Odometry", "nav_msgs__Odometry", sizeof(OdometryMsg)},
    {"tf2_msgs/TFMessage", "tf2_msgs__TFMessage", sizeof(TFMessageMsg)},
    {"sensor_msgs/Imu", "sensor_msgs__Imu", sizeof(ImuMsg)},
    {"sensor_msgs/NavSatFix", "sensor_msgs__NavSatFix", sizeof(NavSatFixMsg)},
    // Variable-length frame buffer; not backed by a fixed struct. Size 0 marks
    // types published straight from the source buffer (see publish_image).
    {"sensor_msgs/CompressedImage", "sensor_msgs__CompressedImage", 0},
};

const TypeDef *find_type(const char *name) {
  for (const auto &def : K_TYPE_DEFS) {
    if (strcmp(def.name, name) == 0)
      return &def;
  }
  return nullptr;
}

bool is_multi_joint_type(const TypeDef *type) {
  if (type == nullptr)
    return false;
  return strcmp(type->name, "sensor_msgs/JointState") == 0 ||
         strcmp(type->name, "trajectory_msgs/JointTrajectory") == 0;
}

}  // namespace ros2
}  // namespace esphome

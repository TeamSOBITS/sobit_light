#ifndef SOBIT_LIGHT_WHEEL_CONTROLLER_H_
#define SOBIT_LIGHT_WHEEL_CONTROLLER_H_

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>


namespace sobit_light {
class WheelController : public rclcpp::Node {
 public:
  WheelController(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  WheelController(
      const std::string& node_name,
      const std::string& name_space,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~WheelController() = default;

  bool controlWheelLinear(const double distance);
  bool controlWheelRotateRad(const double angle_rad);
  bool controlWheelRotateDeg(const double angle_deg);

  double rad2Deg(const double rad);
  double deg2Rad(const double deg);

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  pub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  nav_msgs::msg::Odometry curt_odom_;

  void callbackOdometry(
      const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  double geometryQuat2Yaw(
      const geometry_msgs::msg::Quaternion& geometry_quat);

};
}  // namespace sobit_light


inline void sobit_light::WheelController::callbackOdometry(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  curt_odom_ = *odom_msg;
}

inline double sobit_light::WheelController::geometryQuat2Yaw(
    const geometry_msgs::msg::Quaternion& geometry_quat) {
  tf2::Quaternion quat_tf;
  double roll, pitch, yaw;

  tf2::fromMsg(geometry_quat, quat_tf);
  quat_tf.normalize();
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  return yaw;  
}

inline double sobit_light::WheelController::rad2Deg(
    const double rad) {
  return rad * 180.0 / M_PI;
}

inline double sobit_light::WheelController::deg2Rad(
    const double deg) {
  return deg * M_PI / 180.0;
}

#endif  // SOBIT_LIGHT_WHEEL_CONTROLLER_H_

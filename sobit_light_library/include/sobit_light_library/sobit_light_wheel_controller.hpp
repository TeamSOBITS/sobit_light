#ifndef SOBIT_LIGHT_WHEEL_CONTROLLER_H_
#define SOBIT_LIGHT_WHEEL_CONTROLLER_H_

#include <cmath>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include "sobit_light_library/sobit_light_library.h"

namespace sobit_light {
    class WheelController : public rclcpp::Node {
        private:
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

            nav_msgs::msg::Odometry curt_odom_;

            void checkPublishersConnection(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& pub);
            void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
            double geometryQuat2Yaw(const geometry_msgs::msg::Quaternion& geometry_quat);

        public:
            WheelController(const std::string &name);
            WheelController();
            WheelController(const rclcpp::NodeOptions & options);

            bool controlWheelLinear(const double distance);
            bool controlWheelRotateRad(const double angle_rad);
            bool controlWheelRotateDeg(const double angle_deg);

            double rad2Deg(const double rad);
            double deg2Rad(const double deg);
    };
}

// inline sobit_light::WheelController::WheelController(const rclcpp::NodeOptions & options)
// : Node("sobit_light_wheel_controller", options) {
//     sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "/odom", 1, std::bind(&WheelController::callbackOdometry, this, std::placeholders::_1));
//     pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_mux/input/teleop", 1);
// }

// inline sobit_light::WheelController::WheelController()
// : Node("sobit_light_wheel_controller") {
//     sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "/odom", 1, std::bind(&WheelController::callbackOdometry, this, std::placeholders::_1));
//     pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_mux/input/teleop", 1);
// }

inline void sobit_light::WheelController::checkPublishersConnection(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& pub) {
    rclcpp::Rate loop_rate(10);

    while (pub->get_subscription_count() == 0 && rclcpp::ok()) {
        try { loop_rate.sleep(); }
        catch (const std::exception& ex) { break; }
    }

    return; 
}

inline void sobit_light::WheelController::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    curt_odom_ = *odom_msg;
}

inline double sobit_light::WheelController::geometryQuat2Yaw(const geometry_msgs::msg::Quaternion& geometry_quat) {
    tf2::Quaternion quat_tf;
    double roll, pitch, yaw;

    tf2::fromMsg(geometry_quat, quat_tf);
    quat_tf.normalize();
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

    return yaw;  
}

inline double sobit_light::WheelController::rad2Deg(const double rad) {
    return rad * 180.0 / M_PI;
}

inline double sobit_light::WheelController::deg2Rad(const double deg) {
    return deg * M_PI / 180.0;
}

#endif /* SOBIT_LIGHT_WHEEL_CONTROLLER_H_ */

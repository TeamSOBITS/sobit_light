#ifndef SOBIT_LIGHT_LIBRARY_H_
#define SOBIT_LIGHT_LIBRARY_H_

#include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>

class ROSCommonNode : public rclcpp::Node {
 public:
  ROSCommonNode(const std::string& name)
  : rclcpp::Node(name) {
    // ROS2では特別な初期化は必要ありません
  }

  ROSCommonNode()
  : rclcpp::Node("sobit_light_library_node") {
    // デフォルトのノード名で初期化
  }
};

#endif /* SOBIT_LIGHT_LIBRARY_H_ */
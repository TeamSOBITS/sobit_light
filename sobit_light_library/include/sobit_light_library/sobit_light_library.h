#ifndef SOBIT_LIGHT_LIBRARY_H_
#define SOBIT_LIGHT_LIBRARY_H_

// #include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>

class ROSCommonNode : public rclcpp::Node {
 public:
  ROSCommonNode()
  : rclcpp::Node("sobit_light_library_node") {}
  ROSCommonNode(const std::string& name)
  : rclcpp::Node(name) {}
  ROSCommonNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("sobit_light_library_node", options) {}
  ROSCommonNode(
      const std::string& name,
      const rclcpp::NodeOptions& options)
  : rclcpp::Node(name, options) {}
};

#endif  // SOBIT_LIGHT_LIBRARY_H_

#ifndef SOBIT_LIGHT_LIBRARY_H_
#define SOBIT_LIGHT_LIBRARY_H_

#include <rclcpp/rclcpp.hpp>

class ROSCommonNode : public rclcpp::Node {
 public:
  ROSCommonNode()
  : Node("sobit_light_library_node") {}
  ROSCommonNode(
      const std::string& name)
  : Node(name) {}
  ROSCommonNode(
      const rclcpp::NodeOptions& options)
  : Node("sobit_light_library_node", options) {}
  ROSCommonNode(
      const std::string& name_space,
      const rclcpp::NodeOptions& options)
  : Node("sobit_light_library_node", name_space, options) {}
};

#endif  // SOBIT_LIGHT_LIBRARY_H_

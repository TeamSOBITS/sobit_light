// #ifndef SOBIT_LIGHT_POSE_PARAM_HPP
// #define SOBIT_LIGHT_POSE_PARAM_HPP

#include <vector>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace sobit_light{

struct PoseParams {
  std::string pose_name;
  double arm_shoulder_roll;
  double arm_shoulder_pitch;
  double arm_elbow_pitch;
  double arm_forearm_roll;
  double arm_wrist_pitch;
  double arm_wrist_roll;
  double hand;
  double head_yaw;
  double head_pitch;
};

class PoseParamRead : public rclcpp::Node{
public:
  PoseParamRead();
  ~PoseParamRead();

private:
  std::vector<PoseParams> poses_;
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
};

} // namespace sobit_light

// #endif // SOBIT_LIGHT_POSE_PARAM_HPP

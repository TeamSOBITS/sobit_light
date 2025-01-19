#include "sobit_light_library/sobit_light_param.hpp"

namespace sobit_light{

PoseParamRead::PoseParamRead() : Node("param_read") {
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&PoseParamRead::timer_callback, this));

  this->declare_parameter("poses", std::vector<std::string>());
  auto pose_names = this->get_parameter("poses").as_string_array();

  poses_.clear();
  for (auto pose_name : pose_names) {
    // Declare parameters for each pose
    this->declare_parameter(pose_name + ".arm_shoulder_roll" , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_shoulder_pitch", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_elbow_pitch"   , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_forearm_roll"  , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_wrist_pitch"   , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_wrist_roll"    , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand"              , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".head_yaw"          , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".head_pitch"        , rclcpp::PARAMETER_DOUBLE);

    // Read parameters for each pose
    PoseParams params;
    params.pose_name          = pose_name;
    params.arm_shoulder_roll  = this->get_parameter(pose_name + ".arm_shoulder_roll").as_double();
    params.arm_shoulder_pitch = this->get_parameter(pose_name + ".arm_shoulder_pitch").as_double();
    params.arm_elbow_pitch    = this->get_parameter(pose_name + ".arm_elbow_pitch").as_double();
    params.arm_forearm_roll   = this->get_parameter(pose_name + ".arm_forearm_roll").as_double();
    params.arm_wrist_pitch    = this->get_parameter(pose_name + ".arm_wrist_pitch").as_double();
    params.arm_wrist_roll     = this->get_parameter(pose_name + ".arm_wrist_roll").as_double();
    params.hand               = this->get_parameter(pose_name + ".hand").as_double();
    params.head_yaw           = this->get_parameter(pose_name + ".head_yaw").as_double();
    params.head_pitch         = this->get_parameter(pose_name + ".head_pitch").as_double();

    poses_.push_back(params);
  }

  RCLCPP_INFO(this->get_logger(), "PoseParamRead has been initialized.");
}

PoseParamRead::~PoseParamRead() {
  RCLCPP_INFO(this->get_logger(), "PoseParamRead has been terminated.");
}

void PoseParamRead::timer_callback(){
  // Print out the parameters
  for (const auto & pose : poses_ ) {
    RCLCPP_INFO(this->get_logger(), "pose_name: '%s'", pose.pose_name.c_str());
    RCLCPP_INFO(this->get_logger(), "arm_shoulder_roll: '%.4f'", pose.arm_shoulder_roll);
    RCLCPP_INFO(this->get_logger(), "arm_shoulder_pitch: '%.4f'", pose.arm_shoulder_pitch);
    RCLCPP_INFO(this->get_logger(), "arm_elbow_pitch: '%.4f'", pose.arm_elbow_pitch);
    RCLCPP_INFO(this->get_logger(), "arm_forearm_roll: '%.4f'", pose.arm_forearm_roll);
    RCLCPP_INFO(this->get_logger(), "arm_wrist_pitch: '%.4f'", pose.arm_wrist_pitch);
    RCLCPP_INFO(this->get_logger(), "arm_wrist_roll: '%.4f'", pose.arm_wrist_roll);
    RCLCPP_INFO(this->get_logger(), "hand: '%.4f'", pose.hand);
    RCLCPP_INFO(this->get_logger(), "head_yaw: '%.4f'", pose.head_yaw);
    RCLCPP_INFO(this->get_logger(), "head_pitch: '%.4f'", pose.head_pitch);
  }
} 

} // namespace sobit_light


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sobit_light::PoseParamRead>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

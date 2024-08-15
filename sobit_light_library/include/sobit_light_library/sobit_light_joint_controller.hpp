#ifndef SOBIT_LIGHT_JOINT_CONTROLLER_H_
#define SOBIT_LIGHT_JOINT_CONTROLLER_H_

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// #include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "sobits_msgs/msg/current_state_array.h"
// #include "sobits_msgs/sobits_msgs/msg/current_state_array.h"
// #include "sobits_msgs/include/sobits_msgs/sobits_msgs/msg/current_state_array.h"


// #include "sobit_light_library/sobit_light_library.h"

namespace sobit_light {
enum Joint {
    kArmShoulderRollJoint = 0,
    kArmShoulderPitchJoint,
    // kArmShoulder1PitchJoint,
    // kArmShoulder2PitchJoint,
    kArmElbowPitchJoint,
    kArmForearmRollJoint,
    kArmWristPitchJoint,
    kArmWristRollJoint,
    kHandJoint,
    kHeadYawJoint,
    kHeadPitchJoint,
    kJointNum
};

typedef struct {     
  std::string         pose_name;
  std::vector<double> joint_val;
} Pose;

// class JointController : private ROSCommonNode {
class JointController : public rclcpp::Node {
 public:
  // JointController(const std::string &name);
  JointController(const rclcpp::NodeOptions& options);
  ~JointController();

  bool moveToPose(
      const std::string &pose_name,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveAllJointsRad(
      const double arm_shoulder_roll,
      const double arm_shoulder_pitch,
      const double arm_elbow_pitch,
      const double arm_forearm_roll,
      const double arm_wrist_pitch,
      const double arm_wrist_roll,
      const double hand,
      const double head_yaw,
      const double head_pitch,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveAllJointsDeg(
      const double arm_shoulder_roll,
      const double arm_shoulder_pitch,
      const double arm_elbow_pitch,
      const double arm_forearm_roll,
      const double arm_wrist_pitch,
      const double arm_wrist_roll,
      const double hand,
      const double head_yaw,
      const double head_pitch,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveJointRad(
      const Joint  joint_num,
      const double rad,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveJointDeg(
      const Joint  joint_num,
      const double deg,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveArmDeg(
      const double arm_shoulder_roll,
      const double arm_shoulder_pitch,
      const double arm_elbow_pitch,
      const double arm_forearm_roll,
      const double arm_wrist_pitch,
      const double arm_wrist_roll,
      const double hand,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveArmRad(
      const double arm_shoulder_roll,
      const double arm_shoulder_pitch,
      const double arm_elbow_pitch,
      const double arm_forearm_roll,
      const double arm_wrist_pitch,
      const double arm_wrist_roll,
      const double hand,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveHeadDeg(
      const double head_yaw,
      const double head_pitch,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveHeadRad(
      const double head_yaw,
      const double head_pitch,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveHandToTargetCoord(
      const double target_x, const double target_y, const double target_z, 
      const double shift_x , const double shift_y , const double shift_z,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveHandToTargetTF(
      const std::string &target_name,
      const double shift_x, const double shift_y, const double shift_z,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveHandToPlaceCoord(
      const double target_x, const double target_y, const double target_z, 
      const double shift_x , const double shift_y , const double shift_z,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveHandToPlaceTF(
      const std::string& target_name,
      const double shift_x, const double shift_y, const double shift_z,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveHeadToTargetCoord(
      const double target_x, const double target_y, const double target_z, 
      const double shift_x , const double shift_y , const double shift_z,
      const int32_t sec = 5, bool is_sleep = true);
  bool moveHeadToTargetTF(
      const std::string &target_name,
      const double shift_x, const double shift_y, const double shift_z,
      const int32_t sec = 5, bool is_sleep = true);
  bool graspDecision(const int min_curr = 300, const int max_curr = 1000);
  bool placeDecision(const int min_curr = 500, const int max_curr = 1000);

 private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_control_;

  rclcpp::Subscription<sobits_msgs::msg::current_state_array>::SharedPtr sub_arm_curr_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::string target_frame_;

  std::vector<Pose> kPoseList;

  const std::vector<std::string> kJointNames = {
      "arm_shoulder_roll_joint",
      "arm_shoulder_pitch_joint",
      // "arm_shoulder_1_pitch_joint",
      // "arm_shoulder_2_pitch_joint",
      "arm_elbow_pitch_joint",
      "arm_forearm_roll_joint",
      "arm_wrist_pitch_joint",
      "arm_wrist_roll_joint",
      "hand_joint",
      "head_yaw_joint",
      "head_pitch_joint"
  };

  double kArmWristPitchCurr = 0.;
  double kHandCurr = 0.;

  // TODO: obtain links lengths with TF
  const double kArmUpper   = 0.128;
  const double kArmUpperX  = 0.022;  // shift to the x-axis
  const double kArmLower   = 0.124;
  const double kArmGripper = 0.064 + 0.11225;
  const double kArmLength  = kArmUpper + kArmLower;
  // const double kGraspMinZ = 0.035;
  // const double kGraspMaxZ = 0.080;

  void setJointTrajectory(
      const std::string& joint_name,
      const double rad,
      const int32_t sec = 5,
      trajectory_msgs::msg::JointTrajectory* jt);
  void addJointTrajectory(
      const std::string& joint_name,
      const double rad,
      const int32_t sec = 5,
      trajectory_msgs::msg::JointTrajectory* jt);
  void checkPublishersConnection(const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& pub);
  void declarePoseParams(const std::string& prefix);
  void setPoseParams(
      const std::string& prefix,
      std::vector<Pose>& poses);
  void callbackArmCurr(const sobits_msgs::msg::current_state_array::SharedPtr msg);
  void loadPose();
};
}  // namespace sobit_light

inline void sobit_light::JointController::setJointTrajectory(
    const std::string& joint_name, 
    const double rad, 
    const int32_t sec = 5, 
    trajectory_msgs::msg::JointTrajectory* jt) {
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point; 

  joint_trajectory.joint_names.push_back(joint_name); 
  joint_trajectory_point.positions.push_back(rad);
  // joint_trajectory_point.velocities.push_back(0.0);
  // joint_trajectory_point.accelerations.push_back(0.0);
  // joint_trajectory_point.effort.push_back(0.0);
  joint_trajectory_point.time_from_start = rclcpp::Duration(sec, 0);
  joint_trajectory.points.push_back(joint_trajectory_point);

  *jt = joint_trajectory;
}

inline void sobit_light::JointController::addJointTrajectory(
    const std::string& joint_name, 
    const double rad, 
    const int32_t sec = 5, 
    trajectory_msgs::msg::JointTrajectory* jt) {
  trajectory_msgs::msg::JointTrajectory joint_trajectory = *jt;

  joint_trajectory.joint_names.push_back(joint_name); 
  joint_trajectory.points[0].positions.push_back(rad);
  // joint_trajectory.points[0].velocities.push_back(0.0);
  // joint_trajectory.points[0].accelerations.push_back(0.0);
  // joint_trajectory.points[0].effort.push_back(0.0);
  joint_trajectory.points[0].time_from_start = rclcpp::Duration(sec, 0);

  *jt = joint_trajectory;
}

// TODO: send publisher
inline void sobit_light::JointController::checkPublishersConnection(
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& pub) {
  while (this->count_subscribers("arm_trajectory_controller/command") == 0) {
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
}

inline void sobit_light::JointController::declarePoseParams(
    const std::string& prefix) {
  for (const auto& joint_name : kJointNames) {
    declare_parameter<double>(prefix + joint_name);
  }
}

inline void sobit_light::JointController::setPoseParams(
    const std::string& prefix,
    std::vector<Pose>& poses) {
  Pose pose;
  std::string param_name;
  double value;

  for (const auto& joint_name : kJointNames) {
    param_name = prefix + "." + joint_name;
    value = get_parameter(param_name).as_double();
    pose.pose_name.push_back(value);
  }
  pose.pose_name = prefix;
  poses.push_back(pose);
}


// TODO: obtain current from each actuator
inline void sobit_light::JointController::callbackArmCurr(
    const sobits_msgs::msg::current_state_array::SharedPtr msg) {
  for (const auto actuator : msg->current_state_array) {
    if (actuator->joint_name == kJointNames[kArmWristPitchJoint])
      kArmWristPitchCurr = actuator.current_ma;
    if (actuator->joint_name == kJointNames[kHandJoint])
      kHandCurr = actuator->current_ma;
  }
}

#endif  // SOBIT_LIGHT_JOINT_CONTROLLER_H_

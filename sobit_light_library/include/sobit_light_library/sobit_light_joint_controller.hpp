#ifndef SOBIT_LIGHT_JOINT_CONTROLLER_H_
#define SOBIT_LIGHT_JOINT_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

// #include "sobit_light_library/sobit_light_library.h"
// #include "sobits_msgs/current_state_array.h"
// #include "sobits_msgs/current_state.h"

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

class JointController : private ROSCommonNode {
 public:
  JointController(const std::string &name);
  // JointController();
  ~JointController();
  
  bool MoveToPose(
      const std::string &pose_name, 
      const double sec = 5.0, bool is_sleep = true);
  bool MoveAllJointsRad(
      const double arm_shoulder_roll,
      const double arm_shoulder_pitch,
      const double arm_elbow_pitch,
      const double arm_forearm_roll,
      const double arm_wrist_pitch,
      const double arm_wrist_roll,
      const double hand,
      const double head_yaw,
      const double head_pitch,
      const double sec = 5.0, bool is_sleep = true);
  bool MoveAllJointsDeg(
      const double arm_shoulder_roll,
      const double arm_shoulder_pitch,
      const double arm_elbow_pitch,
      const double arm_forearm_roll,
      const double arm_wrist_pitch,
      const double arm_wrist_roll,
      const double hand,
      const double head_yaw,
      const double head_pitch,
      const double sec = 5.0, bool is_sleep = true);
  bool moveJointRad(
      const Joint  joint_num, 
      const double rad, 
      const double sec = 5.0, bool is_sleep = true);
  bool moveJointDeg(
      const Joint  joint_num, 
      const double deg, 
      const double sec = 5.0, bool is_sleep = true);
  bool moveArmDeg(
      const double arm_shoulder_roll,
      const double arm_shoulder_pitch,
      const double arm_elbow_pitch,
      const double arm_forearm_roll,
      const double arm_wrist_pitch,
      const double arm_wrist_roll,
      const double sec = 5.0, bool is_sleep = true);
  bool moveArmRad(
      const double arm_shoulder_roll,
      const double arm_shoulder_pitch,
      const double arm_elbow_pitch,
      const double arm_forearm_roll,
      const double arm_wrist_pitch,
      const double arm_wrist_roll,
      const double sec = 5.0, bool is_sleep = true);
  bool moveHeadDeg(
      const double head_yaw,
      const double head_pitch,
      const double sec = 5.0, bool is_sleep = true);
  bool moveHeadRad(
      const double head_yaw,
      const double head_pitch,
      const double sec = 5.0, bool is_sleep = true);
  bool moveHandToTargetCoord(
      const double target_pos_x, const double target_pos_y, const double target_pos_z, 
      const double shift_x     , const double shift_y     , const double shift_z,
      const double sec = 5.0, bool is_sleep = true);
  bool moveHandToTargetTF(
      const std::string &target_name,
      const double shift_x, const double shift_y, const double shift_z,
      const double sec = 5.0, bool is_sleep = true);
  bool moveHandToPlaceCoord(
      const double target_pos_x, const double target_pos_y, const double target_pos_z, 
      const double shift_x     , const double shift_y     , const double shift_z,
      const double sec = 5.0, bool is_sleep = true);
  bool moveHandToPlaceTF(
      const std::string& target_name,
      const double shift_x, const double shift_y, const double shift_z,
      const double sec = 5.0, bool is_sleep = true);
  bool moveHeadToTargetCoord(
      const double target_pos_x, const double target_pos_y, const double target_pos_z, 
      const double shift_x     , const double shift_y     , const double shift_z,
      const double sec = 5.0, bool is_sleep = true);
  bool moveHeadToTargetTF(
      const std::string &target_name,
      const double shift_x, const double shift_y, const double shift_z,
      const double sec = 5.0, bool is_sleep = true);
  // bool graspDecision(const int min_curr = 300, const int max_curr = 1000);
  // bool placeDecision(const int min_curr = 500, const int max_curr = 1000);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher  pub_arm_control_;
  ros::Publisher  pub_head_camera_control_;  
  ros::Subscriber sub_curr_arm;

  tf2_ros::Buffer tfBuffer_;

  tf2_ros::TransformListener tfListener_;

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

  std::vector<Pose> pose_list_;

  // double arm_wrist_tilt_current_ = 0.;
  // double hand_current_ = 0.;

  // TODO: obtain links lengths with TF
  const double kArmUpper   = 0.128;
  const double kArmUpperX  = 0.022;  // shift to the x-axis
  const double kArmLower   = 0.124;
  const double kAemGripper = 0.064 + 0.11225;
  const double kArmLength  = kArmUpper + kArmLower;

  // static constexpr const double base_to_shoulder_flex_joint_z_cm = 52.2;
  // static constexpr const double base_to_shoulder_flex_joint_x_cm = 12.2;
  // static constexpr const double arm_upper_link_x_cm = 14.8;
  // static constexpr const double arm_upper_link_z_cm = 2.4;
  // static constexpr const double arm_outer_link_x_cm = 15.0;
  // static constexpr const double grasp_min_z_cm = 35.0;
  // static constexpr const double grasp_max_z_cm = 80.0;

  void setJointTrajectory(
      const std::string& joint_name,
      const double rad,
      const double sec,
      trajectory_msgs::JointTrajectory* jt);
  void addJointTrajectory(
      const std::string& joint_name,
      const double rad,
      const double sec,
      trajectory_msgs::JointTrajectory* jt);
  void checkPublishersConnection(const ros::Publisher& pub);
  void callbackCurrArm(const sobits_msgs::current_state_array& msg);
  void loadPose();
};
}  // namespace sobit_light

inline void sobit_light::JointController::setJointTrajectory(
    const std::string& joint_name, 
    const double rad, 
    const double sec, 
    trajectory_msgs::JointTrajectory* jt) {
  trajectory_msgs::JointTrajectory joint_trajectory;
  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point; 

  joint_trajectory.joint_names.push_back(joint_name); 
  joint_trajectory_point.positions.push_back(rad);
  // joint_trajectory_point.velocities.push_back(0.0);
  // joint_trajectory_point.accelerations.push_back(0.0);
  // joint_trajectory_point.effort.push_back(0.0);
  joint_trajectory_point.time_from_start = ros::Duration(sec);
  joint_trajectory.points.push_back(joint_trajectory_point);

  *jt = joint_trajectory;

  return;
}

inline void sobit_light::JointController::addJointTrajectory(
    const std::string& joint_name, 
    const double rad, 
    const double sec, 
    trajectory_msgs::msg::JointTrajectory* jt) {
  trajectory_msgs::msg::JointTrajectory joint_trajectory = *jt;

  joint_trajectory.joint_names.push_back(joint_name); 
  joint_trajectory.points[0].positions.push_back(rad);
  // joint_trajectory.points[0].velocities.push_back(0.0);
  // joint_trajectory.points[0].accelerations.push_back(0.0);
  // joint_trajectory.points[0].effort.push_back(0.0);
  joint_trajectory.points[0].time_from_start = ros::Duration(sec);

  *jt = joint_trajectory;

  return;
}

inline void sobit_light::JointController::checkPublishersConnection(
    const ros::Publisher& pub) {

  ros::Rate loop_rate(10);
  while (pub.getNumSubscribers() == 0 && ros::ok()) {
    try { loop_rate.sleep();
    } catch (const std::exception& ex) { break; }
  }
  return; 
}

// TODO: obtain current from each actuator
inline void sobit_light::JointController::callbackCurrArm(
    const sobits_msgs::current_state_array& msg) {
  // ros::spinOnce();

  for (const auto actuator : msg.current_state_array) {
    if (actuator.joint_name == joint_names_[ARM_WRIST_TILT_JOINT])
      arm_wrist_tilt_current_ = actuator.current_ma;
    if (actuator.joint_name == joint_names_[HAND_JOINT])
      hand_current_ = actuator.current_ma;
  }
}

#endif  // SOBIT_LIGHT_JOINT_CONTROLLER_H_

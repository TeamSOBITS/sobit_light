#include <map>

#include "sobits_interfaces/action/move_joint.hpp"
#include "sobits_interfaces/action/move_to_pose.hpp"
#include "sobits_interfaces/srv/get_hand_to_target_coord.hpp"
#include "sobits_interfaces/srv/get_hand_to_target_tf.hpp"
// #include "sobits_interfaces/action/move_hand_to_target_coord.hpp"
// #include "sobits_interfaces/action/move_hand_to_target_tf.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.h"
#include "geometry_msgs/msg/vector3.h"
#include "geometry_msgs/msg/point.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>


namespace sobit_light
{

struct PoseParams 
{
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

enum JointIds
{
  kArmShoulderRollJoint = 0,
  kArmShoulderPitchJoint,
  // kArmShoulderPitchSubJoint,
  kArmElbowPitchJoint,
  kArmForearmRollJoint,
  kArmWristPitchJoint,
  kArmWristRollJoint,
  kHandJoint,
  kHeadYawJoint,
  kHeadPitchJoint,
  kJointNum
};

class JointActionServer : public rclcpp::Node
{
public:
  using MoveJoint = sobits_interfaces::action::MoveJoint;
  using MoveToPose = sobits_interfaces::action::MoveToPose;
  using GetHandToTargetCoord = sobits_interfaces::srv::GetHandToTargetCoord;
  using GetHandToTargetTF = sobits_interfaces::srv::GetHandToTargetTF;
  // using MoveHandToTargetCoord = sobits_interfaces::action::MoveHandToTargetCoord;
  // using MoveHandToTargetTF = sobits_interfaces::action::MoveHandToTargetTF;


  using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveJoint>;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveToPose>;
  // using GoalHandleMoveHandToCoord = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveHandToTargetCoord>;
  // using GoalHandleMoveHandToTf = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveHandToTargetTF>;


  explicit JointActionServer(const rclcpp::NodeOptions & options);
  ~JointActionServer();

  geometry_msgs::msg::Vector3 get_euler_from_quat(
    const geometry_msgs::msg::Quaternion& quat);
  geometry_msgs::msg::Quaternion get_quat_from_euler(
    const geometry_msgs::msg::Vector3& rpy);
  geometry_msgs::msg::TransformStamped forward_kinematics(
    const std::vector<double> &target_joint_rad);
  bool inverse_kinematics(
    const geometry_msgs::msg::TransformStamped &goal_coord,
    std::vector<double> &target_joint_rad);
  trajectory_msgs::msg::JointTrajectory set_joints(
    const std::vector<std::string> &target_joint_names,
    const std::vector<double> &target_joint_rad,
    const builtin_interfaces::msg::Duration &time_allowance);

private:
  const std::vector<std::string> kJointNames = {
    "arm_shoulder_roll_joint",
    "arm_shoulder_pitch_joint",
    // "arm_shoulder_pitch_sub_joint",
    "arm_elbow_pitch_joint",
    "arm_forearm_roll_joint",
    "arm_wrist_pitch_joint",
    "arm_wrist_roll_joint",
    "hand_joint",
    "head_yaw_joint",
    "head_pitch_joint"
  };
  const std::vector<std::string> kArmJointNames = {
    "arm_shoulder_roll_joint",
    "arm_shoulder_pitch_joint",
    // "arm_shoulder_pitch_sub_joint",
    "arm_elbow_pitch_joint",
    "arm_forearm_roll_joint",
    "arm_wrist_pitch_joint",
    "arm_wrist_roll_joint",
  };

  static constexpr double kArmUpper   = 0.128;
  static constexpr double kArmLower   = 0.124;
  static constexpr double kArmGripper = 0.064 + 0.11225;
  static constexpr double kArmLength  = kArmUpper + kArmLower;

  static constexpr double kShoElbDiff = 0.022; // axis shift from shoulder to elbow

  std::vector<PoseParams> poses_;
  std::map<std::string, double> init_joint_state_;
  std::map<std::string, double> curt_joint_state_;

  rclcpp_action::Server<MoveJoint>::SharedPtr action_server_move_joints_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_move_to_pose_;
  rclcpp::Service<GetHandToTargetCoord>::SharedPtr service_get_hand_to_coord_;
  rclcpp::Service<GetHandToTargetTF>::SharedPtr service_get_hand_to_tf_;
  // rclcpp_action::Server<MoveHandToTargetCoord>::SharedPtr action_server_move_hand_to_coord_;
  // rclcpp_action::Server<MoveHandToTargetTF>::SharedPtr action_server_move_hand_to_tf_;

  rclcpp_action::GoalResponse handle_move_joints_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveJoint::Goal> goal);
  rclcpp_action::GoalResponse handle_move_to_pose_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal);
  // rclcpp_action::GoalResponse handle_move_hand_to_coord_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveHandToTargetCoord::Goal> goal);
  // rclcpp_action::GoalResponse handle_move_hand_to_tf_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveHandToTargetTF::Goal> goal);

  rclcpp_action::CancelResponse handle_move_joints_cancel(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  rclcpp_action::CancelResponse handle_move_to_pose_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  // rclcpp_action::CancelResponse handle_move_hand_to_coord_cancel(const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle);
  // rclcpp_action::CancelResponse handle_move_hand_to_tf_cancel(const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle);

  void handle_move_joints_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  void handle_move_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  // void handle_move_hand_to_coord_accepted(const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle);
  // void handle_move_hand_to_tf_accepted(const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle);

  void exe_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  void exe_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void get_pos_to_coord(const std::shared_ptr<GetHandToTargetCoord::Request> request, std::shared_ptr<GetHandToTargetCoord::Response> response);
  void get_pos_to_tf(const std::shared_ptr<GetHandToTargetTF::Request> request, std::shared_ptr<GetHandToTargetTF::Response> response);
  // void exe_move_hand_to_coord(const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle);
  // void exe_move_hand_to_tf(const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_control_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
}; // class JointActionServer

inline geometry_msgs::msg::Vector3 JointActionServer::get_euler_from_quat(
  const geometry_msgs::msg::Quaternion& msg_quat)
{
  tf2::Quaternion tf_quat;
  geometry_msgs::msg::Vector3 euler;

  tf2::fromMsg(msg_quat, tf_quat);
  tf_quat.normalize();
  tf2::Matrix3x3(tf_quat).getRPY(euler.x, euler.y, euler.z);

  return euler;  
}

inline geometry_msgs::msg::Quaternion JointActionServer::get_quat_from_euler(
  const geometry_msgs::msg::Vector3& euler)
{
  tf2::Quaternion tf_quat;

  tf_quat.setRPY(euler.x, euler.y, euler.z);

  return tf2::toMsg(tf_quat);
}

} // namespace sobit_light

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_light::JointActionServer)

// #ifndef SOBIT_LIGHT_JOINT_ACTION_SERVER_HPP
// #define SOBIT_LIGHT_JOINT_ACTION_SERVER_HPP

// #include <functional>
// #include <memory>
#include <thread>
#include <map>

#include "sobits_interfaces/action/move_joint.hpp"
#include "sobits_interfaces/action/move_to_pose.hpp"
#include "sobits_interfaces/action/move_hand_to_target_coord.hpp"
#include "sobits_interfaces/action/move_hand_to_target_tf.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// #include "rclcpp_components/register_node_macro.hpp"

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

enum JointIds {
  kArmShoulderRollJoint = 0,
  kArmShoulderPitchJoint,
  kArmShoulderPitchSubJoint,
  kArmElbowPitchJoint,
  kArmForearmRollJoint,
  kArmWristPitchJoint,
  kArmWristRollJoint,
  kHandJoint,
  kHeadYawJoint,
  kHeadPitchJoint,
  kJointNum
};

class JointCtrlLibrary : public rclcpp::Node{
public:
  using MoveJoint = sobits_interfaces::action::MoveJoint;
  using MoveToPose = sobits_interfaces::action::MoveToPose;
  using MoveHandToTargetCoord = sobits_interfaces::action::MoveHandToTargetCoord;
  using MoveHandToTargetTF = sobits_interfaces::action::MoveHandToTargetTF;

  using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveJoint>;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveToPose>;
  using GoalHandleMoveHandToCoord = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveHandToTargetCoord>;
  using GoalHandleMoveHandToTf = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveHandToTargetTF>;


  // JointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  JointCtrlLibrary();
  ~JointCtrlLibrary();

private:
  const std::vector<std::string> kJointNames = {
    "arm_shoulder_roll_joint",
    "arm_shoulder_pitch_joint",
    "arm_shoulder_pitch_sub_joint",
    "arm_elbow_pitch_joint",
    "arm_forearm_roll_joint",
    "arm_wrist_pitch_joint",
    "arm_wrist_roll_joint",
    "hand_joint",
    "head_yaw_joint",
    "head_pitch_joint"
  };

  std::vector<PoseParams> poses_;
  std::map<std::string, double> init_joint_state_;
  std::map<std::string, double> curt_joint_state_;


  rclcpp_action::Server<MoveJoint>::SharedPtr action_server_move_joint_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_move_to_pose_;
  rclcpp_action::Server<MoveHandToTargetCoord>::SharedPtr action_server_move_hand_to_coord_;
  rclcpp_action::Server<MoveHandToTargetTF>::SharedPtr action_server_move_hand_to_tf_;

  rclcpp_action::GoalResponse handle_move_joint_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveJoint::Goal> goal);
  rclcpp_action::GoalResponse handle_move_to_pose_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal);
  rclcpp_action::GoalResponse handle_move_hand_to_coord_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveHandToTargetCoord::Goal> goal);
  rclcpp_action::GoalResponse handle_move_hand_to_tf_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveHandToTargetTF::Goal> goal);

  rclcpp_action::CancelResponse handle_move_joint_cancel(const std::shared_ptr<GoalHandleMoveJoint> goal_handle);
  rclcpp_action::CancelResponse handle_move_to_pose_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  rclcpp_action::CancelResponse handle_move_hand_to_coord_cancel(const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle);
  rclcpp_action::CancelResponse handle_move_hand_to_tf_cancel(const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle);

  void handle_move_joint_accepted(const std::shared_ptr<GoalHandleMoveJoint> goal_handle);
  void handle_move_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void handle_move_hand_to_coord_accepted(const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle);
  void handle_move_hand_to_tf_accepted(const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle);

  void exe_move_joint(const std::shared_ptr<GoalHandleMoveJoint> goal_handle);
  void exe_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void exe_move_hand_to_coord(const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle);
  void exe_move_hand_to_tf(const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle);


  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_control_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

} // namespace sobit_light

// #endif // SOBIT_LIGHT_JOINT_ACTION_SERVER_HPP

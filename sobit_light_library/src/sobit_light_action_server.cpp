#include "sobit_light_library/sobit_light_action_server.hpp"

namespace sobit_light{

JointCtrlLibrary::JointCtrlLibrary() : Node("joint_ctrl_library") {
  // Configure the QoS profile
  rclcpp::QoS qos_profile(1); // depth = 1
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  this->action_server_move_joint_ = rclcpp_action::create_server<MoveJoint>(
      this,
      "move_joint",
      std::bind(&JointCtrlLibrary::handle_move_joint_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointCtrlLibrary::handle_move_joint_cancel, this, std::placeholders::_1),
      std::bind(&JointCtrlLibrary::handle_move_joint_accepted, this, std::placeholders::_1));
  this->action_server_move_to_pose_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_to_pose",
      std::bind(&JointCtrlLibrary::handle_move_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointCtrlLibrary::handle_move_to_pose_cancel, this, std::placeholders::_1),
      std::bind(&JointCtrlLibrary::handle_move_to_pose_accepted, this, std::placeholders::_1));
  this->action_server_move_hand_to_coord_ = rclcpp_action::create_server<MoveHandToTargetCoord>(
      this,
      "move_hand_to_coord",
      std::bind(&JointCtrlLibrary::handle_move_hand_to_coord_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointCtrlLibrary::handle_move_hand_to_coord_cancel, this, std::placeholders::_1),
      std::bind(&JointCtrlLibrary::handle_move_hand_to_coord_accepted, this, std::placeholders::_1));
  this->action_server_move_hand_to_tf_ = rclcpp_action::create_server<MoveHandToTargetTF>(
      this,
      "move_hand_to_tf",
      std::bind(&JointCtrlLibrary::handle_move_hand_to_tf_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointCtrlLibrary::handle_move_hand_to_tf_cancel, this, std::placeholders::_1),
      std::bind(&JointCtrlLibrary::handle_move_hand_to_tf_accepted, this, std::placeholders::_1));

  this->sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile, std::bind(&JointCtrlLibrary::joint_state_callback, this, std::placeholders::_1));
  this->pub_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "joint_trajectory_controller/joint_trajectory", qos_profile);

  RCLCPP_INFO(this->get_logger(), "JointCtrlLibrary has been initialized.");
}
JointCtrlLibrary::~JointCtrlLibrary() {
  this->action_server_move_joint_.reset();
  this->action_server_move_to_pose_.reset();
  this->action_server_move_hand_to_coord_.reset();
  this->action_server_move_hand_to_tf_.reset();

  this->sub_joint_state_.reset();
  this->pub_joint_control_.reset();

  RCLCPP_INFO(this->get_logger(), "JointCtrlLibrary has been terminated.");
}


rclcpp_action::GoalResponse JointCtrlLibrary::handle_move_joint_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJoint::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::GoalResponse JointCtrlLibrary::handle_move_to_pose_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::GoalResponse JointCtrlLibrary::handle_move_hand_to_coord_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveHandToTargetCoord::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::GoalResponse JointCtrlLibrary::handle_move_hand_to_tf_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveHandToTargetTF::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse JointCtrlLibrary::handle_move_joint_cancel(
    const std::shared_ptr<GoalHandleMoveJoint> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse JointCtrlLibrary::handle_move_to_pose_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse JointCtrlLibrary::handle_move_hand_to_coord_cancel(
    const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse JointCtrlLibrary::handle_move_hand_to_tf_cancel(
    const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void JointCtrlLibrary::handle_move_joint_accepted(
    const std::shared_ptr<GoalHandleMoveJoint> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointCtrlLibrary::exe_move_joint, this, std::placeholders::_1), goal_handle}.detach();
}
void JointCtrlLibrary::handle_move_to_pose_accepted(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointCtrlLibrary::exe_move_to_pose, this, std::placeholders::_1), goal_handle}.detach();
}
void JointCtrlLibrary::handle_move_hand_to_coord_accepted(
    const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointCtrlLibrary::exe_move_hand_to_coord, this, std::placeholders::_1), goal_handle}.detach();
}
void JointCtrlLibrary::handle_move_hand_to_tf_accepted(
    const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointCtrlLibrary::exe_move_hand_to_tf, this, std::placeholders::_1), goal_handle}.detach();
}


void JointCtrlLibrary::exe_move_joint(
    const std::shared_ptr<GoalHandleMoveJoint> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();

  // Check if the number of joint names and joint rad are the same
  if (goal->target_joint_names.size() != goal->target_joint_rad.size()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal request");
    auto result = std::make_shared<MoveJoint::Result>();
    goal_handle->abort(result);
    return;
  }

  // Check if the joint names are valid
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    if (std::find(kJointNames.begin(), kJointNames.end(), goal->target_joint_names[i]) == kJointNames.end()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid joint name: %s", goal->target_joint_names[i].c_str());
      auto result = std::make_shared<MoveJoint::Result>();
      goal_handle->abort(result);
      return;
    }
  }

  // TODO: Check if the joint rad are within the joint limits

  // Publish the joint trajectory
  auto joint_trajectory = trajectory_msgs::msg::JointTrajectory();
  joint_trajectory.header.stamp = this->now();
  joint_trajectory.joint_names = goal->target_joint_names;
  joint_trajectory.points.resize(1);
  joint_trajectory.points[0].time_from_start = goal->time_allowance;
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    joint_trajectory.points[0].positions.push_back(goal->target_joint_rad[i]);
  }
  this->pub_joint_control_->publish(joint_trajectory);

  auto start_time = this->now();
  rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");
      auto result = std::make_shared<MoveJoint::Result>();
      goal_handle->canceled(result);
      return;
    }

    auto feedback = std::make_shared<MoveJoint::Feedback>();
    feedback->current_joint_names = goal->target_joint_names;
    for (const auto &joint_name : goal->target_joint_names) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();

  }

  auto result = std::make_shared<MoveJoint::Result>();
  result->message = "Goal has been succeeded";
  result->success = true;
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

// TODO: Implement the following functions
void JointCtrlLibrary::exe_move_to_pose(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();

  auto result = std::make_shared<MoveToPose::Result>();
  goal_handle->succeed(result);
}
void JointCtrlLibrary::exe_move_hand_to_coord(
    const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();

  auto result = std::make_shared<MoveHandToTargetCoord::Result>();
  goal_handle->succeed(result);
}
void JointCtrlLibrary::exe_move_hand_to_tf(
    const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();

  auto result = std::make_shared<MoveHandToTargetTF::Result>();
  goal_handle->succeed(result);
}


void JointCtrlLibrary::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received joint state");

  for (size_t i = 0; i < msg->name.size(); i++) {
    this->curt_joint_state_[msg->name[i]] = msg->position[i];
  }

  RCLCPP_INFO(this->get_logger(), "Current joint state:");
  for (const auto &joint : this->curt_joint_state_) {
    RCLCPP_INFO(this->get_logger(), "  %s: %f", joint.first.c_str(), joint.second);
  }
}

} // namespace sobit_light



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sobit_light::JointCtrlLibrary>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

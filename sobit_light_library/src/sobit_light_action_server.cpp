#include "sobit_light_library/sobit_light_action_server.hpp"

namespace sobit_light{

JointCtrlLibrary::JointCtrlLibrary() :
    Node("joint_ctrl_library"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
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


  //Declare the pose parameters

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

  // TODO: Create a common funtion to publish the joint trajectory
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

  // Check if the pose name is valid
  if (std::find_if(poses_.begin(), poses_.end(), [&](const PoseParams &pose) { return pose.pose_name == goal->pose_name; }) == poses_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pose name: %s", goal->pose_name.c_str());
    auto result = std::make_shared<MoveToPose::Result>();
    goal_handle->abort(result);
    return;
  }

  // Get the target joint rad from the pose name
  std::vector<double> target_joint_rad;
  for (const auto &pose : poses_) {
    if (pose.pose_name == goal->pose_name) {
      target_joint_rad.push_back(pose.arm_shoulder_roll);
      target_joint_rad.push_back(pose.arm_shoulder_pitch);
      target_joint_rad.push_back(pose.arm_elbow_pitch);
      target_joint_rad.push_back(pose.arm_forearm_roll);
      target_joint_rad.push_back(pose.arm_wrist_pitch);
      target_joint_rad.push_back(pose.arm_wrist_roll);
      target_joint_rad.push_back(pose.hand);
      target_joint_rad.push_back(pose.head_yaw);
      target_joint_rad.push_back(pose.head_pitch);
      break;
    }
  }

  // TODO: Create a common funtion to publish the joint trajectory
  // Publish the joint trajectory
  auto joint_trajectory = trajectory_msgs::msg::JointTrajectory();
  joint_trajectory.header.stamp = this->now();
  joint_trajectory.joint_names = kJointNames;
  joint_trajectory.points.resize(1);
  joint_trajectory.points[0].time_from_start = goal->time_allowance;
  for (size_t i = 0; i < kJointNames.size(); i++) {
    joint_trajectory.points[0].positions.push_back(target_joint_rad[i]);
  }

  this->pub_joint_control_->publish(joint_trajectory);

  auto start_time = this->now();
  rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");
      auto result = std::make_shared<MoveToPose::Result>();
      goal_handle->canceled(result);
      return;
    }

    auto feedback = std::make_shared<MoveToPose::Feedback>();
    feedback->current_joint_names = kJointNames;
    for (const auto &joint_name : kJointNames) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  auto result = std::make_shared<MoveToPose::Result>();
  result->message = "Goal has been succeeded";
  result->success = true;
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointCtrlLibrary::exe_move_hand_to_coord(
    const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();

  geometry_msgs::msg::PoseStamped goal_coord;
  goal_coord.header = goal->target_coord.header;
  goal_coord.header.frame_id = this->get_name() + std::string("/base_footprint");

  try{
    goal_coord = tf_buffer_->transform(goal->target_coord, goal_coord.header.frame_id, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    auto result = std::make_shared<MoveHandToTargetCoord::Result>();
    result->message = "[FAIL] Not Find of Transform " + goal->target_coord.header.frame_id + " and " + goal_coord.header.frame_id;
    result->success = false;
    goal_handle->canceled(result);
    RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
    return;
  }

  // TODO: Inverse kinematics to get the target joint rad

  // TODO: Create a common funtion to publish the joint trajectory

  auto result = std::make_shared<MoveHandToTargetCoord::Result>();
  result->message = "[SUCCESS] Move Hand to Coord has finished";
  result->success = true;
  goal_handle->succeed(result);
}

void JointCtrlLibrary::exe_move_hand_to_tf(
    const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();

  geometry_msgs::msg::PoseStamped goal_coord;
  goal_coord.header = goal->tf_differential.header;
  goal_coord.header.frame_id = this->get_name() + std::string("/base_footprint");

  geometry_msgs::msg::TransformStamped target_coord;
  geometry_msgs::msg::TransformStamped target_coord_mid;
  geometry_msgs::msg::TransformStamped target_coord_last;

  auto result = std::make_shared<MoveHandToTargetTF::Result>();

  // Transform the target frame based on the differential tf
  try {
    target_coord = getTransformName2Name(
      goal->target_frame, goal->tf_differential.header.frame_id);

    geometry_msgs::msg::Vector3 euler_target, euler_shift, euler_target_final;
    euler_target = getEulerFromQuat(target_coord.transform.rotation);
    euler_shift = getEulerFromQuat(goal->tf_differential.pose.orientation);

    euler_target_final.x = euler_target.x + euler_shift.x;
    euler_target_final.y = euler_target.y + euler_shift.y;
    euler_target_final.z = euler_target.z + euler_shift.z;

    target_coord_mid = target_coord;
    target_coord_mid.transform.translation.x += goal->tf_differential.pose.position.x;
    target_coord_mid.transform.translation.y += goal->tf_differential.pose.position.y;
    target_coord_mid.transform.translation.z += goal->tf_differential.pose.position.z;
    target_coord_mid.transform.rotation = getQuatFromEuler(euler_target_final);
  } catch (const tf2::TransformException &ex) {
    result->success = false;
    result->message = "[FAIL] Not Find of Transform " + goal->target_frame + " and " + goal->tf_differential.header.frame_id;
    goal_handle->canceled(result);
    RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
    return;
  }

  // Transform the target frame based on the robot base frame
  try {
    target_coord_last = getTransformCoord2Name(
      target_coord_mid, goal_coord.header.frame_id);

    goal_coord.pose.position.x = target_coord_last.transform.translation.x;
    goal_coord.pose.position.y = target_coord_last.transform.translation.y;
    goal_coord.pose.position.z = target_coord_last.transform.translation.z;
    goal_coord.pose.orientation.x = target_coord_last.transform.rotation.x;
    goal_coord.pose.orientation.y = target_coord_last.transform.rotation.y;
    goal_coord.pose.orientation.z = target_coord_last.transform.rotation.z;
  } catch (const tf2::TransformException &ex) {
    result->success = false;
    result->message = "[FAIL] Not Find of Transform " + goal->target_frame + " and " + goal->tf_differential.header.frame_id;
    goal_handle->canceled(result);
    RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
    return;
  }

  // TODO: Inverse kinematics to get the target joint rad

  // TODO: Create a common funtion to publish the joint trajectory


  // Set the result
  result->success = true;
  result->message = "Goal has been succeeded";

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

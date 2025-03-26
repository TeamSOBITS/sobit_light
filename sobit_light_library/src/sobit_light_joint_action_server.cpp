#include "sobit_light_library/sobit_light_joint_action_server.hpp"

namespace sobit_light{

JointActionServer::JointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("joint_action_server", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  // Configure the QoS profile
  rclcpp::QoS qos_profile(1); // depth = 1
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  this->action_server_move_joints_ = rclcpp_action::create_server<MoveJoint>(
      this,
      "move_joint",
      std::bind(&JointActionServer::handle_move_joints_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointActionServer::handle_move_joints_cancel, this, std::placeholders::_1),
      std::bind(&JointActionServer::handle_move_joints_accepted, this, std::placeholders::_1));
  this->action_server_move_to_pose_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_to_pose",
      std::bind(&JointActionServer::handle_move_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointActionServer::handle_move_to_pose_cancel, this, std::placeholders::_1),
      std::bind(&JointActionServer::handle_move_to_pose_accepted, this, std::placeholders::_1));
  this->service_move_hand_to_coord_ = this->create_service<MoveHandToTargetCoord>(
      "move_hand_to_coord",
      std::bind(&JointActionServer::get_pos_to_coord, this, std::placeholders::_1, std::placeholders::_2));
  this->service_move_hand_to_tf_ = this->create_service<MoveHandToTargetTF>(
      "move_hand_to_tf",
      std::bind(&JointActionServer::get_pos_to_tf, this, std::placeholders::_1, std::placeholders::_2));
  // this->action_server_move_hand_to_coord_ = rclcpp_action::create_server<MoveHandToTargetCoord>(
  //     this,
  //     "move_hand_to_coord",
  //     std::bind(&JointActionServer::handle_move_hand_to_coord_goal, this, std::placeholders::_1, std::placeholders::_2),
  //     std::bind(&JointActionServer::handle_move_hand_to_coord_cancel, this, std::placeholders::_1),
  //     std::bind(&JointActionServer::handle_move_hand_to_coord_accepted, this, std::placeholders::_1));
  // this->action_server_move_hand_to_tf_ = rclcpp_action::create_server<MoveHandToTargetTF>(
  //     this,
  //     "move_hand_to_tf",
  //     std::bind(&JointActionServer::handle_move_hand_to_tf_goal, this, std::placeholders::_1, std::placeholders::_2),
  //     std::bind(&JointActionServer::handle_move_hand_to_tf_cancel, this, std::placeholders::_1),
  //     std::bind(&JointActionServer::handle_move_hand_to_tf_accepted, this, std::placeholders::_1));

  this->sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile, std::bind(&JointActionServer::joint_state_callback, this, std::placeholders::_1));
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

  RCLCPP_INFO(this->get_logger(), "JointActionServer has been initialized.");
}
JointActionServer::~JointActionServer()
{
  this->action_server_move_joints_.reset();
  this->action_server_move_to_pose_.reset();
  // this->action_server_move_hand_to_coord_.reset();
  // this->action_server_move_hand_to_tf_.reset();

  this->sub_joint_state_.reset();
  this->pub_joint_control_.reset();

  RCLCPP_INFO(this->get_logger(), "JointActionServer has been terminated.");
}


rclcpp_action::GoalResponse JointActionServer::handle_move_joints_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveJoint::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::GoalResponse JointActionServer::handle_move_to_pose_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
// rclcpp_action::GoalResponse JointActionServer::handle_move_hand_to_coord_goal(
//   const rclcpp_action::GoalUUID & uuid,
//   std::shared_ptr<const MoveHandToTargetCoord::Goal> goal)
// {
//   RCLCPP_INFO(this->get_logger(), "Received goal request");
//   (void)uuid;
//   (void)goal;
//   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
// }
// rclcpp_action::GoalResponse JointActionServer::handle_move_hand_to_tf_goal(
//   const rclcpp_action::GoalUUID & uuid,
//   std::shared_ptr<const MoveHandToTargetTF::Goal> goal)
// {
//   RCLCPP_INFO(this->get_logger(), "Received goal request");
//   (void)uuid;
//   (void)goal;
//   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
// }


rclcpp_action::CancelResponse JointActionServer::handle_move_joints_cancel(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse JointActionServer::handle_move_to_pose_cancel(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
// rclcpp_action::CancelResponse JointActionServer::handle_move_hand_to_coord_cancel(
//   const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle)
// {
//   RCLCPP_INFO(this->get_logger(), "Received cancel request");
//   (void)goal_handle;
//   return rclcpp_action::CancelResponse::ACCEPT;
// }
// rclcpp_action::CancelResponse JointActionServer::handle_move_hand_to_tf_cancel(
//   const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle)
// {
//   RCLCPP_INFO(this->get_logger(), "Received cancel request");
//   (void)goal_handle;
//   return rclcpp_action::CancelResponse::ACCEPT;
// }


void JointActionServer::handle_move_joints_accepted(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointActionServer::exe_move_joints, this, std::placeholders::_1), goal_handle}.detach();
}
void JointActionServer::handle_move_to_pose_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointActionServer::exe_move_to_pose, this, std::placeholders::_1), goal_handle}.detach();
}
// void JointActionServer::handle_move_hand_to_coord_accepted(
//   const std::shared_ptr<GoalHandleMoveHandToCoord> goal_handle)
// {
//   RCLCPP_INFO(this->get_logger(), "Received goal request");
//   (void)goal_handle;
//   std::thread{std::bind(&JointActionServer::exe_move_hand_to_coord, this, std::placeholders::_1), goal_handle}.detach();
// }
// void JointActionServer::handle_move_hand_to_tf_accepted(
//   const std::shared_ptr<GoalHandleMoveHandToTf> goal_handle)
// {
//   RCLCPP_INFO(this->get_logger(), "Received goal request");
//   (void)goal_handle;
//   std::thread{std::bind(&JointActionServer::exe_move_hand_to_tf, this, std::placeholders::_1), goal_handle}.detach();
// }


void JointActionServer::exe_move_joints(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveJoint::Result>();

  // Spin while waiting for the joint state to be updated
  // while (this->curt_joint_state_.empty()) {
  //   RCLCPP_INFO(this->get_logger(), "Waiting for the joint state to be updated");
  //   rclcpp::spin_some(this->get_node_base_interface());
  // }

  // Check if the number of joint names and joint rad are the same
  if (goal->target_joint_names.size() != goal->target_joint_rad.size()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal request. The number of joint names and joint rad are different");
    auto result = std::make_shared<MoveJoint::Result>();
    goal_handle->abort(result);
    return;
  }

  // Check if the joint names are valid
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    if (std::find(kJointNames.begin(), kJointNames.end(), goal->target_joint_names[i]) == kJointNames.end()) {
      RCLCPP_ERROR(this->get_logger(), "The joint name does not exist: %s", goal->target_joint_names[i].c_str());
      auto result = std::make_shared<MoveJoint::Result>();
      goal_handle->abort(result);
      return;
    }
  }

  // TODO: Check if the joint rad are within the joint limits

  // Publish the joint trajectory
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance);

  try {
    this->pub_joint_control_->publish(joint_trajectory);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish the joint trajectory: %s", ex.what());

    result->success = false;
    result->message = "[FAIL] Failed to publish the joint trajectory";
    goal_handle->abort(result);

    return;
  }

  // Publish feedback
  auto start_time = this->now();
  // rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
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

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();

  }

  // Check if goal was reached
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[goal->target_joint_names[i]] - goal->target_joint_rad[i]) > 0.1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal");

      result->success = false;
      result->message = "[FAIL] Failed to reach the goal";
      goal_handle->abort(result);

      return;
    }
  }

  // Clear the current joint state
  curt_joint_state_.clear();

  // Publish the result
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointActionServer::exe_move_to_pose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();

  // Spin while waiting for the joint state to be updated
  // while (this->curt_joint_state_.empty()) {
  //   RCLCPP_INFO(this->get_logger(), "Waiting for the joint state to be updated");
  //   rclcpp::spin_some(this->get_node_base_interface());
  // }

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
      // target_joint_rad.push_back(-pose.arm_shoulder_pitch);
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

  // Publish the joint trajectory
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory = set_joints(kJointNames, target_joint_rad, goal->time_allowance);

  try {
    this->pub_joint_control_->publish(joint_trajectory);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish the joint trajectory: %s", ex.what());

    result->success = false;
    result->message = "[FAIL] Failed to publish the joint trajectory";
    goal_handle->abort(result);

    return;
  }

  // Publish feedback
  auto start_time = this->now();
  rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
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

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();
  }

  // Check if goal was reached
  for (size_t i = 0; i < kJointNames.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[kJointNames[i]] - target_joint_rad[i]) > 0.1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal");

      result->success = false;
      result->message = "[FAIL] Failed to reach the goal";
      goal_handle->abort(result);

      return;
    }
  }

  // Clear the current joint state
  curt_joint_state_.clear();

  // Publish the result
  result->message = "[SUCCESS] Goal has been succeeded";
  result->success = true;
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointActionServer::get_pos_to_coord(
  const std::shared_ptr<MoveHandToTargetCoord::Request> request,
  std::shared_ptr<MoveHandToTargetCoord::Response>      response)
{
  RCLCPP_INFO(this->get_logger(), "Requesting service");

  // Get namespace and delete the first slash
  std::string ns = this->get_namespace();
  ns.erase(0, 1);
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->target_coord.header;
  goal_coord.header.frame_id = ns + std::string("/base_footprint");

  // Spin while waiting for the joint state to be updated
  // while (this->curt_joint_state_.empty()) {
  //   RCLCPP_INFO(this->get_logger(), "Waiting for the joint state to be updated");
  //   rclcpp::spin_some(this->get_node_base_interface());
  // }

  try{
    goal_coord = tf_buffer_->transform(
      request->target_coord, goal_coord.header.frame_id,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
  
    return;
  }

  // Inverse kinematics to get the target joint rad
  bool is_success = false;
  std::vector<double> target_joint_rad = {
    0.0,  // arm_shoulder_roll_joint_rad
    0.0,  // arm_shoulder_pitch_joint_rad
    // 0.0,  // arm_shoulder_pitch_sub_joint_rad
    0.0,  // arm_elbow_pitch_joint_rad
    0.0,  // arm_forearm_roll_joint_rad
    0.0,  // arm_wrist_pitch_joint_rad
    0.0,  // arm_wrist_roll_joint_rad
  };

  is_success = inverse_kinematics(goal_coord, target_joint_rad);

  if (!is_success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to calculate the inverse kinematics");

    response->success = false;
    response->message = "[FAIL] Failed to calculate the inverse kinematics";

    return;
  }

  // TODO: Check values with forward kinematics
  // geometry_msgs::msg::TransformStamped goal_coord_check;
  // do {
  //   target_joint_rad = inverse_kinematics(goal_coord, target_joint_rad);
  //   goal_coord_check = forward_kinematics(target_joint_rad, goal_coord);
  // } while (goal_coord_check != goal_coord);

  // Set the response
  response->target_joint_names = kArmJointNames;
  response->target_joint_rad = target_joint_rad;
  response->success = true;
  response->message = "[SUCCESS] Successfully calculated the target joint rad";

}

void JointActionServer::get_pos_to_tf(
  const std::shared_ptr<MoveHandToTargetTF::Request> request,
  std::shared_ptr<MoveHandToTargetTF::Response>      response)
{
  RCLCPP_INFO(this->get_logger(), "Requesting service");

  // Get namespace and delete the first slash
  std::string ns = this->get_namespace();
  ns.erase(0, 1);
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->tf_differential.header;
  goal_coord.header.frame_id = ns + std::string("/base_footprint");

  geometry_msgs::msg::TransformStamped goal_coord_shift;

  // Spin while waiting for the joint state to be updated
  // while (this->curt_joint_state_.empty()) {
  //   RCLCPP_INFO(this->get_logger(), "Waiting for the joint state to be updated");
  //   rclcpp::spin_some(this->get_node_base_interface());
  // }

  // Transform the target frame based on the differential tf
  try {
    goal_coord_shift = tf_buffer_->lookupTransform(
      request->target_frame, request->tf_differential.header.frame_id,
      tf2::TimePointZero);

    geometry_msgs::msg::Vector3 euler_target, euler_shift;
    euler_target = get_euler_from_quat(goal_coord_shift.transform.rotation);
    euler_shift = get_euler_from_quat(request->tf_differential.transform.rotation);
    euler_target.x += euler_shift.x;
    euler_target.y += euler_shift.y;
    euler_target.z += euler_shift.z;

    goal_coord_shift.transform.translation.x += request->tf_differential.transform.translation.x;
    goal_coord_shift.transform.translation.y += request->tf_differential.transform.translation.y;
    goal_coord_shift.transform.translation.z += request->tf_differential.transform.translation.z;
    goal_coord_shift.transform.rotation = get_quat_from_euler(euler_target);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
    request->target_frame.c_str(), request->tf_differential.header.frame_id.c_str(),
    ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform " + request->target_frame + " to " + request->tf_differential.header.frame_id;
  
    return;
  }

  // Transform the target frame based on the robot base frame
  try {
    goal_coord = tf_buffer_->transform(
      goal_coord_shift, goal_coord.header.frame_id,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform coords to %s: %s",
    goal_coord.header.frame_id.c_str(), ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;

    return;
  }

  // Inverse kinematics to get the target joint rad1
  bool is_success = false;
  std::vector<double> target_joint_rad = {
    0.0,  // arm_shoulder_roll_joint_rad
    0.0,  // arm_shoulder_pitch_joint_rad
    // 0.0,  // arm_shoulder_pitch_sub_joint_rad
    0.0,  // arm_elbow_pitch_joint_rad
    0.0,  // arm_forearm_roll_joint_rad
    0.0,  // arm_wrist_pitch_joint_rad
    0.0,  // arm_wrist_roll_joint_rad
  };

  is_success = inverse_kinematics(goal_coord, target_joint_rad);

  if (!is_success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to calculate the inverse kinematics");

    response->success = false;
    response->message = "[FAIL] Failed to calculate the inverse kinematics";

    return;
  }

  // TODO: Check values with forward kinematics
  // geometry_msgs::msg::TransformStamped goal_coord_check;
  // do {
  //   target_joint_rad = inverse_kinematics(goal_coord, target_joint_rad);
  //   goal_coord_check = forward_kinematics(target_joint_rad, goal_coord);
  // } while (goal_coord_check != goal_coord);

  // Set the response
  response->target_joint_names = kArmJointNames;
  response->target_joint_rad = target_joint_rad;
  response->success = true;
  response->message = "[SUCCESS] Successfully calculated the target joint rad";

}


void JointActionServer::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received joint state");

  for (size_t i = 0; i < msg->name.size(); i++) {
    // Skip sub joints
    if (msg->name[i] == "arm_shoulder_pitch_sub_joint") {
      continue;
    }
    this->curt_joint_state_[msg->name[i]] = msg->position[i];
  }

  // RCLCPP_INFO(this->get_logger(), "Current joint state:");
  // for (const auto &joint : this->curt_joint_state_) {
  //   RCLCPP_INFO(this->get_logger(), "  %s: %f", joint.first.c_str(), joint.second);
  // }
}

trajectory_msgs::msg::JointTrajectory JointActionServer::set_joints(
  const std::vector<std::string> &target_joint_names,
  const std::vector<double> &target_joint_rad,
  const builtin_interfaces::msg::Duration &time_allowance)
{
  // Get current joint state from kCurrentJointState
  std::vector<double> full_target_joint_rad;
  for (size_t i = 0; i < kJointNames.size(); i++) {
    full_target_joint_rad.push_back(this->curt_joint_state_[kJointNames[i]]);
  }
  
  // Update the target joint rad
  for (size_t i = 0; i < target_joint_names.size(); i++) {
    auto it = std::find(kJointNames.begin(), kJointNames.end(), target_joint_names[i]);
    full_target_joint_rad[std::distance(kJointNames.begin(), it)] = target_joint_rad[i];
  }

  auto joint_trajectory = trajectory_msgs::msg::JointTrajectory();
  joint_trajectory.header.stamp = this->now();
  joint_trajectory.points.resize(1);
  joint_trajectory.points[0].time_from_start = time_allowance;
  for (size_t i = 0; i < kJointNames.size(); i++) {
    joint_trajectory.points[0].positions.push_back(full_target_joint_rad[i]);
    joint_trajectory.joint_names.push_back(kJointNames[i]);

    // Add sub joints
    if (joint_trajectory.joint_names[i] == kJointNames[JointIds::kArmShoulderPitchJoint]) {
      joint_trajectory.points[0].positions.push_back(-full_target_joint_rad[i]);
      joint_trajectory.joint_names.push_back("arm_shoulder_pitch_sub_joint");
    }
  }

  return joint_trajectory;
}

bool JointActionServer::forward_kinematics(
  const std::vector<double> &target_joint_rad,
  const geometry_msgs::msg::TransformStamped &goal_coord,
  double &distance)
{
  bool is_success = false;
  geometry_msgs::msg::TransformStamped final_coord;

  final_coord.transform.translation.x = kArmUpper * std::cos(
      target_joint_rad[JointIds::kArmShoulderPitchJoint]);
  final_coord.transform.translation.x += kArmLower * std::cos(
      target_joint_rad[JointIds::kArmShoulderPitchJoint] + 
      target_joint_rad[JointIds::kArmElbowPitchJoint] + M_PI_2);
  final_coord.transform.translation.x += kArmGripper * std::cos(
      target_joint_rad[JointIds::kArmShoulderPitchJoint] +
      target_joint_rad[JointIds::kArmElbowPitchJoint] + M_PI_2 +
      target_joint_rad[JointIds::kArmWristPitchJoint]);

  final_coord.transform.rotation.z = std::atan2(
      goal_coord.transform.translation.y,
      goal_coord.transform.translation.x);

  // Check the distance between the target and final coords
  distance = std::sqrt(
      std::pow(goal_coord.transform.translation.x - final_coord.transform.translation.x, 2) +
      std::pow(goal_coord.transform.translation.y - final_coord.transform.translation.y, 2) +
      std::pow(goal_coord.transform.translation.z - final_coord.transform.translation.z, 2));
  
  // TODO: Update the distance threshold as parameter
  is_success = (distance < 0.01);

  return is_success;
}

bool JointActionServer::inverse_kinematics(
  const geometry_msgs::msg::TransformStamped &goal_coord,
  std::vector<double> &target_joint_rad)
{
  bool is_success = false;
  double goal_position_pos_z = goal_coord.transform.translation.z;
  if (goal_position_pos_z == 0) goal_position_pos_z = 0.03;

  if (goal_position_pos_z > kArmLength) {
    RCLCPP_WARN(this->get_logger(), "The target position is too tall (max:%f[m] < %f[m])", kArmLength, goal_position_pos_z);
    return is_success;
  }
  
  else if (goal_position_pos_z < -(kArmLower + kArmGripper)) {
    RCLCPP_WARN(this->get_logger(), "The target position is too low (%f[m] < min:%f[m])", goal_position_pos_z, -(kArmLower + kArmGripper));
    return is_success;
  }

  // Target is above arm_elbow_pitch_join
  if (0 <= goal_position_pos_z) {
    RCLCPP_INFO(this->get_logger(), "The target position (z:%f[m]) is above arm_elbow_pitch_joint", goal_position_pos_z);

    target_joint_rad[JointIds::kArmShoulderPitchJoint] = std::asin(goal_position_pos_z / kArmLength);
    target_joint_rad[JointIds::kArmElbowPitchJoint] = -M_PI_2;
    target_joint_rad[JointIds::kArmWristPitchJoint] = -target_joint_rad[JointIds::kArmShoulderPitchJoint];
  }

  // Target is below arm_elbow_pitch_join and above wrist_joint
  else if (-kArmLower <= goal_position_pos_z) {
    RCLCPP_INFO(this->get_logger(), "The target position (z:%f[m]) is below arm_elbow_pitch_join and above wrist_joint", goal_position_pos_z);

    target_joint_rad[JointIds::kArmElbowPitchJoint] = std::asin(goal_position_pos_z / kArmLower);
    target_joint_rad[JointIds::kArmWristPitchJoint] = -(M_PI_2 + target_joint_rad[JointIds::kArmElbowPitchJoint]);
  }

  // Target is below wrist_joint
  else {
    RCLCPP_INFO(this->get_logger(), "The target position (z:%f[m]) is below wrist_joint", goal_position_pos_z);

    target_joint_rad[JointIds::kArmElbowPitchJoint] = std::asin((goal_position_pos_z + kArmGripper) / kArmLower) - M_PI_2;
    target_joint_rad[JointIds::kArmWristPitchJoint] = -target_joint_rad[JointIds::kArmElbowPitchJoint];
  }

  // Check the result with forward kinematics
  double distance = 0.0;
  is_success = forward_kinematics(target_joint_rad, goal_coord, distance);

  return is_success;
}

} // namespace sobit_light

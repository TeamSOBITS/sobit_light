#include "sobit_light_library/sobit_light_joint_controller.hpp"
#include "sobit_light_library/sobit_light_wheel_controller.hpp"


namespace sobit_light {

JointController::JointController(
    const std::string& node_name)
: Node(node_name) {
  rclcpp::QoS qos_profile(1); // depth = 1
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  pub_arm_control_  = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_trajectory_controller/command", qos_profile);
  pub_head_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "head_trajectory_controller/command", qos_profile);

  sub_arm_curr_ = this->create_subscription<sobits_msgs::msg::CurrentStateArray>(
      "current_state_array", qos_profile,
      [this](const sobits_msgs::msg::CurrentStateArray::SharedPtr msg) -> void {
        callbackArmCurr(std::move(msg));
      });

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  declarePoseParams("initial_pose");
  declarePoseParams("detecting_pose");
  declarePoseParams("following_pose");

  loadPose();
}

JointController::~JointController() {
  RCLCPP_INFO(this->get_logger(), "JointController has been terminated.");
  sub_arm_curr_.reset();
}

// TODO: Load poses from yaml file
void JointController::loadPose() {
  setPoseParams("initial_pose"  , kPoseList);
  setPoseParams("detecting_pose", kPoseList);
  setPoseParams("following_pose", kPoseList);

  return;
}

bool JointController::moveToPose(
    const std::string &pose_name,
    const int32_t sec , bool is_sleep) {
  bool                is_pose = false;
  std::vector<double> joint_val;

  // Check if the pose_name exists
  for (auto& pose : kPoseList) {
      if (pose_name != pose.pose_name) continue;
      is_pose   = true;
      joint_val = pose.joint_val;
      break;
  }

  // Move to the pose
  if (is_pose) {
      RCLCPP_INFO(this->get_logger(), "[SOBIT LIGHT] Pose '%s' was found.", pose_name.c_str());
      return moveAllJointsRad(
          joint_val[Joint::kArmShoulderRollJoint], 
          joint_val[Joint::kArmShoulderPitchJoint], 
          joint_val[Joint::kArmElbowPitchJoint], 
          joint_val[Joint::kArmForearmRollJoint], 
          joint_val[Joint::kArmWristPitchJoint], 
          joint_val[Joint::kArmWristRollJoint], 
          joint_val[Joint::kHandJoint],
          joint_val[Joint::kHeadYawJoint],
          joint_val[Joint::kHeadPitchJoint],
          sec, is_sleep);
  } else {
      RCLCPP_WARN(this->get_logger(), "[SOBIT LIGHT] Pose '%s' was not found.", pose_name.c_str());
      return false;
  }
}

bool JointController::moveAllJointsRad(
  const double arm_shoulder_roll,
  const double arm_shoulder_pitch,
  const double arm_elbow_pitch,
  const double arm_forearm_roll,
  const double arm_wrist_pitch,
  const double arm_wrist_roll,
  const double hand,
  const double head_yaw,
  const double head_pitch,
  const int32_t sec , bool is_sleep ) {
  try {
    trajectory_msgs::msg::JointTrajectory arm_joint_trajectory;
    trajectory_msgs::msg::JointTrajectory head_joint_trajectory;

    // Arm
    setJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmShoulderRollJoint] , arm_shoulder_roll , sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmShoulderPitchJoint], arm_shoulder_pitch, sec);
    // addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmShoulderPitchJoint], arm_shoulder_pitch, sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmElbowPitchJoint]   , arm_elbow_pitch   , sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmForearmRollJoint]  , arm_forearm_roll  , sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmWristPitchJoint]   , arm_wrist_pitch   , sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmWristRollJoint]    , arm_wrist_roll    , sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kHandJoint]            , hand              , sec);
    
    // Head
    setJointTrajectory(&head_joint_trajectory, kJointNames[Joint::kHeadYawJoint]         ,  head_yaw         , sec);
    addJointTrajectory(&head_joint_trajectory, kJointNames[Joint::kHeadPitchJoint]       ,  head_pitch       , sec);

    // Check publishers connection
    checkPublishersConnection(pub_arm_control_);
    checkPublishersConnection(pub_head_control_);

    // Publish the joint trajectory
    pub_arm_control_->publish(arm_joint_trajectory);
    pub_head_control_->publish(head_joint_trajectory);
    
    if (is_sleep) rclcpp::sleep_for(std::chrono::seconds(sec));
    return true;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "[SOBIT LIGHT] %s", ex.what());

    return false;
  }
}

bool JointController::moveJointRad(
    const Joint  joint_num,
    const double rad,
    const int32_t sec , bool is_sleep ) {
  try {
    trajectory_msgs::msg::JointTrajectory joint_trajectory;

    setJointTrajectory(&joint_trajectory, kJointNames[joint_num], rad, sec);

    // if (joint_num == 1 || joint_num == 3) {
    //     setJointTrajectory(&joint_trajectory, kJointNames[joint_num]    ,  rad, sec);
    //     addJointTrajectory(&joint_trajectory, kJointNames[joint_num + 1], -rad, sec);
    // } else {
    //     setJointTrajectory(&joint_trajectory, kJointNames[joint_num]    ,  rad, sec);
    // }
    
    if (joint_num <= Joint::kHandJoint) {
        checkPublishersConnection(pub_arm_control_);
        pub_arm_control_->publish(joint_trajectory);
    } else {
        checkPublishersConnection(pub_head_control_);
        pub_head_control_->publish(joint_trajectory);
    }

    if (is_sleep) rclcpp::sleep_for(std::chrono::seconds(sec));
    return true;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "[SOBIT LIGHT] %s", ex.what());
    return false;
  }
}

bool JointController::moveHeadRad(
    const double head_yaw,
    const double head_pitch,
    const int32_t sec , bool is_sleep ) {
  try {
    trajectory_msgs::msg::JointTrajectory joint_trajectory;

    setJointTrajectory(&joint_trajectory, kJointNames[Joint::kHeadYawJoint]  , head_yaw  , sec);
    addJointTrajectory(&joint_trajectory, kJointNames[Joint::kHeadPitchJoint], head_pitch, sec);
    
    checkPublishersConnection(pub_head_control_);
    pub_head_control_->publish(joint_trajectory);
    
    if (is_sleep)rclcpp::sleep_for(std::chrono::seconds(sec));
    return true;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "[SOBIT LIGHT] %s", ex.what());
    return false;
  }
}

bool JointController::moveArmRad(
    const double arm_shoulder_roll,
    const double arm_shoulder_pitch,
    const double arm_elbow_pitch,
    const double arm_forearm_roll,
    const double arm_wrist_pitch,
    const double arm_wrist_roll,
    const double hand,
    const int32_t sec , bool is_sleep ) {
  try {
    trajectory_msgs::msg::JointTrajectory arm_joint_trajectory;

    setJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmShoulderRollJoint] , arm_shoulder_roll , sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmShoulderPitchJoint], arm_shoulder_pitch, sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmElbowPitchJoint]   , arm_elbow_pitch   , sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmForearmRollJoint]  , arm_forearm_roll  , sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmWristPitchJoint]   , arm_wrist_pitch   , sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kArmWristRollJoint]    , arm_wrist_roll    , sec);
    addJointTrajectory(&arm_joint_trajectory, kJointNames[Joint::kHandJoint]            , hand              , sec);
    
    checkPublishersConnection (pub_arm_control_);
    pub_arm_control_->publish(arm_joint_trajectory);
    
    if (is_sleep) rclcpp::sleep_for(std::chrono::seconds(sec));
    return true;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "[SOBIT LIGHT] %s", ex.what());
    return false;
  }
}

// TODO (@m.shigemori): Implement inverse kinematics
bool JointController::moveHandToTargetCoord(
    const double target_x, const double target_y, const double target_z, 
    const double shift_x , const double shift_y , const double shift_z,
    const int32_t sec , bool is_sleep ) {
  // sobit_light::WheelController wheel_ctrl;
  WheelController wheel_ctrl;

  // // Calculate goal_position_pos + difference(gap)
  const double goal_position_pos_x = target_x + shift_x;
  const double goal_position_pos_y = target_y + shift_y;
  const double goal_position_pos_z = target_z + shift_z;
  bool is_reached = false;

  // Check if the object is graspable
  if (goal_position_pos_z > kArmLength) {
    // std::cout << "The target is located too tall ("  << goal_position_pos_z << ">80.0)" << std::endl;
    RCLCPP_WARN(this->get_logger(), "The target is located too tall (%f>80.0)", goal_position_pos_z);
    return is_reached;

  } else if (goal_position_pos_z < -kArmLength) {
    // std::cout << "The target is located too low (" << goal_position_pos_z << "<35.0) " << std::endl;
    RCLCPP_WARN(this->get_logger(), "The target is located too low (%f>35.0)", goal_position_pos_z);
    return is_reached;
  }

  double arm_shoulder_roll_joint_rad  = 0.0;
  double arm_shoulder_pitch_joint_rad = 0.0;
  double arm_elbow_pitch_joint_rad    = 0.0;
  double arm_forearm_roll_joint_rad   = 0.0;
  double arm_wrist_pitch_joint_rad    = 0.0;
  double arm_wrist_roll_joint_rad     = 0.0;
  double hand_joint_rad               = 0.0;

  double base_to_arm_forearm_roll_joint_x_cm = 0.0;

  // Target is above arm_elbow_pitch_join
  if (kArmUpper < goal_position_pos_z) {
    // std::cout << "Target (z:" << goal_position_pos_z << ") is above arm_elbow_pitch_joint" << std::endl;
    RCLCPP_INFO(this->get_logger(), "Target (z:%f) is above arm_elbow_pitch_joint", goal_position_pos_z);

    // Caution: Calculating until arm_forearm_roll_joint_x_cm (not target)
    double arm_elbow_pitch_joint_sin = (goal_position_pos_z - kArmUpper) / kArmLower;
    arm_elbow_pitch_joint_rad = std::asin(arm_elbow_pitch_joint_sin);
    arm_forearm_roll_joint_rad = -arm_elbow_pitch_joint_rad;
    arm_shoulder_pitch_joint_rad = 0.0;

    base_to_arm_forearm_roll_joint_x_cm = kArmUpper + kArmLower * std::cos(arm_elbow_pitch_joint_rad);
  }

  // Target is below arm_elbow_pitch_join and above shoulder_flex_joint
  else if (0.0 <= goal_position_pos_z && goal_position_pos_z <= kArmUpper) {
    // std::cout << "Target (z:" << goal_position_pos_z << ") is below arm_elbow_pitch_join and above shoulder_flex_joint" << std::endl;
    RCLCPP_INFO(this->get_logger(), "Target (z:%f) is below arm_elbow_pitch_join and above shoulder_flex_joint", goal_position_pos_z);

    // Caution: Calculating until arm_forearm_roll_joint_x_cm (not target)
    double arm_elbow_pitch_joint_sin = (kArmUpper - goal_position_pos_z) / kArmLower;
    arm_elbow_pitch_joint_rad = -std::asin(arm_elbow_pitch_joint_sin);
    arm_forearm_roll_joint_rad = -arm_elbow_pitch_joint_rad;
    arm_shoulder_pitch_joint_rad = 0.0;

    base_to_arm_forearm_roll_joint_x_cm = kArmUpper + kArmLower * std::cos(arm_elbow_pitch_joint_rad);
  }

  // Target is below shoulder_flex_joint
  else if (goal_position_pos_z < 0.0) {
    // std::cout << "Target (z:" << goal_position_pos_z << ") is below shoulder_flex_joint" << std::endl;
    RCLCPP_INFO(this->get_logger(), "Target (z:%f) is below shoulder_flex_joint", goal_position_pos_z);

    // Caution: Calculating until arm_forearm_roll_joint_x_cm (not target)
    double arm_elbow_pitch_joint_cos = (kArmUpper - goal_position_pos_z) / kArmLower;
    arm_elbow_pitch_joint_rad = std::acos(arm_elbow_pitch_joint_cos);
    arm_forearm_roll_joint_rad = std::asin(arm_elbow_pitch_joint_cos);
    arm_shoulder_pitch_joint_rad = -wheel_ctrl.deg2Rad(90.0);

    base_to_arm_forearm_roll_joint_x_cm = kArmUpper + kArmLower * std::cos(arm_elbow_pitch_joint_rad);
  }

  // Calculate wheel movement (diagonal)
  // - Rotate the robot
  const double rot_rad = std::atan2(goal_position_pos_y, goal_position_pos_x);
  // ROS_INFO("rot_rad = %f(deg:%f)", rot_rad, SobitLightWheelController::rad2Deg(rot_rad));
  wheel_ctrl.controlWheelRotateRad(rot_rad);
  rclcpp::sleep_for(std::chrono::seconds(1));
  // - Move forward the robot
  const double linear_m = std::sqrt(std::pow(goal_position_pos_x, 2) + std::pow(goal_position_pos_y, 2)) - base_to_arm_forearm_roll_joint_x_cm;
  RCLCPP_INFO(this->get_logger(), "linear_m = %f", linear_m);
  wheel_ctrl.controlWheelLinear(linear_m);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // // Calculate wheel movement (+-90->x_pos->-+90->y_pos) NEEDS CONFIRMATION
  // // - Rotate the robot
  // const double rot_deg = goal_position_pos_x > 0.0 ? 90.0:-90.0;
  // ROS_INFO("rot_deg:%f)", rot_deg);
  // wheel_ctrl.controlWheelRotateDeg(rot_deg);
  // rclcpp::sleep_for(std::chrono::seconds(3));

  // // - Move forward the robot
  // ROS_INFO("linear_m = %f", goal_position_pos_x);
  // wheel_ctrl.controlWheelLinear(goal_position_pos_x);
  // rclcpp::sleep_for(std::chrono::seconds(3));

  // // - Rotate the robot
  // ROS_INFO("rot_deg:%f)", -rot_deg);
  // wheel_ctrl.controlWheelRotateDeg(-rot_deg);
  // rclcpp::sleep_for(std::chrono::seconds(3));

  // // - Move forward the robot
  // ROS_INFO("linear_m = %f", goal_position_pos_y);
  // wheel_ctrl.controlWheelLinear(goal_position_pos_y);
  // rclcpp::sleep_for(std::chrono::seconds(3));

  // - Move arm (OPEN)
  is_reached = moveArmRad(
      arm_shoulder_roll_joint_rad,
      arm_shoulder_pitch_joint_rad,
      arm_elbow_pitch_joint_rad,
      arm_forearm_roll_joint_rad,
      arm_wrist_pitch_joint_rad,
      arm_wrist_roll_joint_rad,
      hand_joint_rad,
      sec, is_sleep);

  RCLCPP_INFO(this->get_logger(), "goal_position_pos = (%f, %f, %f)",
      goal_position_pos_x, goal_position_pos_y, goal_position_pos_z);
  // rclcpp::Duration(2, 0)

  return is_reached;
}

bool JointController::moveHandToTargetTF(
    const std::string &target_name,
    const double shift_x, const double shift_y, const double shift_z,
    const int32_t sec , bool is_sleep ) {
  geometry_msgs::msg::TransformStamped transformStamped;
  bool is_reached = false;

  try {
    tf_buffer_->canTransform("arm_shoulder_roll_joint", target_name, this->get_clock()->now(), rclcpp::Duration(0, RCL_S_TO_NS(0.5)));
    transformStamped = tf_buffer_->lookupTransform("arm_shoulder_roll_joint", target_name, this->get_clock()->now());
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "[SOBIT LIGHT] %s", ex.what());
    return false;
  }

  auto& tf_target_to_arm = transformStamped.transform.translation;
  is_reached = moveHandToTargetCoord(
      tf_target_to_arm.x, tf_target_to_arm.y, tf_target_to_arm.z,
      shift_x, shift_y, shift_z,
      sec, is_sleep);
  
  return is_reached;
}

bool JointController::moveHandToPlaceCoord(
    const double target_x, const double target_y,  double target_z, 
    const double shift_x     , const double shift_y     , const double shift_z,
    const int32_t sec , bool is_sleep ) {
  geometry_msgs::msg::Point shift;
  // double target_z   = 0.;
  bool   is_reached = false;

  // Reduce the target_z by 0.002[m], until expected collision is detected
  while (!(is_reached && placeDecision(500, 1000))) {
    is_reached = moveHandToTargetCoord(
        target_x, target_y, target_z, 
        shift_x, shift_y, shift_z+target_z,
        sec, is_sleep);

    if (!is_reached) return is_reached;

    // Accumulate the target_z
    target_z -= 0.002;
  }

  return is_reached;
}

bool JointController::moveHandToPlaceTF(
    const std::string& target_name,
    const double shift_x, const double shift_y, const double shift_z,
    const int32_t sec , bool is_sleep ) {
  geometry_msgs::msg::Point shift;
  geometry_msgs::msg::TransformStamped transform_base_to_target;
  bool is_reached = false;

  try {
    tf_buffer_->canTransform("arm_shoulder_roll_joint", target_name, this->get_clock()->now(), rclcpp::Duration(2, 0));
    transform_base_to_target = tf_buffer_->lookupTransform("arm_shoulder_roll_joint", target_name, this->get_clock()->now());
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "[SOBIT LIGHT] %s", ex.what());
    return false;
  }

  auto& goal_position = transform_base_to_target.transform.translation;

  is_reached = moveHandToPlaceCoord(
      goal_position.x, goal_position.y, goal_position.z,
      shift_x, shift_y, shift_z,
      sec, is_sleep);

  return is_reached;
}

bool JointController::graspDecision(const int min_curr, const int max_curr) {
    bool is_grasped = false;

    // Spin until the current value is obtained
    // while(kHandCurr == 0.) ros::spinOnce();
    std::cout << "kHandCurr = " << kHandCurr << std::endl;

    is_grasped = (min_curr <= kHandCurr && kHandCurr <= max_curr) ? true : false;

    return is_grasped;
}

bool JointController::placeDecision(const int min_curr, const int max_curr) {
    bool is_placed = false;

    // Spin until the current value is obtained
    // while(kArmWristPitchCurr == 0.) ros::spinOnce();
    std::cout << "kArmWristPitchCurr = " << kArmWristPitchCurr << std::endl;

    is_placed = (min_curr <= kArmWristPitchCurr && kArmWristPitchCurr <= max_curr) ? true : false;

    return is_placed;
}

}  // namespace sobit_light

// RCLCPP_COMPONENTS_REGISTER_NODE(sobit_light::JointController)
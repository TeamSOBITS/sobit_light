#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from math import (sin, cos, acos, atan2, sqrt, pow, pi)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from control_msgs.action import FollowJointTrajectory

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from sobits_msgs.msg import CurrentStateArray, CurrentState

from .sobit_light_wheel_controller import WheelController

from rclpy.parameter import Parameter
from rclpy.parameter import ParameterType as Type

class Joints(enumerate):
  kArmShoulderRollJoint     = 0
  kArmShoulderPitchJoint    = 1
  kArmShoulderPitchSubJoint = 2
  kArmElbowPitchJoint       = 3
  kArmForearmRollJoint      = 4
  kArmWristPitchJoint       = 5
  kArmWristRollJoint        = 6
  kHandJoint                = 7
  kHeadYawJoint             = 8
  kHeadPitchJoint           = 9
  kJointNum                 = 10

class Pose:
  def __init__(self, pose_name, joint_val):
    self.name = pose_name
    self.joint_values = joint_val


kJointNames = [
    'arm_shoulder_roll_joint',
    'arm_shoulder_pitch_joint',
    'arm_shoulder_pitch_sub_joint',
    'arm_elbow_pitch_joint',
    'arm_forearm_roll_joint',
    'arm_wrist_pitch_joint',
    'arm_wrist_roll_joint',
    'hand_joint',
    'head_yaw_joint',
    'head_pitch_joint',
]


class JointController(Node):
  def __init__(self, node_name='sobit_light_joint_controller'):
    super().__init__(node_name)
    self.declare_parameters(
        namespace='',
        parameters=[
            ('poses.names', ['initial_pose', 'detecting_pose', 'raise_hand']),
            ('poses.initial_pose', [0.0, -1.5708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ('poses.detecting_pose', [0.0, -1.5708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.7854]),
            ('poses.raise_hand', [0.0, -1.5708, -1.5708, 0.0, 0.0, 0.0, 0.7854, 0.0, 0.0]),
        ]
    )

    # QoS profile for the subscription
    qos_policy = rclpy.qos.QoSProfile(
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        depth=1)
    
    # 
    # self.sub_arm_curr_ = self.create_subscription(
    #     CurrentStateArray,
    #     '/current_state_array',
    #     self.callbackArmCurr,
    #     qos_profile=qos_policy,
    # )

    # self.pub_arm_control_ = self.create_publisher(
    #     JointTrajectory,
    #     '/arm_trajectory_controller/command',
    #     qos_profile=qos_policy,
    # )

    # self.pub_head_control_ = self.create_publisher(
    #     JointTrajectory,
    #     '/head_trajectory_controller/command',
    #     qos_profile=qos_policy,
    # )

    self.action_joints_client_ = ActionClient(
        self,
        FollowJointTrajectory,
        '/joint_trajectory_controller/follow_joint_trajectory',
    )

    self.pub_joints_control_ = self.create_publisher(
        JointState,
        '/joint_states',
        qos_profile=qos_policy,
    )

    self.sub_joint_state_ = self.create_subscription(
        JointState,
        '/joint_states',
        self.callbackJointState,
        qos_profile=qos_policy,
    )

    # Flags
    self.is_running_ = False

    # Variables
    self.tf_buffer_ = Buffer()
    self.tf_listener_ = TransformListener(self.tf_buffer_, self)
    self.all_jt_ = JointTrajectory()

    self.kCurrentArray = [0.0] * Joints.kJointNum
    self.kPositionArray = [0.0] * Joints.kJointNum

    # Constants
    self.kArmUpper   = 0.128
    self.kArmUpperX  = 0.022
    self.kArmLower   = 0.124
    self.kArmGripper = 0.064 + 0.11225
    self.kArmLength  = self.kArmUpper + self.kArmLower
    self.kGraspMinZ  = 0.035
    self.kGraspMaxZ  = 0.080

    self.kPoses = []
  

    # Rate
    self._loop_rate = self.create_rate(20, self.get_clock())

    # Load poses
    self.loadPoses()

  def __del__(self):
    self.get_logger().info('Destroying the node...')

    # self.pub_arm_control_.destroy()
    # self.pub_head_control_.destroy()
    self.pub_joints_control_.destroy()
    # self.sub_arm_curr_.destroy()

  def setJointTrajectoryPoint(self,
      joint_name,
      rad,
      duration):
    joint_trajectory = JointTrajectory()
    joint_trajectory_point = JointTrajectoryPoint()

    joint_trajectory.joint_names.append(joint_name)
    joint_trajectory_point.positions.append(rad)
    joint_trajectory_point.time_from_start = rclpy.time.Duration(seconds=duration).to_msg()
    joint_trajectory.points.append(joint_trajectory_point)

    self.all_jt_ = joint_trajectory


  def addJointTrajectoryPoint(self,
      joint_name,
      rad,
      duration):
    joint_trajectory = JointTrajectory()
    joint_trajectory = self.all_jt_

    joint_trajectory.joint_names.append(joint_name)
    joint_trajectory.points[0].positions.append(rad)
    joint_trajectory.points[0].time_from_start = rclpy.time.Duration(seconds=duration).to_msg()

    self.all_jt_ = joint_trajectory

  def setFirstJT(self, duration):
    joint_trajectory = JointTrajectory()
    joint_trajectory_point = JointTrajectoryPoint()

    joint_trajectory.joint_names.append('arm_shoulder_roll_joint')
    joint_trajectory_point.positions.append(self.kPositionArray[Joints.kArmShoulderRollJoint])
    joint_trajectory_point.time_from_start = rclpy.time.Duration(seconds=duration).to_msg()
    joint_trajectory.points.append(joint_trajectory_point)

    self.all_jt_ = joint_trajectory


  def setRestJT(self, duration):
    joint_trajectory = JointTrajectory()
    joint_trajectory = self.all_jt_

    for i in range(1, Joints.kJointNum):
      joint_trajectory.joint_names.append(kJointNames[i])
      joint_trajectory.points[0].positions.append(self.kPositionArray[i])
      joint_trajectory.points[0].time_from_start = rclpy.time.Duration(seconds=duration).to_msg()

    self.all_jt_ = joint_trajectory

  def updOneJT(self, joint_num, rad, duration):
    joint_trajectory = JointTrajectory()
    joint_trajectory = self.all_jt_


    joint_trajectory.joint_names[joint_num] = kJointNames[joint_num]
    joint_trajectory.points[0].positions[joint_num] = rad
    joint_trajectory.points[0].time_from_start = rclpy.time.Duration(seconds=duration).to_msg()

    self.all_jt_ = joint_trajectory


  def sendGoal(self,duration=5.0):
    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory = self.all_jt_

    print('Waiting for server...')
    self.action_joints_client_.wait_for_server()
    print('Sending goal...')
    self._send_goal_future = self.action_joints_client_.send_goal_async(goal_msg)

    self.get_clock().sleep_until(self.get_clock().now() + rclpy.time.Duration(seconds=duration))

    self.all_jt_ = JointTrajectory()
    self.kPositionArray = [0] * Joints.kJointNum


  def checkPublishersConnection(self, pub):
    while self.kPositionArray[0] == 0:
      self.get_logger().warn('No subscribers connected to %s' % pub.topic_name)
      self.get_logger().warn('Waiting...')
      rclpy.spin_once(self)


  def loadPoses(self):
    names = self.get_parameter('poses.names').value
    initial_joints = self.get_parameter('poses.initial_pose').value
    detecting_joints = self.get_parameter('poses.detecting_pose').value
    raise_hand_joints = self.get_parameter('poses.raise_hand').value

    initial_pose = Pose(names[0], initial_joints)
    detecting_pose = Pose(names[1], detecting_joints)
    raise_hand_pose = Pose(names[2], raise_hand_joints)

    self.kPoses.append(initial_pose)
    self.kPoses.append(detecting_pose)
    self.kPoses.append(raise_hand_pose)

    print(self.kPoses[0].joint_values)

  def moveToPose(self, pose_name, duration=1.0, wait=True):
    is_moved = False
    for i in range(len(self.kPoses)):
      if self.kPoses[i].name == pose_name:
        is_moved = self.moveAllJointsRad(
            self.kPoses[i].joint_values[:6],
            self.kPoses[i].joint_values[6],
            self.kPoses[i].joint_values[7:],
            duration, wait)
        break
    return is_moved

  def moveAllJointsRad(self,
      arm_joint_vals,
      hand_joint_val,
      head_joint_vals,
      duration=1.0, wait=True):
    # arm_joint_trajectory = JointTrajectory()
    # head_joint_trajectory = JointTrajectory()
    while self.kPositionArray[0] == 0:
      self.get_logger().info('Waiting for position...')
      rclpy.spin_once(self)

    # # Arm joints
    # self.setJointTrajectoryPoint(arm_joint_trajectory, kJointNames[Joints.kArmShoulderRollJoint] , arm_joint_vals[0], duration)
    # self.addJointTrajectoryPoint(arm_joint_trajectory, kJointNames[Joints.kArmShoulderPitchJoint], arm_joint_vals[1], duration)
    # self.addJointTrajectoryPoint(arm_joint_trajectory, kJointNames[Joints.kArmShoulderPitchSubJoint], -arm_joint_vals[1], duration)
    # self.addJointTrajectoryPoint(arm_joint_trajectory, kJointNames[Joints.kArmElbowPitchJoint]   , arm_joint_vals[2], duration)
    # self.addJointTrajectoryPoint(arm_joint_trajectory, kJointNames[Joints.kArmForearmRollJoint]  , arm_joint_vals[3], duration)
    # self.addJointTrajectoryPoint(arm_joint_trajectory, kJointNames[Joints.kArmWristPitchJoint]   , arm_joint_vals[4], duration)
    # self.addJointTrajectoryPoint(arm_joint_trajectory, kJointNames[Joints.kArmWristRollJoint]    , arm_joint_vals[5], duration)

    # # Hand joint
    # self.addJointTrajectoryPoint(arm_joint_trajectory, kJointNames[Joints.kHandJoint], hand_joint_val, duration)

    # # Head joints
    # self.setJointTrajectoryPoint(head_joint_trajectory, kJointNames[Joints.kHeadYawJoint]  , head_joint_vals[0], duration)
    # self.addJointTrajectoryPoint(head_joint_trajectory, kJointNames[Joints.kHeadPitchJoint], head_joint_vals[1], duration)

    # # Check publishers connection
    # self.checkPublishersConnection(self.pub_arm_control_)
    # self.checkPublishersConnection(self.pub_head_control_)


    # All Joints:
    self.setJointTrajectoryPoint(kJointNames[Joints.kArmShoulderRollJoint] , arm_joint_vals[0], duration)
    self.addJointTrajectoryPoint(kJointNames[Joints.kArmShoulderPitchJoint], arm_joint_vals[1], duration)
    self.addJointTrajectoryPoint(kJointNames[Joints.kArmShoulderPitchSubJoint], -arm_joint_vals[1], duration)
    self.addJointTrajectoryPoint(kJointNames[Joints.kArmElbowPitchJoint]   , arm_joint_vals[2], duration)
    self.addJointTrajectoryPoint(kJointNames[Joints.kArmForearmRollJoint]  , arm_joint_vals[3], duration)
    self.addJointTrajectoryPoint(kJointNames[Joints.kArmWristPitchJoint]   , arm_joint_vals[4], duration)
    self.addJointTrajectoryPoint(kJointNames[Joints.kArmWristRollJoint]    , arm_joint_vals[5], duration)
    self.addJointTrajectoryPoint(kJointNames[Joints.kHandJoint], hand_joint_val, duration)
    self.addJointTrajectoryPoint(kJointNames[Joints.kHeadYawJoint]  , head_joint_vals[0], duration)
    self.addJointTrajectoryPoint(kJointNames[Joints.kHeadPitchJoint], head_joint_vals[1], duration)

    # Check publishers connection
    # self.checkPublishersConnection(self.pub_arm_control_)
    # self.checkPublishersConnection(self.pub_head_control_)
    self.checkPublishersConnection(self.pub_joints_control_)

    # Publish the joint trajectories
    # self.pub_arm_control_.publish(arm_joint_trajectory)
    # self.pub_head_control_.publish(head_joint_trajectory)
    self.sendGoal(duration)

  def moveJointRad(self,
      joint_num,
      rad,
      duration=1.0, wait=True):
    # arm_joint_trajectory = JointTrajectory()
    while self.kPositionArray[0] == 0:
      self.get_logger().info('Waiting for position...')
      rclpy.spin_once(self)

    # Maintain position
    self.setFirstJT(duration)
    self.setRestJT(duration)

    # Arm joints
    if (joint_num == Joints.kArmShoulderPitchJoint):
      self.updOneJT(joint_num, rad, duration)
      self.updOneJT(joint_num+1, -rad, duration)

    elif (joint_num == Joints.kArmShoulderPitchSubJoint):
      self.updOneJT(joint_num, -rad, duration)
      self.updOneJT(joint_num-1, rad, duration)

    else:
      self.updOneJT(joint_num, rad, duration)
    
    # Check publishers connection
    # if (joint_num < Joints.kHandJoint):
    #   self.checkPublishersConnection(self.pub_arm_control_)
    #   self.pub_arm_control_.publish(arm_joint_trajectory)
    # else:
    #   self.checkPublishersConnection(self.pub_head_control_)
    #   self.pub_head_control_.publish(arm_joint_trajectory)
    
    self.checkPublishersConnection(self.pub_joints_control_)
    self.sendGoal(duration)

  def moveArmRad(self,
      arm_joint_vals,
      hand_joint_val,
      duration=1.0, wait=True):
    # arm_joint_trajectory = JointTrajectory()
    while self.kPositionArray[0] == 0:
      self.get_logger().info('Waiting for position...')
      rclpy.spin_once(self)


    # Maintain position
    self.setFirstJT(duration)
    self.setRestJT(duration)
  
    # Arm joints
    self.updOneJT(Joints.kArmShoulderRollJoint , arm_joint_vals[0], duration)
    self.updOneJT(Joints.kArmShoulderPitchJoint, arm_joint_vals[1], duration)
    self.updOneJT(Joints.kArmShoulderPitchSubJoint, -arm_joint_vals[1], duration)
    self.updOneJT(Joints.kArmElbowPitchJoint   , arm_joint_vals[2], duration)
    self.updOneJT(Joints.kArmForearmRollJoint  , arm_joint_vals[3], duration)
    self.updOneJT(Joints.kArmWristPitchJoint   , arm_joint_vals[4], duration)
    self.updOneJT(Joints.kArmWristRollJoint    , arm_joint_vals[5], duration)

    # Hand joint
    self.updOneJT(Joints.kHandJoint, hand_joint_val, duration)
    
    # Check publishers connection
    # self.checkPublishersConnection(self.pub_arm_control_)
    self.checkPublishersConnection(self.pub_joints_control_)


    # Publish the joint trajectories
    # self.pub_arm_control_.publish(arm_joint_trajectory)
    self.sendGoal(duration)

  def moveHeadRad(self,
      head_joint_vals,
      duration=1.0, wait=True):
    # head_joint_trajectory = JointTrajectory()
    while self.kPositionArray[0] == 0:
      self.get_logger().info('Waiting for position...')
      rclpy.spin_once(self)

    # Maintain position
    self.setFirstJT(duration)
    self.setRestJT(duration)

    # Head joints
    self.updOneJT(Joints.kHeadYawJoint  , head_joint_vals[0], duration)
    self.updOneJT(Joints.kHeadPitchJoint, head_joint_vals[1], duration)

    # Check publishers connection
    # self.checkPublishersConnection(self.pub_head_control_)
    self.checkPublishersConnection(self.pub_joints_control_)

    # Publish the joint trajectories
    # self.pub_head_control_.publish(head_joint_trajectory)
    self.sendGoal(duration)


  def moveHandToTargetCoord(self,
      target_coord,
      shift_coord,
      duration=1.0, wait=True):
    is_reached = False
    wheel_ctrl = WheelController()

    goal_coord = [target_coord[0] + shift_coord[0],
                  target_coord[1] + shift_coord[1],
                  target_coord[2] + shift_coord[2]]

    if goal_coord[2] > self.kArmLength * 2:
        self.get_logger().error('The target is too far (%f > %f)' % (goal_coord[2], self.kArmLength * 2))
        return is_reached

    elif goal_coord[2] < -self.kArmLength:
        self.get_logger().error('The target is located too low (%f < %f)' % (goal_coord[2], -self.kArmLength))
        return is_reached

    arm_shoulder_roll_joint_rad = 0.0
    arm_shoulder_pitch_joint_rad = 0.0
    arm_elbow_pitch_joint_rad = 0.0
    arm_forearm_roll_joint_rad = 0.0
    arm_wrist_pitch_joint_rad = 0.0
    arm_wrist_roll_joint_rad = 0.0
    hand_joint_rad = 0.0
    elbow_joint_z = 0.0

    if goal_coord[2] > self.kArmUpper:
        self.get_logger().info('The target is located above the arm (%f > %f)' % (goal_coord[2], self.kArmUpper))
        elbow_joint_z = self.kArmLength
        arm_shoulder_pitch_joint_rad = pi / 2

    elif 0 <= goal_coord[2] <= self.kArmUpper:
        self.get_logger().info('The target is located below arm_elbow_pitch_join and above shoulder_pitch_joint (%f <= %f <= %f)' % (0, goal_coord[2], self.kArmUpper))
        elbow_joint_z = self.kArmLength * sin(pi / 4)
        arm_shoulder_pitch_joint_rad = pi / 4

    elif goal_coord[2] < 0:
        self.get_logger().info('The target is located below the shoulder_pitch_joint (%f < %f)' % (goal_coord[2], 0))
        elbow_joint_z = 0.0
        arm_shoulder_pitch_joint_rad = 0.0

    base_z = elbow_joint_z - goal_coord[2]
    arm_elbow_pitch_joint_rad = acos(base_z / self.kArmUpper)
    arm_wrist_pitch_joint_rad = pi / 2 - (arm_shoulder_pitch_joint_rad + arm_elbow_pitch_joint_rad)

    arm_wrist_joint_x = self.kArmUpper * (cos(arm_shoulder_pitch_joint_rad) + cos(arm_shoulder_pitch_joint_rad + arm_elbow_pitch_joint_rad))
    arm_wrist_joint_z = self.kArmUpper * (sin(arm_shoulder_pitch_joint_rad) + sin(arm_shoulder_pitch_joint_rad + arm_elbow_pitch_joint_rad))

    # Rotate the robot
    rot_rad = atan2(goal_coord[1], goal_coord[0])
    # wheel_ctrl.controlWheelRotateRad(rot_rad)

    # Move forward
    linear_m = sqrt(goal_coord[0] ** 2 + goal_coord[1] ** 2)
    # wheel_ctrl.controlWheelLinear(linear_m)

    is_reached = self.moveArmRad(
        [
            arm_shoulder_roll_joint_rad,
            arm_shoulder_pitch_joint_rad,
            arm_elbow_pitch_joint_rad,
            arm_forearm_roll_joint_rad,
            arm_wrist_pitch_joint_rad,
            arm_wrist_roll_joint_rad
        ],
        hand_joint_rad,
        duration, wait)

    return is_reached


  def moveHandToTargetTF(self,
      target_name,
      shift_coord,
      duration=1.0, wait=True):
    is_reached = False

    transformStamped = TransformStamped()

    try:
      self.tf_buffer_.can_transform(
          'arm_base_joint', target_name,
          rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
      transformStamped = self.tf_buffer_.lookup_transform('arm_base_joint', target_name, rclpy.time.Time())
    except TransformException as e:
      self.get_logger().error('Failed to get transform: %s' % e)
      return is_reached
    
    target_coord = [
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z]
    is_reached = self.moveHandToTargetCoord(
        target_coord,
        shift_coord,
        duration, wait)

    return is_reached


  def moveHandToPlaceCoord(self,
      target_coord,
      shift_coord,
      duration=1.0, wait=True):
    is_placed = False

    while not(is_placed and self.graspDecision(500.0, 1000.0)):
      is_placed = self.moveHandToTargetCoord(
          target_coord,
          shift_coord,
          duration, wait)
      
      if not is_placed:
        self.get_logger().error('Failed to place the object')
        return is_placed
      
      shift_coord[2] = -0.002
    
    return is_placed


  def moveHandToPlaceTF(self,
      target_name,
      shift_coord,
      duration=1.0, wait=True):
    is_placed = False

    transformStamped = TransformStamped()

    try:
      self.tf_buffer_.can_transform(
          'arm_base_joint', target_name,
          rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
      transformStamped = self.tf_buffer_.lookup_transform('arm_base_joint', target_name, rclpy.time.Time())
    except TransformException as e:
      self.get_logger().error('Failed to get transform: %s' % e)
      return is_placed
    
    target_coord = [
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z]
    is_placed = self.moveHandToPlaceCoord(
        target_coord,
        shift_coord,
        duration, wait)

    return is_placed

  def graspDecision(self,
      min_current, max_current):
    is_grasped = False
    while self.kCurrentArray[kJointNames[Joints.kHandJoint]] == 0:
      self.get_logger().info('Waiting for current...')
      rclpy.spin_once(self)
    
    is_grasped = True if \
      self.kCurrentArray[kJointNames[Joints.kHandJoint]] > min_current \
      and self.kCurrentArray[kJointNames[Joints.kHandJoint]] < max_current \
      else False
  
    return is_grasped

  def placeDecision(self,
      min_current, max_current):
    is_placed = False
    while self.kCurrentArray[kJointNames[Joints.kArmWristPitchJoint]] == 0:
      self.get_logger().info('Waiting for current...')
      rclpy.spin_once(self)
    
    is_placed = True if \
        self.kCurrentArray[kJointNames[Joints.kArmWristPitchJoint]] > min_current \
        and self.kCurrentArray[kJointNames[Joints.kArmWristPitchJoint]] < max_current \
        else False

    return is_placed

  def callbackJointState(self, msg):
    for i in range(len(msg.name)):
      for j in range(Joints.kJointNum):
        if msg.name[i] == kJointNames[j]:
          self.kPositionArray[j] = msg.position[i]

  def callbackArmCurr(self, msg):
    # self.kCurrentArray[kJointNames[Joints.kArmShoulderRollJoint]] = msg.current_state_array[Joints.kArmShoulderRollJoint].current_ma
    # self.kCurrentArray[kJointNames[Joints.kArmShoulderPitchJoint]] = msg.current_state_array[Joints.kArmShoulderPitchJoint].current_ma
    # self.kCurrentArray[kJointNames[Joints.kArmElbowPitchJoint]] = msg.current_state_array[Joints.kArmElbowPitchJoint].current_ma
    self.kCurrentArray[kJointNames[Joints.kArmWristPitchJoint]] = msg.current_state_array[Joints.kArmWristPitchJoint].current_ma
    # self.kCurrentArray[kJointNames[Joints.kArmWristRollJoint]] = msg.current_state_array[Joints.kArmWristRollJoint].current_ma
    self.kCurrentArray[kJointNames[Joints.kHandJoint]] = msg.current_state_array[Joints.kHandJoint].current_ma
    # self.kCurrentArray[kJointNames[Joints.kHeadYawJoint]] = msg.current_state_array[Joints.kHeadYawJoint].current_ma
    # self.kCurrentArray[kJointNames[Joints.kHeadPitchJoint]] = msg.current_state_array[Joints.kHeadPitchJoint].current_ma




def main(args=None):
  rclpy.init(args=args)



  node = JointController()

  # node.moveArmRad([0.0, -1.57, 0.0, 0.0, 0.0, 0.0], 0.0 ,3.0)
  # node.moveArmRad([0.0, -1., 0.0, 0.0, 0.0, 0.0], 0.0, 3.0)
  # node.moveHeadRad([1.57, 0.0], 3.0)
  # node.moveHeadRad([0.0, 1.57], 3.0)
  # node.moveAllJointsRad([0.0, -1.57, 0.0, 0.0, 0.0, 0,0], 0.0, [0.0, 0.0], 3.0)
  # node.moveJointRad(Joints.kHandJoint, 3.14, 3.0)
  # node.moveJointRad(Joints.kArmShoulderRollJoint, 0.0, 3.0)
  node.moveToPose('initial_pose', 3.0)
  # node.moveToPose('raise_hand', 3.0)
  # node.moveToPose('detecting_pose', 3.0)
  # node.moveToPose('initial_pose', 3.0)
  # node.moveHandToTargetCoord([0.0, 0.0, 0.2], [0.0, 0.0, 0.0], 5.0)
  # node.moveToPose('initial_pose', 3.0)
  # node.moveHandToTargetTF('target_name', [0.0, 0.0, 0.0], 1.0)
  # node.moveHandToPlaceCoord([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0)
  # node.moveHandToPlaceTF('target_name', [0.0, 0.0, 0.0], 1.0)

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()

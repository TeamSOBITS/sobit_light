#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from math import (radians, degrees, fmod, copysign, pi)

import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def geoQuat2Yaw(geo_quat):
  quat_list = [geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w]

  (roll, pitch, yaw) = euler_from_quaternion(quat_list)

  return yaw


class WheelController(Node):
  def __init__(self, node_name='sobit_light_wheel_controller'):
    super().__init__(node_name)

    # QoS profile for the subscription
    qos_policy = rclpy.qos.QoSProfile(
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        depth=1)
    
    # 
    self.sub_odom_ = self.create_subscription(
        Odometry,
        '/kachaka/odometry/odometry',
        self.callbackOdometry,
        qos_profile=qos_policy,
    )

    self.pub_cmd_vel_ = self.create_publisher(
        Twist,
        '/kachaka/manual_control/cmd_vel',
        qos_profile=qos_policy,
    )

    # Flags
    self.is_running_ = False

    # Variables
    self.curt_odom_ = Odometry()

    # Rate
    self._loop_rate = self.create_rate(20, self.get_clock())

  def callbackOdometry(self, msg):
    self.is_running_ = True
    self.curt_odom_ = msg

  def controlWheelLinear(self, distance):
    while not(self.is_running_):
      if not rclpy.ok():
        self.sub_odom_.destroy()
      self.get_logger().info('Waiting for odometry...')
      rclpy.spin_once(self)

    # Get the initial position
    init_odom = self.curt_odom_
    self.get_logger().info('Initial odometry: %s' % init_odom)

    # Get the current time
    start_time = self.get_clock().now()

    # Initialize values
    output_vel = Twist()
    moved_distance = 0.0
    target_distance = abs(distance)

    # PID Values
    Kp = 0.1
    Ki = 0.4
    Kd = 0.8

    vel_diff = Kp * distance

    while (moved_distance < target_distance):
      # Get the current time
      curr_time = self.get_clock().now()

      # Calculate the elapsed time
      elapsed_time = curr_time - start_time
      elapsed_time = elapsed_time.nanoseconds
      elapsed_time = elapsed_time / 1e9

      vel_linear = 0.0

      # TODO: Fix the PID controller
      if (target_distance < 0.01) or (target_distance - moved_distance < 0.01):
        break
      if (target_distance < 0.69) and (abs(vel_diff) > 1.0):
        vel_diff = copysign(1.0, vel_diff)

      if (target_distance < 0.1):
        vel_linear = Kp * (target_distance + 0.001 - moved_distance) \
                   - Kd * vel_diff \
                   + Ki / 0.8 * (target_distance + 0.001 - moved_distance) * pow(elapsed_time, 2)
      else:
        vel_linear = Kp * (target_distance + 0.001 - moved_distance) \
                   - Kd * vel_diff \
                   + Ki / (8.0 / target_distance) * (target_distance + 0.001 - moved_distance) * pow(elapsed_time, 2)

      output_vel.linear.x = vel_linear if distance > 0 else -vel_linear
      vel_diff = vel_linear

      self.pub_cmd_vel_.publish(output_vel)

      # Calculate the moved distance
      x_diff = self.curt_odom_.pose.pose.position.x - init_odom.pose.pose.position.x
      y_diff = self.curt_odom_.pose.pose.position.y - init_odom.pose.pose.position.y
      moved_distance = pow(pow(x_diff, 2) + pow(y_diff, 2), 0.5)

      # Debug
      self.get_logger().info('[Wheel Control: Linear] Moved distance: %f, Target distance: %f' % (moved_distance, target_distance))
      rclpy.spin_once(self)
    
    # Stop the robot after the target distance is reached
    output_vel.linear.x = 0.0
    self.pub_cmd_vel_.publish(output_vel)

  def controlWheelRotateRad(self, angle_rad):
    while not(self.is_running_):
      if not rclpy.ok():
        self.sub_odom_.destroy()
      self.get_logger().info('Waiting for odometry...')
      rclpy.spin_once(self)

    # Get the initial position
    init_odom = self.curt_odom_
    init_yaw = geoQuat2Yaw(init_odom.pose.pose.orientation)
    self.get_logger().info('Initial odometry: %s' % init_odom)

    # Get the current time
    start_time = self.get_clock().now()

    # Initialize values
    output_vel = Twist()
    moved_angle_rad = 0.0
    target_angle_rad = abs(angle_rad)
    target_angle_deg = degrees(target_angle_rad)

    # PID Values
    Kp = 0.1
    Ki = 0.4
    Kd = 0.8

    vel_diff = Kp * angle_rad

    while (moved_angle_rad < target_angle_rad-0.01):
      rclpy.spin_once(self)

      # Get the current time
      curr_time = self.get_clock().now()

      # Calculate the elapsed time
      elapsed_time = curr_time - start_time
      elapsed_time = elapsed_time.nanoseconds
      elapsed_time = elapsed_time / 1e9
      # print(elapsed_time)

      vel_angular = 0.0

      if (target_angle_deg < 30):
        vel_angular = Kp * (target_angle_rad + 0.001 - moved_angle_rad) \
                   - Kd * vel_diff \
                   + Ki / 0.8 * (target_angle_rad + 0.001 - moved_angle_rad) * pow(elapsed_time, 2)
      else:
        vel_angular = Kp * (target_angle_rad + 0.001 - moved_angle_rad) \
                   - Kd * vel_diff \
                   + Ki / (8.0 / target_angle_rad) * (target_angle_rad + 0.001 - moved_angle_rad) * pow(elapsed_time, 2)

      output_vel.angular.z = vel_angular if angle_rad > 0 else -vel_angular
      vel_diff = vel_angular
      self.pub_cmd_vel_.publish(output_vel)

      # Calculate the moved distance
      curt_yaw = geoQuat2Yaw(self.curt_odom_.pose.pose.orientation)
      self.get_logger().info('Current yaw: %f, Initial yaw: %f' % (curt_yaw, init_yaw))

      angle_diff_rad = curt_yaw - init_yaw
      angle_diff_rad = fmod(angle_diff_rad, 2*pi)
      moved_angle_rad = abs(angle_diff_rad)

      # Debug
      self.get_logger().info('[Wheel Control: Rotate] Moved angle: %f, Target angle: %f' % (moved_angle_rad, target_angle_rad))

      # Sleep
      # _loop_rate.sleep()

    # Stop the robot after the target angle is reached
    output_vel.angular.z = 0.0
    self.pub_cmd_vel_.publish(output_vel)

  def controlWheelRotateDeg(self, angle_deg):
    self.controlWheelRotateRad(radians(angle_deg))


def main(args=None):
  rclpy.init(args=args)

  node = WheelController()

  node.controlWheelLinear(distance=0.5)
  node.controlWheelRotateRad(angle_rad=1.57)
  node.controlWheelRotateDeg(angle_deg=-90)
  node.controlWheelLinear(distance=-0.5)

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()

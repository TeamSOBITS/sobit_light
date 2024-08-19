#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from sobit_light_module import WheelController

def main(args=None):
  print("Initializing ROS2...")
  rclpy.init(args=args)
  print("ROS2 initialized successfully.")

  wheel_controller = None

  try:
    wheel_controller = WheelController()
    print("WheelController created successfully.")

    # Control methods
    wheel_controller.controlWheelLinear(0.5)
    wheel_controller.controlWheelRotateRad(1.5708)
    wheel_controller.controlWheelRotateDeg(-90)
    wheel_controller.controlWheelLinear(-0.5)

  except Exception as e:
    print(f"Error occurred: {e}")

  finally:
    if wheel_controller is not None:
      wheel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
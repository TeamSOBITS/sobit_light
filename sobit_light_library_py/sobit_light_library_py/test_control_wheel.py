#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.executors import ExternalShutdownException

from sobit_light_library_py import WheelController

def main(args=None):
    try:
        with rclpy.init(args=args):
            wheel_controller = WheelController("test_control_wheel")
            print("WheelController created successfully.")

            # Control methods
            wheel_controller.controlWheelLinear(0.5)
            wheel_controller.controlWheelRotateRad(1.5708)
            wheel_controller.controlWheelRotateDeg(-90)
            wheel_controller.controlWheelLinear(-0.5)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()

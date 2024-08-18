#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from sobit_light_module import WheelController

def main(args=None):
    print("Initializing ROS2...")
    try:
        rclpy.init(args=args)
        print("ROS2 initialized successfully.")

        wheel_controller = WheelController("sobit_light_wheel_controller")
        print("WheelController created successfully.")

        # コントロールメソッドの呼び出し
        wheel_controller.controlWheelLinear(1.5)
        wheel_controller.controlWheelRotateRad(1.5708)
        wheel_controller.controlWheelRotateDeg(-90)

    except Exception as e:
        print(f"An error occurred: {str(e)}")

    finally:
        if 'wheel_controller' in locals() and wheel_controller is not None:
            wheel_controller.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()
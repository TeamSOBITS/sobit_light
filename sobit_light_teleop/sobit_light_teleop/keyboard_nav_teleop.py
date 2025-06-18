import sys
import select
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_nav_teleop')
        self.pub = self.create_publisher(Twist, '/sobit_light/manual_control/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.get_logger().info("Use WASD to move, press 'q' to quit")
        twist = Twist()
        while rclpy.ok():
            key = self.get_key()

            # デフォルト: 停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if key == 'w':
                twist.linear.x = 0.5
            elif key == 's':
                twist.linear.x = -0.5
            elif key == 'a':
                twist.angular.z = 1.0
            elif key == 'd':
                twist.angular.z = -1.0
            elif key == 'q':
                break  # 終了

            self.pub.publish(twist)

        # 停止して終了
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

def main():
    rclpy.init()
    node = KeyboardTeleop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy,JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DualShock_Teleop(Node):
    def __init__(self):
        super().__init__('dualshock_teleop')

        joint_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        #publisher
        self.base_pub = self.create_publisher(Twist, 'manual_control/cmd_vel', 10) #kachaka base
        self.joint_pub = self.create_publisher(JointTrajectory,'joint_trajectory_controller/joint_trajectory',joint_qos_profile) #joint
        
        #subscriber
        self.dualshock_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10) #dualshock
        self.joint_sub = self.create_subscription(JointState,'joint_states',self.joint_state_callback,10) #joint_state

        self.timer = self.create_timer(0.01, self.process)  # 20Hzでprocess()を呼ぶ

        self.AXES = {
            "LEFT_X": 0,   # l_stick_x  +:left     -:right 
            "LEFT_Y": 1,   # l_stick_y  +:up,      -:down 
            "L2": 2,       # l2_button  +:no_touch -:push     
            "RIGHT_X": 3,  # r_stick_x  +:left     -:right
            "RIGHT_Y": 4,  # r_stick_y  +:up,      -:down
            "R2": 5,       # r2_button  +:no_touch -:push
            "DPAD_X": 6,   # d_pad_x    +:left     -:right
            "DPAD_Y": 7    # d_pad_y    +:up       -:down
        }

        self.BUTTONS = {
            "CROSS": 0,    # ×_button
            "CIRCLE": 1,   # ○_button
            "TRIANGLE": 2, # △_button
            "SQUARE": 3,   # □_button
            "L1": 4,       # L1_button
            "R1": 5,       # R1_button
            "L2" :6,        # L2_button
            "R2" :7,        # R2_button
            "SHARE": 8,    # share_button
            "OPTION": 9,   # option_button
            "HOME": 10,     # home_button
            "L_STICK": 11,  # l_stick push
            "R_STICK": 12, # r_stick push
        }

        self.JOINTS = {
            'Shoulder_Roll': 'arm_shoulder_roll_joint', 
            'Shoulder_Pitch_sub': 'arm_shoulder_pitch_sub_joint', 
            'Shoulder_Pitch_main': 'arm_shoulder_pitch_joint', 
            'Elbow_Pitch': 'arm_elbow_pitch_joint',
            'Forearm_Roll': 'arm_forearm_roll_joint',
            'Wrist_Pitch': 'arm_wrist_pitch_joint',
            'Wrist_Roll': 'arm_wrist_roll_joint', 
            'Hand': 'hand_joint', 
            'Head_Yaw': 'head_yaw_joint',
            'Head_Pitch': 'head_pitch_joint'
        }
        self.JOINT_LIMITS = {
            'arm_shoulder_roll_joint': (-3.14,3.14 ),
            'arm_shoulder_pitch_sub_joint': (-0.74, 2.11),
            'arm_shoulder_pitch_joint': (-2.11, 0.74),
            'arm_elbow_pitch_joint': (-1.57, 1.57),
            'arm_forearm_roll_joint': (-3.14, 3.14),
            'arm_wrist_pitch_joint': (-1.57, 1.57),
            'arm_wrist_roll_joint': (-3.13, 3.14),
            'hand_joint': (-0.01791787, 0.02901195),
            'head_yaw_joint': (-2.32, 2.32),
            'head_pitch_joint': (-1.26, 1.57),
        }

        self.last_joy_time = self.get_clock().now()
        self.joy_timeout_sec = 3.0
        self.latest_joint_state = {name: 0.0 for name in self.JOINTS.values()}
        self.current_axes = [0.0] *8
        self.current_buttons = [0] *13
        self.move_flag = False
        self.home_pose_sent = False
        self.chage_pose_sent = False
        self.grip_flag = False
        self.grip_pressed = False
        self.send_home_pose()


        

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.latest_joint_state:
                self.latest_joint_state[name] = pos
        self.last_joy_time = self.get_clock().now()
    
    def check_sign(self,value):
        print(value)
        if value > 0.6:
            return 5
        elif value < -0.6:
            return -5
        else:
            return 0

    def joy_callback(self,msg):
        if len(msg.buttons) < max(self.BUTTONS.values()) + 1 or len(msg.axes) < max(self.AXES.values()) + 1:
            self.get_logger().warn("Received Joy message with insufficient length. Ignored.")
            return

        self.current_axes = msg.axes
        self.current_buttons = msg.buttons
        

    def process(self):
        if not hasattr(self, "current_buttons") or not hasattr(self, "current_axes"):
            return  

        now = self.get_clock().now()
        elapsed = (now - self.last_joy_time).nanoseconds * 1e-9
        if elapsed > self.joy_timeout_sec:
            self.get_logger().warn(f"No Joy signal received for {elapsed:.1f} seconds. Please check the connection.")
            return

        buttons = self.current_buttons
        axes = self.current_axes
        accelrate = 1.5 #R2push rate

        # Base
        if buttons[self.BUTTONS["L1"]] == 1:
            self.move_base(axes[self.AXES["LEFT_X"]], axes[self.AXES["LEFT_Y"]], axes[self.AXES["L2"]])
            self.move_flag = True
        elif buttons[self.BUTTONS["L1"]] == 0 and self.move_flag:
            self.move_base(0, 0, 0)
            self.move_flag = False

        # Head
        if buttons[self.BUTTONS["TRIANGLE"]] == 1:
            degree_x = self.check_sign(axes[self.AXES["LEFT_X"]])
            degree_y = self.check_sign(axes[self.AXES["LEFT_Y"]])
            if degree_x != 0:
                self.move_joint(self.JOINTS["Head_Yaw"], degree_x)
            if degree_y != 0:
                self.move_joint(self.JOINTS["Head_Pitch"], -degree_y)

        # Shoulder
        if buttons[self.BUTTONS["SQUARE"]] == 1:
            degree_x = self.check_sign(axes[self.AXES["LEFT_X"]])
            degree_y = self.check_sign(axes[self.AXES["LEFT_Y"]])
            
            if axes[self.AXES["R2"]] == -1:
                degree_x = degree_x*accelrate
                degree_y = degree_y*accelrate
            if degree_x != 0:
                self.move_joint(self.JOINTS["Shoulder_Roll"], -degree_x*0.4)
            if degree_y != 0:
                self.move_joint(self.JOINTS["Shoulder_Pitch_main"], -degree_y*0.4)
        
        # Elbow & Forearm
        if buttons[self.BUTTONS["CROSS"]] == 1:
            degree_x = self.check_sign(axes[self.AXES["LEFT_X"]])
            degree_y = self.check_sign(axes[self.AXES["LEFT_Y"]])
            if axes[self.AXES["R2"]] == -1:
                degree_x = degree_x*accelrate
                degree_y = degree_y*accelrate
            if degree_x != 0:
                self.move_joint(self.JOINTS["Forearm_Roll"], -degree_x*0.4)
            if degree_y != 0:
                self.move_joint(self.JOINTS["Elbow_Pitch"], -degree_y*0.4)

        # Wrist
        if buttons[self.BUTTONS["CIRCLE"]] == 1:
            degree_x = self.check_sign(axes[self.AXES["LEFT_X"]])
            degree_y = self.check_sign(axes[self.AXES["LEFT_Y"]])
            if axes[self.AXES["R2"]] == -1:
                degree_x = degree_x*accelrate
                degree_y = degree_y*accelrate
            if degree_x != 0:
                self.move_joint(self.JOINTS["Wrist_Roll"], -degree_x*0.4)
            if degree_y != 0:
                self.move_joint(self.JOINTS["Wrist_Pitch"], -degree_y*0.4)
        
        # Hand
        degree_x = self.check_sign(axes[self.AXES["RIGHT_X"]])
        if degree_x != 0:
                self.move_joint(self.JOINTS["Hand"], -degree_x*0.6)
        
        # Hand -grap
        if buttons[self.BUTTONS["R_STICK"]] == 1 and not self.grip_pressed:
            self.grip_pressed = True

            if not self.grip_flag:
                self.grip(0.02901195)  
            else:
                self.grip(-0.01791787) 

            self.grip_flag = not self.grip_flag 

        elif buttons[self.BUTTONS["R_STICK"]] == 0:
            self.grip_pressed = False

        #initial_pose
        if buttons[self.BUTTONS["OPTION"]] == 1 and not self.home_pose_sent:
            self.send_home_pose()
            self.home_pose_sent = True
        elif buttons[self.BUTTONS["OPTION"]] == 0:
            self.home_pose_sent = False

            
    def move_base(self, stick_x, stick_y, l2):

        twist = Twist()
        speed = -(l2 - 1.0)*0.5 
        twist.linear.x = stick_y * 0.2 * (speed + 1.0)
        twist.angular.z = stick_x * 1.0 * (speed + 1.0)
        self.base_pub.publish(twist)
    
    def move_joint(self,joint_name,degree):
        current_pos = self.latest_joint_state[joint_name]
        new_pos = current_pos + math.radians(degree)

        if joint_name in self.JOINT_LIMITS:
            min_limit, max_limit = self.JOINT_LIMITS[joint_name]
            new_pos = max(min(new_pos, max_limit), min_limit)

        if joint_name == self.JOINTS["Shoulder_Pitch_main"]:
            sub_joint = self.JOINTS["Shoulder_Pitch_sub"]
            new_sub_pos = -new_pos
            if sub_joint in self.JOINT_LIMITS:
                min_limit, max_limit = self.JOINT_LIMITS[sub_joint]
                if not (min_limit <= new_sub_pos <= max_limit):
                    new_sub_pos = max(min(new_sub_pos, max_limit), min_limit)
                    new_pos = -new_sub_pos

        traj_msg = JointTrajectory()

        if joint_name == self.JOINTS["Shoulder_Pitch_main"]:
            traj_msg.joint_names = [joint_name,sub_joint]
        else:
            traj_msg.joint_names = [joint_name]
        
        point = JointTrajectoryPoint()
        if joint_name == self.JOINTS["Shoulder_Pitch_main"]:
            point.positions = [new_pos,new_sub_pos]
        else:
            point.positions = [new_pos]

        traj_msg.points.append(point)
        
        self.joint_pub.publish(traj_msg)

        if joint_name == self.JOINTS["Shoulder_Pitch_main"]:
            self.get_logger().info(f"{joint_name} を {new_pos:.6f} rad に更新して送信しました")
            self.get_logger().info(f"{sub_joint} を {new_sub_pos:.6f} rad に更新して送信しました")
        else:
            self.get_logger().info(f"{joint_name} を {new_pos:.6f} rad に更新して送信しました")
    
    def send_home_pose(self):
        initial_pose = {
            self.JOINTS['Shoulder_Roll']: 0.0,
            self.JOINTS['Shoulder_Pitch_main']: -1.5708,
            self.JOINTS['Shoulder_Pitch_sub']: 1.5708,
            self.JOINTS['Elbow_Pitch']: 0.0,
            self.JOINTS['Forearm_Roll']: 0.0,
            self.JOINTS['Wrist_Pitch']: 0.0,
            self.JOINTS['Wrist_Roll']: 0.0,
            # self.JOINTS['Hand']: 0.0,
            self.JOINTS['Head_Yaw']: 0.0,
            self.JOINTS['Head_Pitch']: 0.0,
        }

        traj_msg = JointTrajectory()
        traj_msg.joint_names = list(initial_pose.keys())

        point = JointTrajectoryPoint()
        point.positions = list(initial_pose.values())
        point.time_from_start = rclpy.duration.Duration(seconds=5.0).to_msg()

        traj_msg.points.append(point)
        self.joint_pub.publish(traj_msg)
        self.get_logger().info("Initial (HOME) pose command sent.")
        time.sleep(5)

    def grip(self,value):

        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.JOINTS["Hand"]]
        point = JointTrajectoryPoint()
        point.positions = [value]
        point.time_from_start = rclpy.duration.Duration(seconds=0.5).to_msg()

        traj_msg.points.append(point)
        self.joint_pub.publish(traj_msg)
        time.sleep(0.5)
      
def main():
    rclpy.init()
    node = DualShock_Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

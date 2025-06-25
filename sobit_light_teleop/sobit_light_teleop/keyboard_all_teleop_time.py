import sys
import select
import tty
import termios
import rclpy
import time
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HandJointTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_all_teleop')

        # コントローラに登録されている全ジョイント名リスト（順番を覚えておく）
        self.full_joint_names = [
            'arm_shoulder_roll_joint',
            'arm_elbow_pitch_joint',
            'arm_forearm_roll_joint',
            'arm_wrist_pitch_joint',
            'arm_wrist_roll_joint',
            'hand_joint',
            'arm_shoulder_pitch_sub_joint',
            'arm_shoulder_pitch_joint',
            'head_yaw_joint',
            'head_pitch_joint'
        ]

        # 最新ジョイント状態を辞書で管理（初期は0.0で初期化）
        self.latest_joint_state = {name: 0.0 for name in self.full_joint_names}

        # 操作対象ジョイント（はじめはhand_joint）
        self.arm_shoulder_roll_joint = ['arm_shoulder_roll_joint']
        self.arm_elbow_pitch_joint = ['arm_elbow_pitch_joint']
        self.arm_forearm_roll_joint = ['arm_forearm_roll_joint']
        self.arm_wrist_pitch_joint = ['arm_wrist_pitch_joint']
        self.arm_wrist_roll_joint = ['arm_wrist_roll_joint']
        self.target_joints = ['hand_joint']
        self.arm_shoulder_pitch_sub_joint = ['arm_shoulder_pitch_sub_joint']
        self.arm_shoulder_pitch_joint = ['arm_shoulder_pitch_joint']
        self.head_yaw_joint = ['head_yaw_joint']
        self.head_pitch_joint = ['head_pitch_joint']

        # JointStateトピック購読（常に最新値を取得して保存）
        self.joint_sub = self.create_subscription(
            JointState,
            '/sobit_light/joint_states',
            self.joint_state_callback,
            10
        )
        # Cmd_velトピックパブリッシャ
        self.base_pub = self.create_publisher(
            Twist, 
            '/sobit_light/manual_control/cmd_vel', 
            10)
        
        # JointTrajectoryトピックパブリッシャ
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/sobit_light/joint_trajectory_controller/joint_trajectory',
            10)

        self.base_timer_period = 0.01 #second パブリッシュ間隔
        self.base_pub_timer = self.create_timer(self.base_timer_period, self.base_timer_callback)

        # キーボード入力用設定
        self.settings = termios.tcgetattr(sys.stdin)
        self.selected_joint_key = None
        
        self.key_joint_map = {
            'z': 'arm_shoulder_joint',
            'x': 'arm_elbow_or_forearm_joint',
            'c': 'arm_wrist_joint',
            'v': 'head_joint',
            'f': 'hand_joint',
            # 'i': 'UP',
            # 'k': 'DOWN',
            # 'l': 'RIGHT',
            # 'j': 'LEFT',

        }

        self.angle = 1.0 #一回のパブリッシュでのjointのangle
        self.selected_joint_name = None
    
    def joint_state_callback(self, msg):
        # 受信したJointStateの値を最新値として更新
        for name, pos in zip(msg.name, msg.position):
            if name in self.latest_joint_state:
                self.latest_joint_state[name] = pos

    def get_key(self):
        # キーボードから1文字読み込み（ノンブロッキング）
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def base_timer_callback(self):

        twist = Twist()
        # self.get_logger().info("手動ジョイント操作モード")
        # self.get_logger().info("q: 終了")

        # デフォルト: 停止
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        key = self.get_key()

        if key == 'q':
            self.get_logger().info("終了します")
            rclpy.shutdown()
            return 
        elif key in self.key_joint_map:
            self.selected_joint_key = key
            self.selected_joint_name = self.key_joint_map[key]
            self.get_logger().info(f"関節選択: {self.selected_joint_name}")

        elif key == 'w':
            twist.linear.x = 0.5
            #self.base_pub.publish(twist)

        elif key == 's':
            twist.linear.x = -0.5
            #self.base_pub.publish(twist)
  
        elif key == 'a':
            twist.angular.z = 1.0
            #self.base_pub.publish(twist)
         
        elif key == 'd':
            twist.angular.z = -1.0
            #self.base_pub.publish(twist)
        elif key == ' ':
            twist.angular.z = 0
            twist.linear.x = 0

            
        
        elif key == 'i' or key == 'k':  # UP or DOWN

            if self.selected_joint_name == None:
                self.get_logger().info("先に関節を選択してください")
            else:
                if self.selected_joint_name == 'arm_shoulder_joint':
                    joint_name = 'arm_shoulder_pitch_joint'

                if self.selected_joint_name == 'arm_elbow_or_forearm_joint':
                    joint_name = 'arm_elbow_pitch_joint'

                if self.selected_joint_name == 'arm_wrist_joint':
                    joint_name = 'arm_wrist_pitch_joint'

                if self.selected_joint_name == 'head_joint':
                    joint_name = 'head_pitch_joint'

                if key == 'i':
                    self.get_logger().info(f"{joint_name} を 更新して送信しました")
                    self.modify_joint(joint_name, -self.angle)
                if key == 'k':
                    self.get_logger().info(f"{joint_name} を 更新して送信しました")
                    self.modify_joint(joint_name, self.angle)
            
        elif key == 'l' or key == 'j':  #  RIGHT or LEFT
            if self.selected_joint_name == None:
                self.get_logger().info("先に関節を選択してください")
            else:
                if self.selected_joint_name == 'arm_shoulder_joint':
                    joint_name = 'arm_shoulder_roll_joint'

                if self.selected_joint_name == 'arm_elbow_or_forearm_joint':
                    joint_name = 'arm_forearm_roll_joint'

                if self.selected_joint_name == 'arm_wrist_joint':
                    joint_name = 'arm_wrist_roll_joint'

                if self.selected_joint_name == 'head_joint':
                    joint_name = 'head_yaw_joint'

                if self.selected_joint_name == 'hand_joint':
                    joint_name = self.selected_joint_name

                if key == 'l':
                    self.get_logger().info(f"{joint_name} を 更新して送信しました")
                    if joint_name == 'head_yaw_joint':
                        self.modify_joint(joint_name, -self.angle)
                    else:
                        self.modify_joint(joint_name, self.angle)
                if key == 'j':
                    self.get_logger().info(f"{joint_name} を 更新して送信しました")
                    if joint_name == 'head_yaw_joint':
                        self.modify_joint(joint_name, self.angle)
                    else:
                        self.modify_joint(joint_name, -self.angle)
            
        self.base_pub.publish(twist)

    def modify_joint(self, joint_name, degrees):
        # 度からラジアンへ変換
        radians = math.radians(degrees)

        # 最新値に加算して更新
        current_pos = self.latest_joint_state[joint_name]
        new_pos = current_pos + radians
        self.latest_joint_state[joint_name] = new_pos

        if joint_name == "arm_shoulder_pitch_joint":
            sub_joint = "arm_shoulder_pitch_sub_joint"
            # sab_current_pos = self.latest_joint_state[sub_joint]
            new_sub_pos = -new_pos
            self.latest_joint_state[sub_joint] = new_sub_pos

        # JointTrajectoryメッセージ作成（操作ジョイントのみ）
        traj_msg = JointTrajectory()
        if joint_name == "arm_shoulder_pitch_joint":
            traj_msg.joint_names = [joint_name,sub_joint]
        else:
            traj_msg.joint_names = [joint_name]

        point = JointTrajectoryPoint()

        if joint_name == "arm_shoulder_pitch_joint":
            point.positions = [new_pos,new_sub_pos]
        else:
            point.positions = [new_pos]

        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100_000_000  # 0.1秒

        traj_msg.points.append(point)

        self.joint_pub.publish(traj_msg)

        if joint_name == "arm_shoulder_pitch_joint":
            self.get_logger().info(f"{joint_name} を {new_pos:.6f} rad に更新して送信しました")
            self.get_logger().info(f"{sub_joint} を {new_sub_pos:.6f} rad に更新して送信しました")
        else:
            self.get_logger().info(f"{joint_name} を {new_pos:.6f} rad に更新して送信しました")



def main():
    rclpy.init()
    try:
        node = HandJointTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("キーボード割り込みにより終了します。")
    finally:
        # ノードが作成されていれば破棄
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        # ターミナル設定を元に戻す ★ここに追加★
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)



if __name__ == '__main__':
    main()
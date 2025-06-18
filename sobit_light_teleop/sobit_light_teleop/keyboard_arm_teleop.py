import sys
import select
import tty
import termios
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HandJointTeleop(Node):
    def __init__(self):
        super().__init__('hand_arm_teleop')

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

        # JointTrajectoryトピックパブリッシャ
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/sobit_light/joint_trajectory_controller/joint_trajectory',
            10
        )


        # キーボード入力用設定
        self.settings = termios.tcgetattr(sys.stdin)

        self.key_joint_map = {
            'a': ('arm_shoulder_roll_joint', 10.0),
            'z': ('arm_shoulder_roll_joint', -10.0),
            's': ('arm_elbow_pitch_joint', 10.0),
            'x': ('arm_elbow_pitch_joint', -10.0),
            'd': ('arm_forearm_roll_joint', 10.0),
            'c': ('arm_forearm_roll_joint', -10.0),
            'f': ('arm_wrist_pitch_joint', 10.0),
            'v': ('arm_wrist_pitch_joint', -10.0),
            'g': ('arm_wrist_roll_joint', 10.0),
            'b': ('arm_wrist_roll_joint', -10.0),
            'h': ('hand_joint', 10.0),
            'n': ('hand_joint', -10.0),
            # 'j': ('arm_shoulder_pitch_sub_joint', 10.0),
            # 'm': ('arm_shoulder_pitch_sub_joint', -10.0),
            'k': ('arm_shoulder_pitch_joint', 10.0),
            ',': ('arm_shoulder_pitch_joint', -10.0),
            'l': ('head_yaw_joint', 10.0),
            '.': ('head_yaw_joint', -10.0),
            ';': ('head_pitch_joint', 10.0),
            '/': ('head_pitch_joint', -10.0),
        }

        self.run()

    def joint_state_callback(self, msg):
        # 受信したJointStateの値を最新値として更新
        for name, pos in zip(msg.name, msg.position):
            if name in self.latest_joint_state:
                self.latest_joint_state[name] = pos

    def get_key(self):
        # キーボードから1文字読み込み（ノンブロッキング）
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        # メインループ
        while rclpy.ok():
            self.get_logger().info("手動ジョイント操作モード開始")
            self.get_logger().info("q: 終了")
            self.get_logger().info("arm_shoulder_roll_joint a: 増加, z: 減少")
            self.get_logger().info("arm_elbow_pitch_joint s: 増加, x: 減少")
            self.get_logger().info("arm_forearm_roll_joint d: 増加, c: 減少")
            self.get_logger().info("arm_wrist_pitch_joint f: 増加, v: 減少")
            self.get_logger().info("arm_wrist_roll_joint g: 増加, b: 減少")
            self.get_logger().info("hand_joint h: 増加, n: 減少")
            #self.get_logger().info("arm_shoulder_pitch_sub_joint j: 増加, m: 減少") #注意
            self.get_logger().info("arm_shoulder_pitch_joint k: 増加, ,: 減少")
            self.get_logger().info("head_yaw_joint l: 増加, .: 減少")
            self.get_logger().info("head_pitch_joint ;: 増加, /: 減少")
            key = self.get_key()
            if key == 'q':
                self.get_logger().info("終了します")
                break
            elif key in self.key_joint_map:
                joint_name, delta = self.key_joint_map[key]
                self.modify_joint(joint_name, delta)

            rclpy.spin_once(self, timeout_sec=0.1)

    def modify_joint(self, joint_name, degrees):
        # 度からラジアンへ変換
        radians = degrees * 3.1416 / 180.0

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
        point.time_from_start.nanosec = 500_000_000  # 0.5秒

        traj_msg.points.append(point)

        self.joint_pub.publish(traj_msg)

        # 少し待機（パブリッシュ間隔）
        time.sleep(0.3)
        if joint_name == "arm_shoulder_pitch_joint":
            self.get_logger().info(f"{joint_name} を {new_pos:.6f} rad に更新して送信しました")
            self.get_logger().info(f"{sub_joint} を {new_sub_pos:.6f} rad に更新して送信しました")
        else:
            self.get_logger().info(f"{joint_name} を {new_pos:.6f} rad に更新して送信しました")



def main():
    rclpy.init()
    try:
        node = HandJointTeleop()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

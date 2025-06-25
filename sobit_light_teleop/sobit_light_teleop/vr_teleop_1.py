import json
import struct
import socket
import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from trajectory_msgs.msg import JointTrajectory

class QuestInterface(Node):
    def __init__(self):
        super().__init__('vr_teleop')

        self.host = '0.0.0.0'
        self.port = 8000
        self.header_size = 4
        self.last_data = None

        # transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # tranform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publisher
        self.base_pub = self.create_publisher(
            Twist, 
            '/sobit_light/manual_control/cmd_vel', 
            10
        )
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/sobit_light/joint_trajectory_controller/joint_trajectory',
            10
        )

        # subscriber
        # self.joint_sub = self.create_subscription(
        #     JointState,
        #     '/sobit_light/joint_states',
        #     self.joint_state_callback,
        #     10
        # )
        # service

    def receive_message(self, conn, msg_size):
        data = b''
        while len(data) < msg_size:
            chunk = conn.recv(msg_size - len(data))
            if not chunk:
                break
            data += chunk
        return data.decode()
    
    def send_message(self, socket, message):
        message_encoded = message.encode()
        # Create the header with 2 bytes representing the length of the message
        header = len(message_encoded).to_bytes(2, byteorder='big')
        # Send the header followed by the message
        socket.send(header + message_encoded)

    def transform_controller(self, pose):
        trans = np.eye(4)
        trans[:3, :3] = R.from_euler('x', 0.5*np.pi).as_matrix()
        mat = trans.dot(pose)
        trans[:3, :3] = R.from_euler('yz', [np.pi, -0.5*np.pi]).as_matrix()
        mat = mat.dot(trans)
        return mat
    
    def broadcast_tf(self, pose, parent, child):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        mat = np.asarray(pose)
        t.transform.translation.x = mat[0,3]
        t.transform.translation.y = mat[1,3]
        t.transform.translation.z = mat[2,3]

        q = R.from_matrix(mat[:3, :3]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
    
    def move_base(self, x, z, faster=False):
        speed_linear = 0.38 if faster else 0.1
        speed_angular = 1.5 if faster else 0.5
        msg = Twist()
        msg.linear.x = x * speed_linear
        msg.angular.z = z * speed_angular
        self.get_logger().info(f'publish (x,y,z)=({msg.linear.x},{msg.linear.y},{msg.angular.z}')
        self.base_pub.publish(msg)

    def process(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)
        while rclpy.ok():
            conn, addr =server_socket.accept()
            self.get_logger().info("Connection from: " + str(addr))
            while rclpy.ok():
                header = conn.recv(self.header_size)
                if not header:
                    self.get_logger().warning("Connection closed.")
                msg_size = struct.unpack('!I', header)[0]
                data = self.receive_message(conn, msg_size)
                # self.get_logger().info('Received Data')

                if not data:
                    self.get_logger().info("No data received.")
                    continue
                data = json.loads(data)

                # self.get_logger().info(f'Data:{data}')
                for device_id in ["controller_1", "controller_2"]:
                    pose  = data[device_id]["pose"]
                    pose.append([0,0,0,1])
                    updated_pose = self.transform_controller(pose)
                    data[device_id]['pose'] = updated_pose
                    self.broadcast_tf(updated_pose, "odom", device_id)
                data = {"L": data['controller_1'], "R": data['controller_2']}
                if self.last_data is None:
                    self.last_data = data
                    continue
                
                speed_trigger = data["L"]["inputs"]["trigger"] > 0.5

                # self.get_logger().info(f'L_trigger : {data["L"]["inputs"]["trigger"]}')

                if speed_trigger:
                    self.get_logger().info('Pressed button : L_trigger')
                # move base
                if data["L"]["inputs"]["grip_button"]==1:
                        x = data["L"]["inputs"]["trackpad_y"]
                        z = -data["L"]["inputs"]["trackpad_x"]
                        self.move_base(x,z,faster=speed_trigger)
                
                self.last_data = data
                rclpy.spin_once(self, timeout_sec=0.1)
        self.process()


def main():
    rclpy.init()
    node = QuestInterface()
    try:
        node.process()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
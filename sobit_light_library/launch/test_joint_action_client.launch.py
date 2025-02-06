from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = "sobit_light"
    robot_id = 0

    joint_move_action_client_node = Node(
        package="sobit_light_library",
        executable="joint_move_action_client",
        name="joint_move_action_client",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        output="screen",
    )
    joint_pose_action_client_node = Node(
        package="sobit_light_library",
        executable="joint_pose_action_client",
        name="joint_pose_action_client",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        output="screen",
    )
    joint_coord_action_client_node = Node(
        package="sobit_light_library",
        executable="joint_coord_action_client",
        name="joint_coord_action_client",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        output="screen",
    )
    joint_tf_action_client_node = Node(
        package="sobit_light_library",
        executable="joint_tf_action_client",
        name="joint_tf_action_client",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        output="screen",
    )


    return LaunchDescription([
        joint_move_action_client_node,
        # joint_pose_action_client_node,
        # joint_coord_action_client_node,
        # joint_tf_action_client_node,
    ])

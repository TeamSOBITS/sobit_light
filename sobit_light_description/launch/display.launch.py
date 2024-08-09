from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    use_gui = LaunchConfiguration('use_gui', default='True')

    robot_name = "sobit_light"
    package_name = robot_name + "_description"

    urdf_launch_package = FindPackageShare('urdf_launch')
    description_package = FindPackageShare(package_name)
    default_rviz_config_path = PathJoinSubstitution([description_package, 'rviz', 'display.rviz'])

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(name='jsp_gui', default_value='true', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui'))

    ld.add_action(DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))

    # need to manually pass configuration in because of https://github.com/ros2/launch/issues/313
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([urdf_launch_package, 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': package_name,
            'urdf_package_path': PathJoinSubstitution(['robots', robot_name+'_robot.urdf.xacro'])}.items()
    ))

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gui)
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui)
    ))

    # Visualize robot in rviz    
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))
    return ld

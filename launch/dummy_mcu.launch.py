from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # USB 포트 인자

    return LaunchDescription([
        Node(
            package='echo_mcu',
            executable='dummy_serial_node',
            name='dummy_serial_node',
            output='screen'
        ),
        Node(
            package='echo_mcu',
            executable='cmd_node',
            name='cmd_node',
            output='screen'
        ),
        Node(
            package='echo_mcu',
            executable='odom_node',
            name='odom_node',
            output='screen'
        ),
        Node(
            package='echo_mcu',
            executable='tf_node',
            name='tf_node',
            output='screen'
        ),
    ])

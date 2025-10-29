from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # USB 포트 인자
    usb_port_arg = DeclareLaunchArgument(
        'usb_port',
        default_value='/dev/ttyACM0',
        description='Serial USB port for MCU'
    )
    usb_port = LaunchConfiguration('usb_port')

    return LaunchDescription([
        usb_port_arg,

        Node(
            package='echo_mcu',
            executable='serial_node',
            name='serial_node',
            output='screen',
            parameters=[{'usb_port': usb_port}]
        ),
        Node(
            package='echo_mcu',
            executable='cmd_node_v2',
            name='cmd_node_v2',
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

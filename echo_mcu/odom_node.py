#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # 로봇 파라미터
        self.wheel_radius = 0.013
        self.pulses_per_rev = 68600
        self.wheel_base = 0.15

        # 상태 변수
        self.prev_left = None
        self.prev_right = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 퍼블리셔
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 구독: MCU에서 CSV 형식 엔코더 값 수신
        self.create_subscription(String, '/mcu_rx', self.mcu_callback, 10)

    def mcu_callback(self, msg: String):
        try:
            left_str, right_str = msg.data.split(' ')
            left = int(left_str)
            right = int(right_str)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse MCU CSV: {msg.data}")
            return

        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            return

        delta_left = left - self.prev_left
        delta_right = right - self.prev_right
        self.prev_left = left
        self.prev_right = right

        d_left = 2 * math.pi * self.wheel_radius * delta_left / self.pulses_per_rev
        d_right = 2 * math.pi * self.wheel_radius * delta_right / self.pulses_per_rev
        d_center = (d_left + d_right) / 2
        delta_theta = (d_right - d_left) / self.wheel_base

        self.theta += delta_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

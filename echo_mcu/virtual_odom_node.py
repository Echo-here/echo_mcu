#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf_transformations import quaternion_from_euler
import math
import time

class HybridOdomNode(Node):
    def __init__(self):
        super().__init__('hybrid_odom_node')

        # 로봇 파라미터
        self.wheel_radius = 0.013
        self.pulses_per_rev = 68600
        self.wheel_base = 0.15

        # 상태 변수
        self.prev_left = None
        self.left_total = 0.0

        self.fake_right_vel = 0.0
        self.cmd_time = time.time()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 퍼블리셔
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 구독
        self.create_subscription(String, '/mcu_rx', self.mcu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Timer 50Hz
        self.create_timer(0.02, self.update_odom)

        self.get_logger().info("HybridOdomNode Initialized")

    def cmd_callback(self, msg: Twist):
        """오른쪽 바퀴 속도 업데이트 (teleop 기반)"""
        v = msg.linear.x
        w = msg.angular.z
        self.fake_right_vel = v + (self.wheel_base / 2.0) * w
        self.cmd_time = time.time()
        self.get_logger().info(f"[CMD] v={v:.3f}, w={w:.3f}, fake_right_vel={self.fake_right_vel:.3f}")

    def mcu_callback(self, msg: String):
        """왼쪽 바퀴만 MCU에서 수신"""
        try:
            left_str, _ = msg.data.split(',')
            left = int(left_str)
        except:
            self.get_logger().warn(f"MCU data parse failed: {msg.data}")
            return

        if self.prev_left is None:
            self.prev_left = left
            return

        delta_left = left - self.prev_left
        self.prev_left = left

        d_left = 2 * math.pi * self.wheel_radius * delta_left / self.pulses_per_rev
        self.left_total += d_left

        self.get_logger().info(f"[MCU] left={left}, delta_left={delta_left}, d_left={d_left:.6f}, total_left={self.left_total:.6f}")

    def update_odom(self):
        dt = 0.02
        d_left = self.left_total
        d_right = self.fake_right_vel * dt

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        now = self.get_clock().now().to_msg()

        # 로그
        self.get_logger().info(f"[ODOM] dL={d_left:.6f}, dR={d_right:.6f}, x={self.x:.3f}, y={self.y:.3f}, th={self.theta:.3f}")

        # Odometry 퍼블리시
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.odom_pub.publish(odom)

        # 업데이트 후 왼쪽 거리 초기화
        self.left_total = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = HybridOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

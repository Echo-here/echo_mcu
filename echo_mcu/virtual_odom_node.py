#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

class HybridOdomNode(Node):
    def __init__(self):
        super().__init__('hybrid_odom_node')

        # 로봇 파라미터
        self.wheel_radius = 0.065  # 130mm diameter
        self.wheel_base = 0.3

        # 상태 변수
        self.prev_left = None
        self.left_total = 0.0
        self.fake_right_total = 0.0

        self.fake_right_vel = 0.0  # cmd_vel 기반 예측
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscribers
        self.create_subscription(String, '/mcu_rx', self.mcu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publisher & TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(0.02, self.update_odom)  # 50Hz

        self.get_logger().info("HybridOdomNode Initialized")

    def cmd_callback(self, msg: Twist):
        """오른쪽 바퀴 가상 속도 계산"""
        v = msg.linear.x
        w = msg.angular.z
        self.fake_right_vel = v + (self.wheel_base / 2.0) * w
        self.get_logger().info(f"[CMD] linear={v:.3f}, angular={w:.3f}, fake_right_vel={self.fake_right_vel:.3f}")

    def mcu_callback(self, msg: String):
        """왼쪽 엔코더 처리"""
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

        d_left = 2 * math.pi * self.wheel_radius * delta_left / 68600  # pulses per rev
        self.left_total += d_left
        self.get_logger().info(f"[ENC] left={left}, delta_left={delta_left}, left_total={self.left_total:.4f}")

    def update_odom(self):
        now = self.get_clock().now().to_msg()
        dt = 0.02

        d_left = self.left_total
        d_right = self.fake_right_vel * dt
        self.fake_right_total += d_right

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # 콘솔 로그
        self.get_logger().info(f"[ODOM] x={self.x:.3f}, y={self.y:.3f}, th={self.theta:.3f}, dL={d_left:.4f}, dR={d_right:.4f}")

        # Odometry publish
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.odom_pub.publish(odom)

        # TF broadcast
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # 누적 리셋
        self.left_total = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = HybridOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

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

class HybridOdomNode(Node):
    def __init__(self):
        super().__init__('hybrid_odom_node')

        # 로봇 파라미터
        self.wheel_radius = 0.065   # 왼쪽 바퀴 실제 radius [m]
        self.pulses_per_rev = 68600
        self.wheel_base = 0.3       # 좌우 바퀴 간 거리 [m]

        # 상태 변수
        self.prev_left = None
        self.left_total = 0.0

        self.fake_right_vel = 0.0
        self.cmd_time = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscribers
        self.create_subscription(String, '/mcu_rx', self.mcu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer (50Hz)
        self.timer = self.create_timer(0.02, self.update_odom)

        self.get_logger().info("HybridOdomNode Initialized")

    def cmd_callback(self, msg: Twist):
        """cmd_vel 기반 오른쪽 바퀴 속도 설정"""
        self.fake_right_vel = msg.linear.x + (self.wheel_base / 2.0) * msg.angular.z
        self.cmd_time = self.get_clock().now().nanoseconds * 1e-9  # seconds

    def mcu_callback(self, msg: String):
        """왼쪽 바퀴 엔코더 수신"""
        try:
            left_str, _ = msg.data.split(',')  # 오른쪽은 MCU 안씀
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
        self.left_total = d_left  # 최근 이동 거리만 저장

    def update_odom(self):
        now = self.get_clock().now().to_msg()
        dt = 0.02  # timer 주기

        # 오른쪽 바퀴 거리 계산: 직진이면 왼쪽과 동일
        if abs(self.fake_right_vel) < 1e-4:  # cmd_vel 없음
            d_right = self.left_total
        else:
            d_right = self.fake_right_vel * dt

        d_left = self.left_total
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # pose 업데이트
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Odometry publish
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

        # TF broadcast
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = HybridOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

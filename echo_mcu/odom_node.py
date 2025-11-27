#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # 로봇 파라미터
        self.wheel_radius = 0.065
        self.pulses_per_rev = 68600
        self.wheel_base = 0.3

        # 상태 변수
        self.prev_left = None
        self.prev_right = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 최신 엔코더 값 저장
        self.latest_left = None
        self.latest_right = None

        # 퍼블리셔 & TF 브로드캐스터
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        # MCU 엔코더 구독 (큐 20)
        self.create_subscription(String, '/mcu_rx', self.mcu_callback, 20)

        # Timer: 10Hz
        self.timer = self.create_timer(0.1, self.publish_odom)

        self.get_logger().info("OdomNode initialized (10Hz, queue_size=20)")

    def mcu_callback(self, msg: String):
        try:
            left_str, right_str = msg.data.split(', ')
            self.latest_left = int(left_str)
            self.latest_right = int(right_str)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse MCU CSV: {msg.data}")

    def publish_odom(self):
        if self.latest_left is None or self.latest_right is None:
            return

        if self.prev_left is None:
            self.prev_left = self.latest_left
            self.prev_right = self.latest_right
            return

        delta_left = self.latest_left - self.prev_left
        delta_right = self.latest_right - self.prev_right
        self.prev_left = self.latest_left
        self.prev_right = self.latest_right

        # 거리 계산
        d_left = 2 * math.pi * self.wheel_radius * delta_left / self.pulses_per_rev
        d_right = 2 * math.pi * self.wheel_radius * delta_right / self.pulses_per_rev
        d_center = (d_left + d_right) / 2
        delta_theta = (d_right - d_left) / self.wheel_base

        # 위치 업데이트
        self.theta += delta_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Odometry 메시지 작성
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

        # TF 브로드캐스트
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
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

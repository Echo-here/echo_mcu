#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import math

class CmdVelOdomNode(Node):
    def __init__(self):
        super().__init__('cmdvel_odom_node')

        # 로봇 파라미터
        self.wheel_base = 0.3  # 바퀴 사이 간격 (m)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 좌/우 바퀴 가상 거리
        self.d_left = 0.0
        self.d_right = 0.0

        # 퍼블리셔 & TF 브로드캐스터
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # cmd_vel 구독
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Timer 50Hz
        self.timer = self.create_timer(0.02, self.update_odom)
        self.last_cmd = Twist()  # 마지막 cmd_vel 저장

        self.get_logger().info("CmdVelOdomNode Initialized")

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg

    def update_odom(self):
        dt = 0.02
        # 속도 스케일링 (실제 로봇 속도 대비)
        scale = 0.7  # 50% 정도로 줄임
        v = self.last_cmd.linear.x * scale
        w = self.last_cmd.angular.z * scale

        # 좌/우 바퀴 거리 계산
        self.d_left  = (v - w * self.wheel_base / 2.0) * dt
        self.d_right = (v + w * self.wheel_base / 2.0) * dt

        # 중심 이동 & 회전
        d_center = (self.d_left + self.d_right)/2.0
        delta_theta = (self.d_right - self.d_left)/self.wheel_base

        self.theta += delta_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Odometry publish
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
    node = CmdVelOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

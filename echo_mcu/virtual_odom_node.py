#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf_transformations import quaternion_from_euler
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # 로봇 파라미터
        self.wheel_base = 0.15  # 로봇 바퀴 간 거리(m)

        # 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_cmd = Twist()

        # 속도 스케일링 (실제 로봇과 맞추기)
        self.scale_linear = 0.5   # 직진 속도 보정
        self.scale_angular = 0.6  # 회전 속도 보정

        # 퍼블리셔
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 구독: cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # 타이머
        self.timer = self.create_timer(0.02, self.update_odom)  # 50Hz

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg

    def update_odom(self):
        dt = 0.02  # 50Hz
        v = self.last_cmd.linear.x * self.scale_linear
        w = self.last_cmd.angular.z * self.scale_angular

        # 좌표 계산
        self.theta += w * dt
        d_center = v * dt
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

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

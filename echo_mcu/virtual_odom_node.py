#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf_transformations import quaternion_from_euler
import math

class CmdVelOdomNode(Node):
    def __init__(self):
        super().__init__('cmdvel_odom_node')

        # 로봇 파라미터
        self.wheel_radius = 0.065  # m
        self.wheel_base = 0.3      # m (휠베이스 30cm)

        # 상태 변수
        self.fake_right_vel = 0.0
        self.fake_left_vel = 0.0  # cmd_vel 기반
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer
        self.timer = self.create_timer(0.02, self.update_odom)  # 50Hz

    def cmd_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # 오른쪽/왼쪽 바퀴 속도 계산 (cmd_vel 기반)
        self.fake_right_vel = v + (self.wheel_base / 2.0) * w
        self.fake_left_vel  = v - (self.wheel_base / 2.0) * w

    def update_odom(self):
        dt = 0.02
        # 바퀴 이동 거리
        d_right = self.fake_right_vel * dt
        d_left  = self.fake_left_vel  * dt

        # Differential drive 계산
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Odometry 메시지 발행
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

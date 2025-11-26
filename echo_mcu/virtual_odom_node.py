#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class CmdVelOdomNode(Node):
    def __init__(self):
        super().__init__('cmdvel_odom_node')

        # 로봇 파라미터
        self.wheel_base = 0.3  # 미터

        # 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 현재 cmd_vel 값
        self.v = 0.0
        self.w = 0.0

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer: 50Hz
        self.timer = self.create_timer(0.02, self.update_odom)
        self.get_logger().info("CmdVelOdomNode Initialized")

    def cmd_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_odom(self):
        dt = 0.02  # 타이머 주기

        # 중심 이동 거리와 회전량
        d_center = self.v * dt
        d_theta = self.w * dt

        # 누적 pose 계산
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        now = self.get_clock().now().to_msg()

        # Odometry publish
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

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

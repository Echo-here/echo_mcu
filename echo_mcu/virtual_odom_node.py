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
        self.wheel_radius = 0.065        # 130mm diameter
        self.pulses_per_rev = 68600      # confirmed PPR
        self.wheel_base = 0.3           # later calibrate

        # 상태 변수
        self.prev_left = None
        self.left_total = 0

        self.fake_right_vel = 0.0
        self.cmd_time = time.time()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscribers
        self.create_subscription(String, '/mcu_rx', self.mcu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer
        self.timer = self.create_timer(0.02, self.update_odom)  # 50Hz odom update

        # 디버그용
        self.get_logger().info("HybridOdomNode Initialized")

    def cmd_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        v_r = v + (self.wheel_base / 2.0) * w
        self.fake_right_vel = v_r
        self.cmd_time = time.time()

        self.get_logger().debug(f"[CMD] v={v:.3f}, w={w:.3f}, fake_right_vel={v_r:.3f}")

    def mcu_callback(self, msg: String):
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

        self.d_left = 2 * math.pi * self.wheel_radius * delta_left / self.pulses_per_rev

        self.get_logger().debug(
            f"[MCU] left={left}, delta_left={delta_left}, d_left={self.d_left:.6f}"
        )

    def update_odom(self):
        now = self.get_clock().now().to_msg()

        d_left = getattr(self, "d_left", 0.0)

        dt = 0.02
        d_right = self.fake_right_vel * dt

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # 디버그 로그
        self.get_logger().info(
            f"[ODOM] dL={d_left:.6f}, dR={d_right:.6f}, x={self.x:.3f}, y={self.y:.3f}, th={self.theta:.3f}"
        )

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = now

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.odom_pub.publish(odom)

        self.d_left = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = HybridOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

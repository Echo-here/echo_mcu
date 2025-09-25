#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from math import sin, cos

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        self.rx_sub = self.create_subscription(String, '/mcu_rx', self.rx_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

    def rx_callback(self, msg: String):
        line = msg.data
        parts = line.split(',')
        if len(parts) != 3:
            return
        try:
            x, y, theta = map(float, parts)
            odom = Odometry()
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = sin(theta / 2.0)
            odom.pose.pose.orientation.w = cos(theta / 2.0)
            self.odom_pub.publish(odom)
        except:
            return

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdNode(Node):
    def __init__(self):
        super().__init__('cmd_node')

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.tx_pub = self.create_publisher(String, '/mcu_tx', 10)

    def cmd_callback(self, msg: Twist):
        data = f"{msg.linear.x},{msg.linear.y},{msg.angular.z}"
        self.tx_pub.publish(String(data=data))

def main(args=None):
    rclpy.init(args=args)
    node = CmdNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

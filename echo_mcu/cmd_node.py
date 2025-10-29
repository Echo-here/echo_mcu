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
        duty = int(min(max(abs(msg.linear.x), abs(msg.angular.z)) * 10, 9))

        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            data = "s"
            self.tx_pub.publish(String(data=data))
            self.get_logger().info(f"Publish STOP: {data}")
            return

        if msg.linear.x > 0:
            l_dir, r_dir = 'f', 'f'
        elif msg.linear.x < 0:
            l_dir, r_dir = 'b', 'b'
        else:
            l_dir, r_dir = 's', 's'

        if msg.angular.z > 0: 
            l_dir, r_dir = 'b', 'f'
        elif msg.angular.z < 0:
            l_dir, r_dir = 'f', 'b'

        cmd = f"{l_dir} {duty} {r_dir} {duty}"

        self.tx_pub.publish(String(data=cmd))

        self.get_logger().info(f"Publish to MCU: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

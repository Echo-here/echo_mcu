#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class DummySerialNode(Node):
    def __init__(self):
        super().__init__('dummy_serial_node')

        self.get_logger().info("Running in dummy mode (no serial)")

        # Publisher
        self.rx_pub = self.create_publisher(String, '/mcu_rx', 10)
        
        # Subscriber
        self.tx_sub = self.create_subscription(String, '/mcu_tx', self.tx_callback, 10)

        # Timer: 주기적으로 dummy 데이터 발행
        self.timer = self.create_timer(0.1, self.publish_dummy_data)

    def tx_callback(self, msg: String):
        # 받은 데이터 로그 출력
        self.get_logger().info(f"TX received: {msg.data}")

    def publish_dummy_data(self):
        # 임의의 더미 데이터 생성
        dummy_value = random.randint(0, 100)
        line = f"dummy:{dummy_value}"
        self.get_logger().info(f"Publishing: {line}")
        self.rx_pub.publish(String(data=line))


def main(args=None):
    rclpy.init(args=args)
    node = DummySerialNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

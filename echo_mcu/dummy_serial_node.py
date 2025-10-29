#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class DummySerialNode(Node):
    """
    더미 MCU 데이터를 CSV 형식으로 발행
    """

    def __init__(self):
        super().__init__('dummy_serial_node')

        self.get_logger().info("Running in dummy mode (no serial)")

        # Publisher
        self.rx_pub = self.create_publisher(String, '/mcu_rx', 10)
        
        # Subscriber (옵션)
        self.tx_sub = self.create_subscription(String, '/mcu_tx', self.tx_callback, 10)

        # 상태 변수: 엔코더 누적값
        self.left_count = 0
        self.right_count = 0

        # Timer: 주기적으로 dummy 데이터 발행
        self.timer = self.create_timer(0.1, self.publish_dummy_data)

    def tx_callback(self, msg: String):
        # 받은 데이터 로그 출력
        self.get_logger().info(f"TX received: {msg.data}")

    def publish_dummy_data(self):
        # 누적 엔코더 값 증가 (임의의 움직임)
        self.left_count += random.randint(600, 700)
        #self.right_count += random.randint(5, 10)

        # CSV 형식으로 발행
        line = f"{self.left_count},{self.right_count}"
        self.get_logger().info(f"Publishing: {line}")
        self.rx_pub.publish(String(data=line))


def main(args=None):
    rclpy.init(args=args)
    node = DummySerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

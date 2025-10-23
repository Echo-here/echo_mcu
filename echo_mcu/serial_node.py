#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        usb_port = self.get_parameter('usb_port').value

        # 시리얼 열기
        self.ser = serial.Serial(usb_port, 115200, timeout=0.1)
        self.get_logger().info(f"Using USB port: {usb_port}")

        # ROS2 토픽
        self.rx_pub = self.create_publisher(String, '/mcu_rx', 10)
        self.tx_sub = self.create_subscription(String, '/mcu_tx', self.tx_callback, 10)

        # TX 큐
        self.tx_queue = []
        self.tx_lock = threading.Lock()

        # RX 스레드
        self.rx_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.rx_thread.start()

        # TX 스레드
        self.tx_thread = threading.Thread(target=self.write_serial_loop, daemon=True)
        self.tx_thread.start()

    # ROS2에서 들어온 명령 큐에 넣기
    def tx_callback(self, msg: String):
        with self.tx_lock:
            self.tx_queue.append(msg.data)

    # 비동기 TX 처리
    def write_serial_loop(self):
        while rclpy.ok():
            data = None
            with self.tx_lock:
                if self.tx_queue:
                    data = self.tx_queue.pop(0)
            if data:
                self.ser.write((data + '\n').encode('utf-8'))
                self.ser.flush()
                self.get_logger().info(f"TX: {data}")
            time.sleep(0.001)  # 1ms 쉬고 루프

    # 비동기 RX 처리
    def read_serial_loop(self):
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.rx_pub.publish(String(data=line))
                    self.get_logger().info(f"RX: {line}")
            time.sleep(0.001)  # 1ms 쉬고 루프


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

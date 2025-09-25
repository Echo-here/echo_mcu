#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        usb_port = self.get_parameter('usb_port').value

        self.ser = serial.Serial(usb_port, 115200, timeout=0.1)
        self.get_logger().info(f"Using USB port: {usb_port}")

        self.rx_pub = self.create_publisher(
            String, '/mcu_rx', 10
        )
        self.tx_sub = self.create_subscription(
            String, '/mcu_tx', self.tx_callback, 10
        )

        self.timer = self.create_timer(0.05, self.read_serial)
        init_msg = String()
        init_msg.data = "q" 
        self.tx_callback(init_msg)

    def tx_callback(self, msg: String):
        self.ser.write((msg.data + '\n').encode('utf-8'))

    def read_serial(self):
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f"Line: {line}")
            if line:
                self.rx_pub.publish(String(data=line))
    

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

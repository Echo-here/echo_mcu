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
        v = msg.linear.x       # 전진 속도
        w = msg.angular.z      # 회전 속도
        k = 0.8                # 조향 민감도
        max_speed = 1.0        # 최대 속도 [m/s]
        min_duty = 1           # 최소 duty 보장

        # --- 정지 ---
        if abs(v) < 1e-3 and abs(w) < 1e-3:
            self.tx_pub.publish(String(data="s"))
            self.get_logger().info("STOP")
            return

        # --- 제자리 회전 처리 (linear 거의 0일 때) ---
        if abs(v) < 1e-3 and abs(w) > 1e-3:
            if w > 0:
                l_dir, r_dir = 'b', 'f'
            else:
                l_dir, r_dir = 'f', 'b'
            duty = max(int(min(abs(w) * 9, 9)), min_duty)
            cmd = f"{l_dir} {duty} {r_dir} {duty}"
            self.tx_pub.publish(String(data=cmd))
            self.get_logger().info(f"SPIN -> {cmd}")
            return

        # --- 일반 속도 분배 ---
        v_l = v * (1 - k * w)
        v_r = v * (1 + k * w)

        duty_l = max(int(min(abs(v_l / max_speed * 9), 9)), min_duty)
        duty_r = max(int(min(abs(v_r / max_speed * 9), 9)), min_duty)

        l_dir = 'f' if v_l >= 0 else 'b'
        r_dir = 'f' if v_r >= 0 else 'b'

        cmd = f"{l_dir} {duty_l} {r_dir} {duty_r}"
        self.tx_pub.publish(String(data=cmd))
        self.get_logger().info(f"v={v:.2f}, w={w:.2f} -> {cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

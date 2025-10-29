def cmd_callback(self, msg: Twist):
    L = 0.3   # 바퀴 간 거리 [m]
    v = msg.linear.x
    w = msg.angular.z

    # 후륜 보정 계수 (0~1 사이, 낮을수록 회전 덜 민감)
    k_turn = 0.7  

    v_l = v - (w * L / 2) * k_turn
    v_r = v + (w * L / 2) * k_turn

    # 속도 -> duty 변환
    max_speed = 1.0
    duty_l = int(min(max(abs(v_l / max_speed * 9), 0), 9))
    duty_r = int(min(max(abs(v_r / max_speed * 9), 0), 9))

    l_dir = 'f' if v_l >= 0 else 'b'
    r_dir = 'f' if v_r >= 0 else 'b'

    if v == 0.0 and w == 0.0:
        self.tx_pub.publish(String(data="s"))
        self.get_logger().info("STOP")
        return

    cmd = f"{l_dir} {duty_l} {r_dir} {duty_r}"
    self.tx_pub.publish(String(data=cmd))
    self.get_logger().info(f"Publish to MCU: {cmd}")

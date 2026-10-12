#!/usr/bin/env python3
"""
08_pot_to_servo.py — หมุนโพเทนชิโอมิเตอร์บนบอร์ด → servo หนีบ (s1) หมุนตาม (ตัวอย่างอ่านค่า → คำนวณ → สั่ง)
รันที่ : 🟡 Jetson   ก่อนรัน: agent ค้างไว้ · source ~/ros_env.sh · มีคนเฝ้าแขนกล · Ctrl+C = จบ
ช่วงมุมจำกัดไว้ MIN_DEG–MAX_DEG เพื่อไม่ให้แขนชนสุด · s2 ค้างที่ HOLD_S2 · ส่งไม่ถี่เกิน 5 ครั้ง/วินาที
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

MIN_DEG, MAX_DEG = 0.0, 85.0                                  # ช่วงของตัวหนีบ (ตาม preset open/close)
HOLD_S2, HOLD_S3 = 103.0, 90.0


class PotServo(Node):
    def __init__(self):
        super().__init__('example_pot_to_servo')
        self.pub = self.create_publisher(Float32MultiArray, '/servo/cmd', 10)
        self.create_subscription(Float32MultiArray, '/pot/raw', self.on_pot, 10)
        self.last_sent, self.last_deg = 0.0, None

    def on_pot(self, m):
        raw = m.data[1]                                       # 0–4095
        deg = MIN_DEG + (MAX_DEG - MIN_DEG) * max(0.0, min(1.0, raw / 4095.0))
        now = time.time()
        if now - self.last_sent < 0.2:                        # ≤ 5 Hz
            return
        if self.last_deg is not None and abs(deg - self.last_deg) < 2.0:   # เปลี่ยนน้อยกว่า 2° ไม่ต้องส่ง
            return
        self.pub.publish(Float32MultiArray(data=[deg, HOLD_S2, HOLD_S3]))
        self.last_sent, self.last_deg = now, deg
        self.get_logger().info(f'POT {raw:.0f} → s1 {deg:.0f}°')


def main():
    rclpy.init()
    node = PotServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

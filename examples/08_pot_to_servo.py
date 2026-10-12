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

MIN_DEG, MAX_DEG = 0.0, 85.0                                  # ช่วงมุมของตัวหนีบ (ตาม preset open/close)
HOLD_S2, HOLD_S3 = 103.0, 90.0                                # มุมคงที่ของตัวยก (s2) และตัวที่ 3


class PotServo(Node):
    """อ่าน POT → แปลงเป็นมุม → สั่ง servo หนีบ"""

    def __init__(self):
        super().__init__('example_pot_to_servo')
        self.pub = self.create_publisher(Float32MultiArray, '/servo/cmd', 10)      # ช่องส่ง: servo
        self.create_subscription(Float32MultiArray, '/pot/raw', self.on_pot, 10)   # ช่องรับ: ค่า POT
        self.last_sent, self.last_deg = 0.0, None             # เวลาและมุมที่ส่งครั้งล่าสุด (ใช้กันส่งถี่/ซ้ำ)

    def on_pot(self, m):
        """ถูกเรียกเมื่อได้ค่า POT ใหม่ (~3 ครั้ง/วินาที):
        แปลงค่า ADC 0–4095 เป็นมุม MIN_DEG–MAX_DEG แบบเส้นตรง แล้วส่งถ้าจำเป็น"""
        raw = m.data[1]                                       # m.data = [แรงดัน, ADC] → ใช้ค่า ADC (0–4095)
        ratio = max(0.0, min(1.0, raw / 4095.0))              # สัดส่วน 0.0–1.0 ของการหมุน POT
        deg = MIN_DEG + (MAX_DEG - MIN_DEG) * ratio           # แปลงสัดส่วนเป็นมุม
        now = time.time()
        if now - self.last_sent < 0.2:                        # ส่งไม่ถี่เกิน 5 ครั้ง/วินาที (ไม่ให้สาย serial ตัน)
            return
        if self.last_deg is not None and abs(deg - self.last_deg) < 2.0:   # มุมเปลี่ยนน้อยกว่า 2° ไม่ต้องส่ง (กัน servo สั่น)
            return
        self.pub.publish(Float32MultiArray(data=[deg, HOLD_S2, HOLD_S3]))   # ต้องส่งครบ 3 ค่าเสมอ
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

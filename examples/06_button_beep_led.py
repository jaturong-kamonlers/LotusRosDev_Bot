#!/usr/bin/env python3
"""
06_button_beep_led.py — node แบบ "รับแล้วตอบ": กดปุ่มบนบอร์ด → บี๊บ + LED ติด · ปล่อย → LED ดับ
รันที่ : 🟡 Jetson   ก่อนรัน: agent ค้างไว้ · source ~/ros_env.sh · Ctrl+C = จบ
แสดงหลัก subscriber + publisher ใน node เดียว (callback ถูกเรียกทุกครั้งที่มีข้อความใหม่)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class ButtonBeep(Node):
    """รับสถานะปุ่ม → ส่งคำสั่ง LED และเสียงกลับไปที่บอร์ด"""

    def __init__(self):
        super().__init__('example_button_beep')
        self.led = self.create_publisher(Bool, '/led/state', 10)                   # ช่องส่ง: LED
        self.buz = self.create_publisher(Float32MultiArray, '/buzzer/play', 10)    # ช่องส่ง: เสียง
        self.create_subscription(Bool, '/button/state', self.on_button, 10)       # ช่องรับ: ปุ่ม
        self.count = 0                                                            # นับจำนวนครั้งที่กด
        self.get_logger().info('พร้อม — กดปุ่มบนบอร์ดได้เลย')

    def on_button(self, msg: Bool):
        """ถูกเรียกเมื่อปุ่มเปลี่ยนสถานะ: msg.data = True (เพิ่งกด) / False (เพิ่งปล่อย)"""
        self.led.publish(Bool(data=msg.data))                 # LED ตามสถานะปุ่ม: กด = ติด · ปล่อย = ดับ
        if msg.data:                                          # เฉพาะตอนกด (ไม่บี๊บตอนปล่อย)
            self.count += 1
            freq = 600.0 + 100.0 * (self.count % 8)           # กดแต่ละครั้งเสียงสูงขึ้น 100 Hz วนทุก 8 ครั้ง
            self.buz.publish(Float32MultiArray(data=[freq, 120.0]))   # บี๊บสั้น 120 ms
            self.get_logger().info(f'กดครั้งที่ {self.count} → บี๊บ {freq:.0f} Hz')


def main():
    rclpy.init()
    node = ButtonBeep()
    try:
        rclpy.spin(node)                                      # รอฟังปุ่มไปเรื่อย ๆ
    except KeyboardInterrupt:
        pass
    finally:
        node.led.publish(Bool(data=False))                    # จบแล้วดับ LED
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

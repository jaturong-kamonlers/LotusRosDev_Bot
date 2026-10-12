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
    def __init__(self):
        super().__init__('example_button_beep')
        self.led = self.create_publisher(Bool, '/led/state', 10)
        self.buz = self.create_publisher(Float32MultiArray, '/buzzer/play', 10)
        self.create_subscription(Bool, '/button/state', self.on_button, 10)
        self.count = 0
        self.get_logger().info('พร้อม — กดปุ่มบนบอร์ดได้เลย')

    def on_button(self, msg: Bool):
        self.led.publish(Bool(data=msg.data))                 # LED ตามสถานะปุ่ม
        if msg.data:                                          # true = เพิ่งกด
            self.count += 1
            freq = 600.0 + 100.0 * (self.count % 8)           # กดแต่ละครั้งเสียงสูงขึ้น
            self.buz.publish(Float32MultiArray(data=[freq, 120.0]))
            self.get_logger().info(f'กดครั้งที่ {self.count} → บี๊บ {freq:.0f} Hz')


def main():
    rclpy.init()
    node = ButtonBeep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.led.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

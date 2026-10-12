#!/usr/bin/env python3
"""
00_template_node.py — แม่แบบสำหรับเริ่ม node ใหม่ (คัดลอกแล้วเปลี่ยนชื่อไฟล์ก่อนแก้)
  cp 00_template_node.py ../my_first_node.py      แล้วแก้ใน VS Code
รันที่ : 🟡 Jetson   ก่อนรัน: agent ค้างไว้ · terminal: source ~/ros_env.sh · หรือกด F5 ใน VS Code (ใช้ .vscode/ros.env)
โครง  : publisher (สั่งบอร์ด) + subscriber (อ่านบอร์ด) + timer (ทำงานเป็นจังหวะ) — ลบส่วนที่ไม่ใช้ได้
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray     # เพิ่ม/ลด import ตามชนิดข้อความที่ใช้


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')                                       # ① ชื่อ node (ห้ามซ้ำ)
        self.led = self.create_publisher(Bool, '/led/state', 10)          # ② ช่องส่งคำสั่ง
        self.create_subscription(Bool, '/button/state', self.on_button, 10)   # ③ ช่องรับข้อมูล
        self.create_timer(1.0, self.on_timer)                             # ④ เรียกทุก 1 วินาที
        self.ticks = 0
        self.get_logger().info('my_node เริ่มทำงาน')

    def on_button(self, msg: Bool):                                       # ⑤ ทำงานเมื่อมีข้อความใหม่
        self.get_logger().info(f'ปุ่ม: {"กด" if msg.data else "ปล่อย"}')

    def on_timer(self):                                                   # ⑥ ทำงานตามจังหวะ
        self.ticks += 1
        self.led.publish(Bool(data=self.ticks % 2 == 1))                  # ตัวอย่าง: สลับไฟทุกวินาที


def main():
    rclpy.init()
    node = MyNode()
    try:
        rclpy.spin(node)                                                  # วนรอ callback จนกด Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        node.led.publish(Bool(data=False))                                # ⑦ เก็บกวาดก่อนจบ
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

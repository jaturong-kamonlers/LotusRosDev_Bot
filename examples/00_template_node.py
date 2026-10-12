#!/usr/bin/env python3
"""
00_template_node.py — แม่แบบสำหรับเริ่ม node ใหม่ (คัดลอกแล้วเปลี่ยนชื่อไฟล์ก่อนแก้)
  cp 00_template_node.py ../my_first_node.py      แล้วแก้ใน VS Code
รันที่ : 🟡 Jetson   ก่อนรัน: เปิด agent ค้างไว้ (bash ~/agent.sh) · terminal: source ~/ros_env.sh · หรือกด F5 ใน VS Code
โครง  : publisher (สั่งบอร์ด) + subscriber (อ่านบอร์ด) + timer (ทำงานเป็นจังหวะ) — ลบส่วนที่ไม่ใช้ได้
ผลลัพธ์: LED บนบอร์ดกะพริบทุก 1 วินาที · กดปุ่มบนบอร์ดแล้วมีข้อความ "ปุ่ม: กด" ขึ้นใน terminal
"""
# ── นำเข้าไลบรารี ─────────────────────────────────────────────
import rclpy                                          # ไลบรารีหลักของ ROS 2 สำหรับ Python
from rclpy.node import Node                           # คลาสแม่ของทุก node — node ของเราจะ "สืบทอด" จากคลาสนี้
from std_msgs.msg import Bool, Float32MultiArray      # ชนิดข้อความ: Bool = จริง/เท็จ · Float32MultiArray = รายการตัวเลขทศนิยม
                                                      # (ต้องตรงกับที่บอร์ดใช้ — ดูตารางบทที่ 14.1)


class MyNode(Node):
    """node ของเรา: ทุกอย่างที่ node ทำ (ส่ง รับ ตั้งเวลา) เขียนไว้ในคลาสนี้"""

    def __init__(self):
        """ทำงานครั้งเดียวตอนสร้าง node — ใช้ "ลงทะเบียน" ช่องส่ง ช่องรับ และตัวจับเวลา"""
        super().__init__('my_node')                                       # ① ตั้งชื่อ node (ห้ามซ้ำกับ node อื่นที่รันอยู่)
        self.led = self.create_publisher(Bool, '/led/state', 10)          # ② ช่องส่ง: ส่ง Bool ไป topic /led/state (10 = ความยาวคิว)
        self.create_subscription(Bool, '/button/state', self.on_button, 10)   # ③ ช่องรับ: เมื่อมีข้อความใน /button/state ให้เรียก on_button
        self.create_timer(1.0, self.on_timer)                             # ④ ตัวจับเวลา: เรียก on_timer ทุก 1.0 วินาที
        self.ticks = 0                                                    # ตัวแปรนับรอบ (เก็บไว้ใน self เพื่อใช้ข้ามฟังก์ชันได้)
        self.get_logger().info('my_node เริ่มทำงาน')                       # พิมพ์ข้อความลง terminal แบบมีเวลา/ชื่อ node กำกับ

    def on_button(self, msg: Bool):                                       # ⑤ callback ของช่องรับ
        """ถูกเรียกอัตโนมัติทุกครั้งที่บอร์ดส่งสถานะปุ่มมา
        msg.data = True เมื่อกด · False เมื่อปล่อย (บอร์ดส่งเฉพาะตอนสถานะเปลี่ยน)"""
        self.get_logger().info(f'ปุ่ม: {"กด" if msg.data else "ปล่อย"}')

    def on_timer(self):                                                   # ⑥ callback ของตัวจับเวลา
        """ถูกเรียกทุก 1 วินาที: สลับ LED ติด/ดับ"""
        self.ticks += 1
        on = (self.ticks % 2 == 1)                                        # รอบคี่ = ติด · รอบคู่ = ดับ
        self.led.publish(Bool(data=on))                                   # สร้างข้อความ Bool แล้วส่งออกไปที่บอร์ด


def main():
    """จุดเริ่มโปรแกรม: เปิด ROS 2 → สร้าง node → วนรอ → ปิดให้เรียบร้อย"""
    rclpy.init()                                                          # เริ่มระบบ ROS 2 ในโปรแกรมนี้ (ต้องเรียกก่อนสร้าง node)
    node = MyNode()                                                       # สร้าง node (เรียก __init__ ด้านบน)
    try:
        rclpy.spin(node)                                                  # วนรอเหตุการณ์ (ข้อความเข้า/ครบเวลา) ไปเรื่อย ๆ จนกด Ctrl+C
    except KeyboardInterrupt:                                             # กด Ctrl+C → เข้ามาตรงนี้แทนที่จะ error
        pass
    finally:                                                              # ⑦ ทำเสมอก่อนจบ ไม่ว่าจะจบแบบไหน
        node.led.publish(Bool(data=False))                                # เก็บกวาด: ดับ LED
        node.destroy_node()                                               # คืนทรัพยากรของ node
        rclpy.shutdown()                                                  # ปิดระบบ ROS 2


if __name__ == '__main__':                                                # เมื่อรันไฟล์นี้ตรง ๆ (python3 ไฟล์.py) ให้เรียก main()
    main()

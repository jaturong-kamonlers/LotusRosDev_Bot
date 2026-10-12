#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════════
# s00_template.py — แม่แบบแบบเรียงลำดับ (ไม่มี class) สำหรับเริ่ม node ใหม่
#   คัดลอกก่อนแก้:  cp s00_template.py ../my_simple_node.py
# ทำงาน : LED กะพริบทุก 1 วินาที · กดปุ่มบนบอร์ดแล้วพิมพ์ "ปุ่ม: กด/ปล่อย"
# รันที่ : 🟡 Jetson · เปิด agent ค้างไว้ (bash ~/agent.sh) · source ~/ros_env.sh
# เทียบแบบ class : ../00_template_node.py (ผลลัพธ์เหมือนกัน)
# ═══════════════════════════════════════════════════════════════════
import time
import rclpy
from std_msgs.msg import Bool

# ── ขั้นที่ 1: เริ่ม ROS 2 และสร้าง node ─────────────────────────
rclpy.init()
node = rclpy.create_node('my_node')                      # ชื่อ node (ห้ามซ้ำกับตัวอื่นที่รันอยู่)

# ── ขั้นที่ 2: สร้างช่องส่ง (publisher) ─────────────────────────
led = node.create_publisher(Bool, '/led/state', 10)      # ส่ง Bool ไปที่ LED ของบอร์ด

# ── ขั้นที่ 3: สร้างช่องรับ (subscriber) ─────────────────────────
#   ROS 2 ส่งข้อความเข้ามาทาง "ฟังก์ชันรับ" เสมอ — ฟังก์ชันนี้ทำแค่เก็บค่าลงตัวแปร
button = None                                            # ค่าปุ่มล่าสุด (None = ยังไม่มีข้อมูล)
def on_button(msg):
    global button                                        # global = แก้ตัวแปร button ด้านนอกฟังก์ชัน
    button = msg.data                                    # True = กด · False = ปล่อย
node.create_subscription(Bool, '/button/state', on_button, 10)

# ── ขั้นที่ 4: loop หลัก (เหมือน loop() ของ Arduino) ──────────────
time.sleep(1.0)                                          # รอ discovery
print('my_node เริ่มทำงาน (Ctrl+C = จบ)')
last_button = None                                       # ค่าปุ่มรอบก่อน (ใช้ดูว่าเปลี่ยนไหม)
led_on = False
next_blink = time.time()                                 # เวลาที่ต้องสลับ LED ครั้งถัดไป
try:
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)          # รับข้อความที่เข้ามา (ถ้ามี) — ต้องมีใน loop เสมอ

        if button != last_button:                        # ปุ่มเปลี่ยนสถานะ
            last_button = button
            print('ปุ่ม:', 'กด' if button else 'ปล่อย')

        if time.time() >= next_blink:                    # ครบ 1 วินาที → สลับ LED
            led_on = not led_on
            led.publish(Bool(data=led_on))
            next_blink = time.time() + 1.0
except KeyboardInterrupt:                                # กด Ctrl+C
    pass

# ── ขั้นที่ 5: เก็บกวาดแล้วปิด ──────────────────────────────────
led.publish(Bool(data=False))                            # ดับ LED
node.destroy_node()
rclpy.shutdown()

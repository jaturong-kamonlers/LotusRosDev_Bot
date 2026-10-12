#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════════
# s06_button_beep_led.py — กดปุ่ม → บี๊บ + LED ติด · ปล่อย → LED ดับ (แบบเรียงลำดับ)
# กดแต่ละครั้งเสียงสูงขึ้น 100 Hz (วนทุก 8 ครั้ง) · Ctrl+C = จบ
# เทียบแบบ class : ../06_button_beep_led.py
# ═══════════════════════════════════════════════════════════════════
import rclpy
from std_msgs.msg import Bool, Float32MultiArray

rclpy.init()
node = rclpy.create_node('example_button_beep')
led = node.create_publisher(Bool, '/led/state', 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
buzzer = node.create_publisher(Float32MultiArray, '/buzzer/play', 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ

button = None                          # ค่าปุ่มล่าสุด
def on_button(msg):
    global button
    button = msg.data
node.create_subscription(Bool, '/button/state', on_button, 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ

last = None                            # ค่าปุ่มรอบก่อน
count = 0                              # จำนวนครั้งที่กด
print('พร้อม — กดปุ่มบนบอร์ดได้เลย (Ctrl+C = จบ)')
try:
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)       # รับข้อความ
        if button is None or button == last:          # ยังไม่มีข้อมูล หรือปุ่มไม่เปลี่ยน → ข้าม
            continue
        last = button
        led.publish(Bool(data=button))                # LED ตามปุ่ม: กด = ติด · ปล่อย = ดับ
        if button:                                    # เฉพาะตอนกด
            count += 1
            freq = 600.0 + 100.0 * (count % 8)   # ความถี่ 600–1300 Hz ไล่ขึ้นทีละ 100 Hz · วนกลับทุก 8 ครั้งที่กด
            buzzer.publish(Float32MultiArray(data=[freq, 120.0]))   # [ความถี่ Hz, ความยาว 120 ms]
            print('กดครั้งที่', count, '→ บี๊บ', int(freq), 'Hz')
except KeyboardInterrupt:
    pass

led.publish(Bool(data=False))
node.destroy_node()
rclpy.shutdown()

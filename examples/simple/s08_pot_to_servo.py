#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════════
# s08_pot_to_servo.py — หมุน POT บนบอร์ด → servo หนีบ (s1) หมุนตาม (แบบเรียงลำดับ)
# ช่วงมุม 0–85° · s2 ค้าง 103° · ส่งไม่ถี่เกิน 5 ครั้ง/วินาที · มีคนเฝ้าแขนกล · Ctrl+C = จบ
# topic : /pot/raw [V, ADC 0–4095] → /servo/cmd [s1, s2, s3]
# เทียบแบบ class : ../08_pot_to_servo.py
# ═══════════════════════════════════════════════════════════════════
import time
import rclpy
from std_msgs.msg import Float32MultiArray

MIN_DEG, MAX_DEG = 0.0, 85.0   # ช่วงมุมตัวหนีบ s1: 0° = อ้า · 85° = หนีบ (ตาม preset open/close)
HOLD_S2, HOLD_S3 = 103.0, 90.0   # s2 ยกค้าง 103° (ท่า home) · s3 = 90° กลาง (ไม่ใช้)

rclpy.init()
node = rclpy.create_node('example_pot_to_servo')
servo = node.create_publisher(Float32MultiArray, '/servo/cmd', 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ

pot_raw = None                                   # ค่า ADC ของ POT ล่าสุด
def on_pot(msg):
    global pot_raw
    pot_raw = msg.data[1]
node.create_subscription(Float32MultiArray, '/pot/raw', on_pot, 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ

last_deg = None
last_sent = 0.0
try:
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)   # รอข้อความเข้าไม่เกิน 0.05 วิ แล้ววนรอบใหม่ (≈20 รอบ/วินาที)
        if pot_raw is None:                      # ยังไม่ได้ค่า
            continue
        ratio = max(0.0, min(1.0, pot_raw / 4095.0))       # สัดส่วน 0–1 ของการหมุน
        deg = MIN_DEG + (MAX_DEG - MIN_DEG) * ratio        # แปลงเป็นมุม
        if time.time() - last_sent < 0.2:                  # ไม่ส่งถี่เกิน 5 ครั้ง/วินาที
            continue
        if last_deg is not None and abs(deg - last_deg) < 2.0:   # เปลี่ยนน้อยกว่า 2° ไม่ส่ง
            continue
        servo.publish(Float32MultiArray(data=[deg, HOLD_S2, HOLD_S3]))
        print(f'POT {pot_raw:.0f} → s1 {deg:.0f}°')
        last_deg = deg
        last_sent = time.time()
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()

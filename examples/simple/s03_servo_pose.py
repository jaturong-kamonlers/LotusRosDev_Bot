#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════════
# s03_servo_pose.py — สั่งแขนกลด้วยชื่อท่าหรือมุม (แบบเรียงลำดับ)
# ใช้   : python3 s03_servo_pose.py home  ·  python3 s03_servo_pose.py 60 120 90  ·  python3 s03_servo_pose.py demo
# topic : /servo/cmd  (Float32MultiArray = [s1 หนีบ, s2 ยก, s3] องศา 0–180 · ต้องครบ 3 ค่า)
# ⚠️ มีคนเฝ้าแขนกล · เริ่มจากมุมกลาง · ห้ามรันพร้อมภารกิจ/Tuner
# เทียบแบบ class : ../03_servo_pose.py
# ═══════════════════════════════════════════════════════════════════
import json, os, sys, time
import rclpy
from std_msgs.msg import Float32MultiArray

# ── อ่านท่าจากไฟล์ preset (ไฟล์เดียวกับ Tuner · เก็บเป็น [s1, s2]) ──
presets = {'home': [0.0, 103.0, 90.0]}                   # ค่าตั้งต้นถ้าอ่านไฟล์ไม่ได้
try:
    data = json.load(open(os.path.expanduser('~/servo_presets.json')))
    presets = {}
    for name in data:
        presets[name] = [float(data[name][0]), float(data[name][1]), 90.0]   # s3 = 90 (ไม่ใช้)
except (OSError, ValueError):
    pass

rclpy.init()
node = rclpy.create_node('example_servo')
servo = node.create_publisher(Float32MultiArray, '/servo/cmd', 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
time.sleep(1.0)

def move(angles):
    """ส่งมุม 3 ค่า (ตัดให้อยู่ 0–180 ก่อน)"""
    a = []
    for x in angles:
        a.append(max(0.0, min(180.0, float(x))))   # ตัดมุมให้อยู่ 0–180° (ช่วงที่ servo หมุนได้)
    servo.publish(Float32MultiArray(data=a))
    print('servo →', a)

args = sys.argv[1:] or ['home']                          # ไม่พิมพ์อะไร = ท่า home
if args[0] == 'demo':
    for name in presets:                                 # ไล่ทุกท่า
        print('ท่า', name)
        move(presets[name])
        time.sleep(1.0)
elif args[0] in presets:
    move(presets[args[0]])
elif len(args) == 3:   # พิมพ์มุมมา 3 ค่า = [s1 หนีบ, s2 ยก, s3]
    move(args)
else:
    print('ท่าที่มี:', ', '.join(presets), '  หรือใส่มุม 3 ค่า เช่น 60 120 90')
time.sleep(0.3)   # รอ 0.3 วิ ให้ข้อความถึงบอร์ดก่อนปิดโปรแกรม

node.destroy_node()
rclpy.shutdown()

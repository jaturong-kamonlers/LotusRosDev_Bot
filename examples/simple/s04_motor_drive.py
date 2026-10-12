#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════════
# s04_motor_drive.py — เดินหน้า → หยุด → หมุนซ้าย → หยุด (แบบเรียงลำดับ)
# ใช้   : python3 s04_motor_drive.py [ความเร็ว m/s] [วินาที]     เช่น  python3 s04_motor_drive.py 0.10 2
# topic : /cmd_vel  (Twist: linear.x = เดินหน้า m/s · angular.z = หมุน rad/s บวก = ซ้าย)
# ⚠️ ครั้งแรกยกล้อลอย · มีคนเฝ้า · Ctrl+C = หยุดทันที
# หลัก  : เฟิร์มแวร์หยุดล้อเองถ้าเงียบเกิน 200 ms → ต้องส่งซ้ำ 20 ครั้ง/วินาที
# เทียบแบบ class : ../04_motor_drive.py
# ═══════════════════════════════════════════════════════════════════
import sys, time
import rclpy
from geometry_msgs.msg import Twist

MAX_LIN, MAX_ANG = 0.25, 1.2                             # เพดานความปลอดภัย
speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

rclpy.init()
node = rclpy.create_node('example_motor_drive')
cmd = node.create_publisher(Twist, '/cmd_vel', 10)
time.sleep(1.0)

def drive(vx, wz, seconds):
    """ส่งความเร็วเดิมซ้ำทุก 0.05 วิ (20 Hz) นาน seconds วินาที"""
    t = Twist()
    t.linear.x = max(-MAX_LIN, min(MAX_LIN, float(vx)))  # จำกัดไม่ให้เกินเพดาน
    t.angular.z = max(-MAX_ANG, min(MAX_ANG, float(wz)))
    end = time.time() + seconds
    while time.time() < end:
        cmd.publish(t)
        time.sleep(0.05)

def stop():
    """ส่งความเร็ว 0 ซ้ำ 10 ครั้ง ให้แน่ใจว่าล้อหยุด"""
    for i in range(10):
        cmd.publish(Twist())                             # Twist() เปล่า = ทุกค่าเป็น 0
        time.sleep(0.05)

# ── ตรวจความปลอดภัยก่อนขยับ ─────────────────────────────────────
if node.count_publishers('/cmd_vel') > 1:                # มีโปรแกรมอื่นส่งล้ออยู่ → คำสั่งจะตีกัน
    print('❌ มี node อื่นส่ง /cmd_vel อยู่ (Nav2/teleop/ภารกิจ) — ปิดก่อน')
elif input('⚠️ หุ่นจะเคลื่อนที่ — มีคนเฝ้าและพื้นที่ว่างแล้ว? พิมพ์ y แล้ว Enter: ').strip().lower() == 'y':
    try:
        print('เดินหน้า', speed, 'm/s', secs, 'วิ')
        drive(speed, 0.0, secs)
        stop()
        time.sleep(0.5)
        print('หมุนซ้าย 0.6 rad/s', secs, 'วิ')
        drive(0.0, 0.6, secs)
    except KeyboardInterrupt:
        print('หยุด (Ctrl+C)')
    stop()                                               # ไม่ว่าจบแบบไหน ต้องหยุดล้อ

node.destroy_node()
rclpy.shutdown()

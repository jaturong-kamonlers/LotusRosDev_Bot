#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════════
# s05_read_sensors.py — อ่านเซนเซอร์ทุกตัวแล้วพิมพ์สรุปทุก 1 วินาที (แบบเรียงลำดับ · ไม่สั่งหุ่น)
# topic : /button/state (Bool) · /ir/raw [Lv,Rv,Lraw,Rraw] · /pot/raw [V,raw]
#         /odom/unfiltered (Odometry) · /imu/data (Imu)
# เทียบแบบ class : ../05_read_sensors.py
# ═══════════════════════════════════════════════════════════════════
import math, time
import rclpy
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

rclpy.init()
node = rclpy.create_node('example_read_sensors')

# ── ตัวแปรเก็บค่าล่าสุด (None = ยังไม่ได้รับ) ──────────────────
button = None
ir = None          # [แรงดันซ้าย, แรงดันขวา, ADC ซ้าย, ADC ขวา]
pot = None         # [แรงดัน, ADC]
odom = None        # (x, y, ความเร็วเดินหน้า)
gyro_z = None      # ความเร็วหมุน rad/s
count = 0          # จำนวนข้อความ odom ที่ได้ใน 1 วินาที

# ── ฟังก์ชันรับ: ทำแค่เก็บค่าลงตัวแปร ───────────────────────────
def on_button(msg):
    global button; button = msg.data
def on_ir(msg):
    global ir; ir = list(msg.data)
def on_pot(msg):
    global pot; pot = list(msg.data)
def on_odom(msg):
    global odom, count
    p = msg.pose.pose.position
    odom = (p.x, p.y, msg.twist.twist.linear.x)
    count += 1
def on_imu(msg):
    global gyro_z; gyro_z = msg.angular_velocity.z

node.create_subscription(Bool, '/button/state', on_button, 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
node.create_subscription(Float32MultiArray, '/ir/raw', on_ir, 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
node.create_subscription(Float32MultiArray, '/pot/raw', on_pot, 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
node.create_subscription(Odometry, '/odom/unfiltered', on_odom, 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
node.create_subscription(Imu, '/imu/data', on_imu, 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ

# ── loop หลัก: รับข้อความตลอด · ครบ 1 วินาทีพิมพ์สรุป ────────────
next_print = time.time() + 1.0
try:
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)   # รอข้อความเข้าไม่เกิน 0.05 วิ แล้ววนรอบใหม่ (≈20 รอบ/วินาที)
        if time.time() >= next_print:
            line = 'ปุ่ม ' + {None: '?', True: 'กด', False: 'ปล่อย'}[button]
            line += f' | IR L {ir[0]:.2f}V R {ir[1]:.2f}V' if ir else ' | IR -'
            line += f' | POT {pot[0]:.2f}V' if pot else ' | POT -'
            line += f' | odom x={odom[0]:+.2f} y={odom[1]:+.2f} v={odom[2]:+.2f}' if odom else ' | odom -'
            line += f' | gyro_z {math.degrees(gyro_z):+.1f}°/s' if gyro_z is not None else ' | gyro -'
            line += f'   [odom {count} ข้อความ/วินาที]'
            print(line)
            count = 0
            next_print = time.time() + 1.0
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()

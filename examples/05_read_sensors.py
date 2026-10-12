#!/usr/bin/env python3
"""
05_read_sensors.py — อ่านเซนเซอร์ทุกตัวของบอร์ดแล้วพิมพ์สรุปทุก 1 วินาที (ไม่สั่งอะไรหุ่น)
รันที่ : 🟡 Jetson (หรือ 💻 HOST ถ้า DDS ข้ามเครื่องใช้ได้)   ก่อนรัน: agent ค้างไว้ · source ~/ros_env.sh
topic ที่อ่าน:
  /button/state      std_msgs/Bool                 true = กดอยู่ (ส่งเฉพาะตอนสถานะเปลี่ยน)
  /ir/raw            std_msgs/Float32MultiArray    [แรงดันซ้าย V, แรงดันขวา V, ADC ซ้าย, ADC ขวา]
  /pot/raw           std_msgs/Float32MultiArray    [แรงดัน V, ADC 0–4095]
  /odom/unfiltered   nav_msgs/Odometry             ตำแหน่ง/ความเร็วจาก encoder (~3 Hz)
  /imu/data          sensor_msgs/Imu               ใช้ได้แค่ angular_velocity และ linear_acceleration (MPU6050 ไม่มีเข็มทิศ)
"""
import math                                           # ใช้แปลงเรเดียน → องศา
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class Sensors(Node):
    """node ที่ "ฟัง" อย่างเดียว: รับค่าเซนเซอร์ 5 ชนิด เก็บค่าล่าสุดไว้ แล้วสรุปทุกวินาที"""

    def __init__(self):
        super().__init__('example_read_sensors')
        # ค่าล่าสุดของแต่ละเซนเซอร์ (None = ยังไม่เคยได้รับ)
        self.v = {'button': None, 'ir': None, 'pot': None, 'odom': None, 'gyro_z': None}
        self.n = {k: 0 for k in self.v}                       # นับจำนวนข้อความที่ได้ในแต่ละวินาที
        # ช่องรับ 5 ช่อง — lambda m: … คือฟังก์ชันสั้น ๆ ที่รับข้อความ m แล้วเก็บค่าที่ต้องการ
        self.create_subscription(Bool, '/button/state', lambda m: self.got('button', m.data), 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
        self.create_subscription(Float32MultiArray, '/ir/raw', lambda m: self.got('ir', list(m.data)), 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
        self.create_subscription(Float32MultiArray, '/pot/raw', lambda m: self.got('pot', list(m.data)), 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
        self.create_subscription(Odometry, '/odom/unfiltered', self.on_odom, 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
        self.create_subscription(Imu, '/imu/data', lambda m: self.got('gyro_z', m.angular_velocity.z), 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ
        self.create_timer(1.0, self.report)                   # สรุปผลทุก 1 วินาที

    def got(self, key, value):
        """เก็บค่าล่าสุดของเซนเซอร์ key และนับว่าได้ข้อความเพิ่ม 1"""
        self.v[key] = value
        self.n[key] += 1

    def on_odom(self, m):
        """ดึงเฉพาะค่าที่สนใจจากข้อความ Odometry (ซึ่งมีหลายสิบค่า):
        ตำแหน่ง x, y (เมตร) และความเร็วเดินหน้า/หมุน"""
        p, t = m.pose.pose.position, m.twist.twist
        self.got('odom', (p.x, p.y, t.linear.x, t.angular.z))

    def report(self):
        """พิมพ์สรุปค่าล่าสุดทุกเซนเซอร์ในบรรทัดเดียว แล้วรีเซ็ตตัวนับ"""
        v = self.v
        ir = f'IR L {v["ir"][0]:.2f}V R {v["ir"][1]:.2f}V' if v['ir'] else 'IR -'
        pot = f'POT {v["pot"][0]:.2f}V' if v['pot'] else 'POT -'
        od = f'odom x={v["odom"][0]:+.2f} y={v["odom"][1]:+.2f} v={v["odom"][2]:+.2f}' if v['odom'] else 'odom -'
        gz = f'gyro_z {math.degrees(v["gyro_z"]):+.1f}°/s' if v['gyro_z'] is not None else 'gyro -'
        btn = {None: 'ปุ่ม ?', True: 'ปุ่ม กด', False: 'ปุ่ม ปล่อย'}[v['button']]
        rate = ' '.join(f'{k}:{c}' for k, c in self.n.items())   # จำนวนข้อความ/วินาที — ควรได้ ~3 (ปุ่มได้ 0 ถ้าไม่ได้กด)
        self.get_logger().info(f'{btn} | {ir} | {pot} | {od} | {gz}   [msg/s {rate}]')
        self.n = {k: 0 for k in self.n}


def main():
    rclpy.init()
    node = Sensors()
    try:
        rclpy.spin(node)                                      # วนรับข้อความจนกด Ctrl+C (subscriber/timer ทำงานได้เพราะบรรทัดนี้)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

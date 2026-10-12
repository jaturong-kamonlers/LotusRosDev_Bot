#!/usr/bin/env python3
"""
03_servo_pose.py — สั่งแขนกล (servo) ด้วยชื่อท่า หรือมุมโดยตรง ผ่าน /servo/cmd
รันที่ : 🟡 Jetson   ก่อนรัน: agent ค้างไว้ · source ~/ros_env.sh · มีคนเฝ้าแขนกล
ใช้     : python3 03_servo_pose.py home              (ท่าจาก ~/servo_presets.json)
          python3 03_servo_pose.py 60 120 90         (มุม s1 s2 s3 เป็นองศา)
          python3 03_servo_pose.py demo              (ไล่ทุกท่าในไฟล์ preset)
ข้อความ: std_msgs/msg/Float32MultiArray  data = [s1, s2, s3]  0–180°  ต้องครบ 3 ค่าเสมอ
          s1 = หนีบ (GPIO32) · s2 = ยก (GPIO33) · s3 = สำรอง (GPIO5)
⚠️ เริ่มจากมุมกลางก่อน · มุมสุด (0/180) อาจดันแขนชนจน servo ร้อน · ห้ามรันพร้อมภารกิจ/Tuner
"""
import json, os, sys, time                            # json = อ่านไฟล์ preset · os = จัดการ path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

PRESET_FILE = os.path.expanduser('~/servo_presets.json')     # ไฟล์เดียวกับที่ Tuner ใช้ เก็บท่าเป็น [s1, s2]
S3_DEFAULT = 90.0                                             # servo ตัวที่ 3 ไม่ได้ใช้ ให้อยู่กลางไว้


def load_presets():
    """อ่านท่าทั้งหมดจากไฟล์ preset → คืนค่าเป็น {ชื่อท่า: [s1, s2, s3]}
    ถ้าอ่านไฟล์ไม่ได้ (ไม่มีไฟล์/รูปแบบผิด) คืนเฉพาะท่า home ค่าตั้งต้น"""
    try:
        return {k: [float(v[0]), float(v[1]), S3_DEFAULT] for k, v in json.load(open(PRESET_FILE)).items()}
    except (OSError, ValueError):
        return {'home': [0.0, 103.0, 90.0]}   # ท่าสำรองถ้าอ่าน ~/servo_presets.json ไม่ได้: s1 หนีบ 0°=อ้า · s2 ยก 103° · s3 90°=กลาง (ไม่ใช้)


class Servo(Node):
    """node ที่ส่งมุมไปให้ servo ทั้ง 3 ตัวของบอร์ด"""

    def __init__(self):
        super().__init__('example_servo')
        self.pub = self.create_publisher(Float32MultiArray, '/servo/cmd', 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ

    def move(self, angles):
        """ส่งมุม [s1, s2, s3] ไปที่บอร์ด — จำกัดให้อยู่ 0–180° ก่อนส่ง กันค่าผิดพลาด"""
        a = [max(0.0, min(180.0, float(x))) for x in angles]  # min/max = ตัดค่าที่เกินช่วงทิ้ง
        self.pub.publish(Float32MultiArray(data=a))
        self.get_logger().info(f'servo → s1={a[0]:.0f}° s2={a[1]:.0f}° s3={a[2]:.0f}°')


def main():
    """ดูว่าผู้ใช้พิมพ์อะไรต่อท้าย: demo / ชื่อท่า / มุม 3 ค่า แล้วสั่งตามนั้น"""
    presets = load_presets()
    args = sys.argv[1:] or ['home']                           # ไม่พิมพ์อะไรต่อท้าย = ท่า home
    rclpy.init()
    node = Servo()
    time.sleep(1.0)                                           # รอ discovery
    try:
        if args[0] == 'demo':                                 # ไล่ทุกท่าในไฟล์
            for name, ang in presets.items():
                node.get_logger().info(f'ท่า {name}')
                node.move(ang)
                time.sleep(1.0)                               # รอ servo หมุนไปถึง + ไม่ส่งถี่เกินไป
        elif args[0] in presets:                              # พิมพ์ชื่อท่าที่มีในไฟล์
            node.move(presets[args[0]])
        elif len(args) == 3:                                  # พิมพ์มุม 3 ค่า
            node.move(args)
        else:                                                 # พิมพ์ผิด → บอกตัวเลือกที่มี
            print('ท่าที่มี:', ', '.join(presets), '  หรือใส่มุม 3 ค่า เช่น 60 120 90')
        time.sleep(0.3)                                       # ให้ข้อความสุดท้ายส่งออกก่อนปิด
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

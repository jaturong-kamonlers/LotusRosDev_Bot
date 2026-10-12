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
import json, os, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

PRESET_FILE = os.path.expanduser('~/servo_presets.json')     # ไฟล์เดียวกับที่ Tuner ใช้ [s1, s2]
S3_DEFAULT = 90.0


def load_presets():
    try:
        return {k: [float(v[0]), float(v[1]), S3_DEFAULT] for k, v in json.load(open(PRESET_FILE)).items()}
    except (OSError, ValueError):
        return {'home': [0.0, 103.0, 90.0]}


class Servo(Node):
    def __init__(self):
        super().__init__('example_servo')
        self.pub = self.create_publisher(Float32MultiArray, '/servo/cmd', 10)

    def move(self, angles):
        a = [max(0.0, min(180.0, float(x))) for x in angles]  # จำกัด 0–180 ก่อนส่ง
        self.pub.publish(Float32MultiArray(data=a))
        self.get_logger().info(f'servo → s1={a[0]:.0f}° s2={a[1]:.0f}° s3={a[2]:.0f}°')


def main():
    presets = load_presets()
    args = sys.argv[1:] or ['home']
    rclpy.init()
    node = Servo()
    time.sleep(1.0)
    try:
        if args[0] == 'demo':
            for name, ang in presets.items():
                node.get_logger().info(f'ท่า {name}')
                node.move(ang)
                time.sleep(1.0)                               # ส่งไม่ถี่เกิน + รอ servo ไปถึง
        elif args[0] in presets:
            node.move(presets[args[0]])
        elif len(args) == 3:
            node.move(args)
        else:
            print('ท่าที่มี:', ', '.join(presets), '  หรือใส่มุม 3 ค่า เช่น 60 120 90')
        time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

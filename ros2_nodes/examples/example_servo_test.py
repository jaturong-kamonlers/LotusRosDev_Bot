#!/usr/bin/env python3
"""
lotus_servo_node.py — ตัวอย่างควบคุม Servo1
ลำดับ: 0 องศา (1 วิ) → 90 องศา (1 วิ) → 0 องศา
Topic: /servo/cmd (std_msgs/Float32MultiArray) [s1_deg, s2_deg, s3_deg]
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class LotusServoNode(Node):
    def __init__(self):
        super().__init__('lotus_servo_node')
        self.pub = self.create_publisher(
            Float32MultiArray, '/servo/cmd', 10)
        # สถานะ servo ปัจจุบัน (ค่าเริ่มต้น 90 องศา)
        self.s1 = 90.0
        self.s2 = 90.0
        self.s3 = 90.0
        self.get_logger().info('LotusServoNode started')

    def set_servo(self, s1: float = None, s2: float = None, s3: float = None):
        """
        ส่งคำสั่ง servo — ถ้าไม่ระบุตัวไหน ใช้ค่าเดิม
        s1, s2, s3: องศา 0–180
        """
        if s1 is not None: self.s1 = float(max(0, min(180, s1)))
        if s2 is not None: self.s2 = float(max(0, min(180, s2)))
        if s3 is not None: self.s3 = float(max(0, min(180, s3)))
        msg = Float32MultiArray()
        msg.data = [self.s1, self.s2, self.s3]
        self.pub.publish(msg)
        self.get_logger().info(f'Servo: S1={self.s1:.0f} S2={self.s2:.0f} S3={self.s3:.0f} deg')

def main(args=None):
    rclpy.init(args=args)
    node = LotusServoNode()

    time.sleep(1.0)  # รอ node เชื่อมต่อ

    # ── ตัวอย่าง: Servo1 เท่านั้น S2/S3 อยู่ที่ 90 ──────
    node.get_logger().info('=== Servo1 demo: 0 → 90 → 0 ===')

    # ขั้น 1: S1 → 0 องศา
    node.get_logger().info('Step 1: S1 = 0 deg')
    node.set_servo(s1=0.0)
    time.sleep(1.0)

    # ขั้น 2: S1 → 90 องศา
    node.get_logger().info('Step 2: S1 = 90 deg')
    node.set_servo(s1=90.0)
    time.sleep(1.0)

    # ขั้น 3: S1 → 0 องศา อีกครั้ง
    node.get_logger().info('Step 3: S1 = 0 deg')
    node.set_servo(s1=0.0)
    time.sleep(1.0)

    node.get_logger().info('Demo done — S1 back to 90 deg')
    node.set_servo(s1=90.0)  # reset

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

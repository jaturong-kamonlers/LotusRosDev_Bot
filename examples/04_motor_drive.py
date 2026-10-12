#!/usr/bin/env python3
"""
04_motor_drive.py — สั่งล้อแบบตั้งเวลา: เดินหน้า → หยุด → หมุนซ้าย → หยุด ผ่าน /cmd_vel
รันที่ : 🟡 Jetson   ก่อนรัน: agent ค้างไว้ · source ~/ros_env.sh
⚠️ ครั้งแรกให้ยกล้อลอย · บนพื้นต้องมีคนเฝ้าและที่ว่าง ≥ 1 m · Ctrl+C = หยุดทันที
ใช้     : python3 04_motor_drive.py               (ค่าเริ่มต้น 0.10 m/s 2 วิ, หมุน 0.6 rad/s 2 วิ)
          python3 04_motor_drive.py 0.15 3        (ความเร็ว m/s, เวลา วินาที)
ข้อความ: geometry_msgs/msg/Twist  linear.x = เดินหน้า (m/s, ลบ = ถอย) · angular.z = หมุน (rad/s, บวก = ซ้าย)
หลักสำคัญ: เฟิร์มแวร์หยุดล้อเองถ้าไม่มีคำสั่งใหม่เกิน 200 ms → ต้องส่งต่อเนื่อง (ที่นี่ 20 ครั้ง/วินาที)
"""
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MAX_LIN, MAX_ANG = 0.25, 1.2                                  # เพดานความปลอดภัย


class Drive(Node):
    def __init__(self):
        super().__init__('example_motor_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x = max(-MAX_LIN, min(MAX_LIN, float(vx)))
        t.angular.z = max(-MAX_ANG, min(MAX_ANG, float(wz)))
        self.pub.publish(t)

    def run_for(self, vx, wz, seconds, hz=20):
        end = time.time() + seconds
        while time.time() < end:
            self.send(vx, wz)
            time.sleep(1.0 / hz)

    def stop(self):
        for _ in range(10):                                   # ส่ง 0 ซ้ำให้แน่ใจ
            self.send(0.0, 0.0)
            time.sleep(0.05)


def main():
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    rclpy.init()
    node = Drive()
    time.sleep(1.0)
    if node.count_publishers('/cmd_vel') > 1:
        print('❌ มี node อื่นส่ง /cmd_vel อยู่ (Nav2/teleop/ภารกิจ) — ปิดก่อน'); node.destroy_node(); rclpy.shutdown(); return
    if input('⚠️ หุ่นจะเคลื่อนที่ — มีคนเฝ้าและพื้นที่ว่างแล้ว? พิมพ์ y แล้ว Enter: ').strip().lower() != 'y':
        node.destroy_node(); rclpy.shutdown(); return
    try:
        node.get_logger().info(f'เดินหน้า {speed} m/s {secs} วิ'); node.run_for(speed, 0.0, secs)
        node.stop(); time.sleep(0.5)
        node.get_logger().info(f'หมุนซ้าย 0.6 rad/s {secs} วิ');  node.run_for(0.0, 0.6, secs)
    except KeyboardInterrupt:
        node.get_logger().warn('หยุด (Ctrl+C)')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

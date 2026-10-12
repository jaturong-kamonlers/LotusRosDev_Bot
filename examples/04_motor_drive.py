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
from geometry_msgs.msg import Twist                   # ข้อความความเร็ว: linear (เส้นตรง) + angular (หมุน)

MAX_LIN, MAX_ANG = 0.25, 1.2                          # เพดานความปลอดภัย: เร็วสุด 0.25 m/s · หมุนสุด 1.2 rad/s


class Drive(Node):
    """node ที่ส่งความเร็วไปให้ล้อ"""

    def __init__(self):
        super().__init__('example_motor_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ

    def send(self, vx=0.0, wz=0.0):
        """ส่งคำสั่งความเร็ว 1 ครั้ง: vx = เดินหน้า (m/s) · wz = หมุน (rad/s)
        จำกัดค่าไม่ให้เกินเพดานก่อนส่งเสมอ"""
        t = Twist()                                           # สร้างข้อความเปล่า (ทุกค่าเป็น 0)
        t.linear.x = max(-MAX_LIN, min(MAX_LIN, float(vx)))   # ใส่ความเร็วเดินหน้า (ตัดไม่ให้เกินเพดาน)
        t.angular.z = max(-MAX_ANG, min(MAX_ANG, float(wz)))  # ใส่ความเร็วหมุน
        self.pub.publish(t)

    def run_for(self, vx, wz, seconds, hz=20):   # hz=20: ส่งซ้ำ 20 ครั้ง/วินาที (ทุก 0.05 วิ) — เฟิร์มแวร์หยุดล้อถ้าเงียบเกิน 200 ms
        """ส่งความเร็วเดิมซ้ำ ๆ hz ครั้ง/วินาที นาน seconds วินาที
        (ต้องส่งซ้ำ เพราะเฟิร์มแวร์หยุดล้อเมื่อเงียบเกิน 200 ms)"""
        end = time.time() + seconds                           # เวลาที่ต้องหยุด
        while time.time() < end:
            self.send(vx, wz)
            time.sleep(1.0 / hz)                              # 20 Hz = ทุก 0.05 วินาที

    def stop(self):
        """ส่งความเร็ว 0 ซ้ำ 10 ครั้ง (0.5 วินาที) ให้แน่ใจว่าล้อหยุด"""
        for _ in range(10):   # 10 ครั้ง × 0.05 วิ = 0.5 วินาที
            self.send(0.0, 0.0)
            time.sleep(0.05)   # 0.05 วิ ระหว่างคำสั่ง (= 20 ครั้ง/วินาที)


def main():
    """ตรวจความปลอดภัย → ถามยืนยัน → เดินหน้า → หยุด → หมุน → หยุด"""
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10   # ความเร็วเดินหน้าเริ่มต้น 0.10 m/s (ช้า · ปลอดภัย) ถ้าไม่ใส่ค่าต่อท้าย
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0   # เวลาแต่ละช่วงเริ่มต้น 2.0 วินาที ถ้าไม่ใส่ค่าต่อท้าย
    rclpy.init()
    node = Drive()
    time.sleep(1.0)                                           # รอ discovery (และให้นับ publisher อื่นได้ถูก)
    # ถ้ามีโปรแกรมอื่นส่ง /cmd_vel อยู่ด้วย คำสั่งจะตีกัน → ไม่ทำต่อ
    if node.count_publishers('/cmd_vel') > 1:
        print('❌ มี node อื่นส่ง /cmd_vel อยู่ (Nav2/teleop/ภารกิจ) — ปิดก่อน'); node.destroy_node(); rclpy.shutdown(); return
    # ให้คนยืนยันว่าพร้อมก่อนขยับ
    if input('⚠️ หุ่นจะเคลื่อนที่ — มีคนเฝ้าและพื้นที่ว่างแล้ว? พิมพ์ y แล้ว Enter: ').strip().lower() != 'y':
        node.destroy_node(); rclpy.shutdown(); return
    try:
        node.get_logger().info(f'เดินหน้า {speed} m/s {secs} วิ'); node.run_for(speed, 0.0, secs)
        node.stop(); time.sleep(0.5)                          # หยุดพักครึ่งวินาทีก่อนหมุน
        node.get_logger().info(f'หมุนซ้าย 0.6 rad/s {secs} วิ');  node.run_for(0.0, 0.6, secs)   # vx 0 m/s · wz 0.6 rad/s (≈34°/วินาที) = หมุนซ้ายอยู่กับที่
    except KeyboardInterrupt:
        node.get_logger().warn('หยุด (Ctrl+C)')
    finally:
        node.stop()                                           # ไม่ว่าจบแบบไหน ต้องสั่งหยุดล้อเสมอ
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
01_led_blink.py — กะพริบ LED บนบอร์ด (GPIO23) ผ่าน topic /led/state
รันที่ : 🟡 Jetson   ก่อนรัน: เปิด agent ค้างไว้ (bash ~/agent.sh) แล้ว source ~/ros_env.sh
ใช้     : python3 01_led_blink.py [จำนวนครั้ง] [วินาทีต่อจังหวะ]      เช่น  python3 01_led_blink.py 5 0.5
ข้อความ: std_msgs/msg/Bool   true = ติด · false = ดับ
หมายเหตุ: สั่ง /led/state ครั้งแรกแล้ว เฟิร์มแวร์จะเลิกใช้ LED เป็นไฟสถานะ จนกว่าจะกด RST
         ถ้า log ขึ้นแต่ไฟไม่กะพริบ = agent ยังไม่เปิด (ros2 node list ต้องเห็น /linorobot_base_node)
"""
import sys, time                                      # sys = อ่านค่าที่พิมพ์ต่อท้ายคำสั่ง · time = หน่วงเวลา
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool                         # LED ใช้ข้อความชนิด Bool (จริง/เท็จ)


class LedBlink(Node):
    """node ที่มีหน้าที่เดียว: ส่งคำสั่งติด/ดับ LED"""

    def __init__(self):
        super().__init__('example_led_blink')                 # ชื่อ node (ห้ามซ้ำกับตัวอื่น)
        self.pub = self.create_publisher(Bool, '/led/state', 10)   # ช่องส่งไปที่ LED ของบอร์ด

    def set_led(self, on: bool):
        """สั่ง LED: on=True ติด · on=False ดับ แล้วพิมพ์บอกใน terminal"""
        self.pub.publish(Bool(data=on))                       # สร้างข้อความแล้วส่ง (บอร์ดรับแล้วสั่งขา GPIO23)
        self.get_logger().info('LED ติด' if on else 'LED ดับ')


def main():
    """อ่านจำนวนครั้ง/จังหวะจากคำสั่ง แล้วสลับติด-ดับตามนั้น"""
    times = int(sys.argv[1]) if len(sys.argv) > 1 else 5      # sys.argv[1] = ค่าแรกหลังชื่อไฟล์ · ไม่ใส่ใช้ 5 ครั้ง
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5 # sys.argv[2] = ค่าที่สอง · ไม่ใส่ใช้ 0.5 วินาที
    rclpy.init()
    node = LedBlink()
    time.sleep(1.0)                                           # รอ discovery ให้ agent เห็น publisher ก่อน (ไม่งั้นคำสั่งแรกหาย)
    try:
        for _ in range(times):                                # ทำซ้ำตามจำนวนครั้ง
            node.set_led(True);  time.sleep(period)           # ติด แล้วรอ
            node.set_led(False); time.sleep(period)           # ดับ แล้วรอ
    except KeyboardInterrupt:                                 # กด Ctrl+C ระหว่างกะพริบ
        pass
    finally:
        node.set_led(False)                                   # จบแล้วดับเสมอ (บรรทัด "LED ดับ" สุดท้ายที่ซ้ำมาจากตรงนี้)
        time.sleep(0.2)                                       # ให้เวลาข้อความสุดท้ายส่งออกไปก่อนปิด
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
01_led_blink.py — กะพริบ LED บนบอร์ด (GPIO23) ผ่าน topic /led/state
รันที่ : 🟡 Jetson   ก่อนรัน: เปิด agent ค้างไว้ (bash ~/agent.sh) แล้ว source ~/ros_env.sh
ใช้     : python3 01_led_blink.py [จำนวนครั้ง] [วินาทีต่อจังหวะ]      เช่น  python3 01_led_blink.py 5 0.5
ข้อความ: std_msgs/msg/Bool   true = ติด · false = ดับ
หมายเหตุ: สั่ง /led/state ครั้งแรกแล้ว เฟิร์มแวร์จะเลิกใช้ LED เป็นไฟสถานะ จนกว่าจะกด RST
"""
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LedBlink(Node):
    def __init__(self):
        super().__init__('example_led_blink')                 # ชื่อ node (ห้ามซ้ำกับตัวอื่น)
        self.pub = self.create_publisher(Bool, '/led/state', 10)

    def set_led(self, on: bool):
        self.pub.publish(Bool(data=on))
        self.get_logger().info('LED ติด' if on else 'LED ดับ')


def main():
    times = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    rclpy.init()
    node = LedBlink()
    time.sleep(1.0)                                           # รอ discovery ให้ agent เห็น publisher ก่อน
    try:
        for _ in range(times):
            node.set_led(True);  time.sleep(period)
            node.set_led(False); time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_led(False)                                   # จบแล้วดับเสมอ
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

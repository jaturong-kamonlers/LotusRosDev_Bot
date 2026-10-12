#!/usr/bin/env python3
"""
07_ir_threshold.py — ตั้ง threshold ของ IR แล้วดูว่าซ้าย/ขวา "เจอวัตถุ" หรือไม่ (เทียบแรงดันกับ threshold)
รันที่ : 🟡 Jetson   ก่อนรัน: agent ค้างไว้ · source ~/ros_env.sh
ใช้     : python3 07_ir_threshold.py 2.0        (threshold เป็นโวลต์ · ไวขึ้น = ค่าน้อยลง)
ข้อความ: /ir/config  std_msgs/Float32  (ค่าหายเมื่อ ESP32 รีเซ็ต — ต้องส่งใหม่ทุกครั้ง)
          /ir/raw     std_msgs/Float32MultiArray [Lv, Rv, Lraw, Rraw]
หมายเหตุ: เซนเซอร์ IR ส่วนใหญ่แรงดันสูง = วัตถุใกล้ (ตรวจของจริงด้วยการเอามือบัง)
"""
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray   # Float32 = ตัวเลขตัวเดียว (ใช้ส่ง threshold)


class IrWatch(Node):
    """ส่งค่า threshold ให้บอร์ด และตัดสินจากแรงดัน IR ว่าเจอวัตถุหรือไม่"""

    def __init__(self, thr):
        super().__init__('example_ir_threshold')
        self.thr = thr                                        # threshold ที่ผู้ใช้กำหนด (โวลต์)
        self.cfg = self.create_publisher(Float32, '/ir/config', 10)                 # ช่องส่ง: ตั้งค่า threshold บนบอร์ด
        self.create_subscription(Float32MultiArray, '/ir/raw', self.on_ir, 10)     # ช่องรับ: ค่า IR ดิบ
        self.create_timer(2.0, self.send_cfg)                 # ส่งค่าตั้งซ้ำทุก 2 วิ เผื่อ ESP32 เพิ่งรีเซ็ต (ค่าเก่าหาย)

    def send_cfg(self):
        """ส่ง threshold ไปที่บอร์ด (บอร์ดใช้แสดงเส้น THR บนจอ OLED)"""
        self.cfg.publish(Float32(data=float(self.thr)))

    def on_ir(self, m):
        """ถูกเรียกทุกครั้งที่ได้ค่า IR: m.data[0] = แรงดันซ้าย · m.data[1] = แรงดันขวา
        ถ้าแรงดันสูงกว่า threshold ถือว่า "เจอ" วัตถุด้านนั้น"""
        lv, rv = m.data[0], m.data[1]
        l = 'เจอ' if lv > self.thr else '—'
        r = 'เจอ' if rv > self.thr else '—'
        self.get_logger().info(f'ซ้าย {lv:.2f}V [{l}]   ขวา {rv:.2f}V [{r}]   (threshold {self.thr:.2f}V)')


def main():
    thr = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0    # ค่าที่พิมพ์ต่อท้าย · ไม่ใส่ใช้ 2.0 V
    rclpy.init()
    node = IrWatch(thr)
    time.sleep(1.0); node.send_cfg()                          # รอ discovery แล้วส่งค่าตั้งครั้งแรกทันที
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

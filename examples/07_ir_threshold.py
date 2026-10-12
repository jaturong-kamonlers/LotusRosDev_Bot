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
from std_msgs.msg import Float32, Float32MultiArray


class IrWatch(Node):
    def __init__(self, thr):
        super().__init__('example_ir_threshold')
        self.thr = thr
        self.cfg = self.create_publisher(Float32, '/ir/config', 10)
        self.create_subscription(Float32MultiArray, '/ir/raw', self.on_ir, 10)
        self.create_timer(2.0, self.send_cfg)                 # ส่งซ้ำทุก 2 วิ เผื่อ ESP32 เพิ่งรีเซ็ต

    def send_cfg(self):
        self.cfg.publish(Float32(data=float(self.thr)))

    def on_ir(self, m):
        lv, rv = m.data[0], m.data[1]
        l = 'เจอ' if lv > self.thr else '—'
        r = 'เจอ' if rv > self.thr else '—'
        self.get_logger().info(f'ซ้าย {lv:.2f}V [{l}]   ขวา {rv:.2f}V [{r}]   (threshold {self.thr:.2f}V)')


def main():
    thr = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    rclpy.init()
    node = IrWatch(thr)
    time.sleep(1.0); node.send_cfg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

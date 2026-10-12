#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════════
# s07_ir_threshold.py — ตั้ง threshold IR แล้วบอกว่าซ้าย/ขวา "เจอวัตถุ" ไหม (แบบเรียงลำดับ)
# ใช้   : python3 s07_ir_threshold.py 2.0     (โวลต์ · ค่าน้อย = ไวขึ้น)
# topic : /ir/config (Float32 → บอร์ด · หายเมื่อ ESP32 รีเซ็ต) · /ir/raw [Lv, Rv, Lraw, Rraw]
# เทียบแบบ class : ../07_ir_threshold.py
# ═══════════════════════════════════════════════════════════════════
import sys, time
import rclpy
from std_msgs.msg import Float32, Float32MultiArray

thr = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0

rclpy.init()
node = rclpy.create_node('example_ir_threshold')
config = node.create_publisher(Float32, '/ir/config', 10)

ir = None                                        # ค่า IR ล่าสุด
def on_ir(msg):
    global ir
    ir = list(msg.data)
node.create_subscription(Float32MultiArray, '/ir/raw', on_ir, 10)

time.sleep(1.0)
next_config = 0.0                                # เวลาที่ต้องส่งค่าตั้งครั้งถัดไป (0 = ส่งทันที)
try:
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() >= next_config:           # ส่งค่าตั้งซ้ำทุก 2 วิ (เผื่อบอร์ดเพิ่งรีเซ็ต)
            config.publish(Float32(data=thr))
            next_config = time.time() + 2.0
        if ir is not None:                       # มีค่าใหม่ → ตัดสินแล้วพิมพ์
            left = 'เจอ' if ir[0] > thr else '—'
            right = 'เจอ' if ir[1] > thr else '—'
            print(f'ซ้าย {ir[0]:.2f}V [{left}]   ขวา {ir[1]:.2f}V [{right}]   (threshold {thr:.2f}V)')
            ir = None                            # ล้างค่า รอค่าใหม่รอบหน้า (ไม่พิมพ์ซ้ำ)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()

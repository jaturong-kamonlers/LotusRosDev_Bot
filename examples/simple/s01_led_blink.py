#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════════
# s01_led_blink.py — กะพริบ LED บนบอร์ด (แบบเรียงลำดับ)
# ใช้   : python3 s01_led_blink.py [จำนวนครั้ง] [วินาที]     เช่น  python3 s01_led_blink.py 5 0.5
# topic : /led/state  (std_msgs/Bool · True = ติด, False = ดับ)
# เทียบแบบ class : ../01_led_blink.py
# ═══════════════════════════════════════════════════════════════════
import sys, time
import rclpy
from std_msgs.msg import Bool

# อ่านค่าที่พิมพ์ต่อท้ายคำสั่ง (ไม่ใส่ใช้ค่าตั้งต้น)
times = int(sys.argv[1]) if len(sys.argv) > 1 else 5
period = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

rclpy.init()
node = rclpy.create_node('example_led_blink')
led = node.create_publisher(Bool, '/led/state', 10)
time.sleep(1.0)                                  # รอ discovery (ไม่งั้นคำสั่งแรกหาย)

try:
    for i in range(times):
        led.publish(Bool(data=True))             # ติด
        print('LED ติด')
        time.sleep(period)
        led.publish(Bool(data=False))            # ดับ
        print('LED ดับ')
        time.sleep(period)
except KeyboardInterrupt:
    pass

led.publish(Bool(data=False))                    # จบแล้วดับเสมอ
time.sleep(0.2)                                  # ให้ข้อความสุดท้ายส่งออกไปก่อน
node.destroy_node()
rclpy.shutdown()

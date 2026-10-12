#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════════
# s02_buzzer_melody.py — เล่นสเกลโด–โด หรือเสียงเดียว (แบบเรียงลำดับ)
# ใช้   : python3 s02_buzzer_melody.py            (สเกล)
#         python3 s02_buzzer_melody.py 1000 300   (ความถี่ Hz, ความยาว ms)
# topic : /buzzer/play  (Float32MultiArray = [ความถี่ Hz, ความยาว ms])
# เทียบแบบ class : ../02_buzzer_melody.py
# ═══════════════════════════════════════════════════════════════════
import sys, time
import rclpy
from std_msgs.msg import Float32MultiArray

notes = [('C4', 261.6), ('D4', 293.7), ('E4', 329.6), ('F4', 349.2),   # (ชื่อโน้ต, ความถี่) เรียงตามที่จะเล่น
         ('G4', 392.0), ('A4', 440.0), ('B4', 493.9), ('C5', 523.3)]

rclpy.init()
node = rclpy.create_node('example_buzzer')
buzzer = node.create_publisher(Float32MultiArray, '/buzzer/play', 10)
time.sleep(1.0)

try:
    if len(sys.argv) == 3:                                   # ใส่ 2 ค่า → เสียงเดียว
        freq = float(sys.argv[1]); ms = float(sys.argv[2])
        buzzer.publish(Float32MultiArray(data=[freq, ms]))   # ค่าต้องเป็น float
        time.sleep(ms / 1000 + 0.1)                          # รอเสียงจบ
    else:                                                    # ไม่ใส่ → เล่นทีละโน้ต
        for name, freq in notes:
            print(name, int(freq), 'Hz')
            buzzer.publish(Float32MultiArray(data=[freq, 250.0]))   # โน้ตละ 250 ms
            time.sleep(0.30)                                 # รอเสียงจบ + เว้นช่วง
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()

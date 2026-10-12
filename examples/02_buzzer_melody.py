#!/usr/bin/env python3
"""
02_buzzer_melody.py — เล่นเพลงสั้น ๆ ด้วย buzzer บนบอร์ด (GPIO18) ผ่าน /buzzer/play
รันที่ : 🟡 Jetson   ก่อนรัน: agent ค้างไว้ · source ~/ros_env.sh
ใช้     : python3 02_buzzer_melody.py            (เล่นสเกล โด–โด)
          python3 02_buzzer_melody.py 1000 300   (เสียงเดียว: ความถี่ Hz, ความยาว ms)
ข้อความ: std_msgs/msg/Float32MultiArray  data = [ความถี่ Hz, ความยาว ms]
          เฟิร์มแวร์จำกัด 20–20000 Hz และ 1–10000 ms · ส่งเสียงใหม่ทับเสียงเดิมทันที
"""
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray            # buzzer รับรายการตัวเลข 2 ค่า [ความถี่, ความยาว]

# ตารางโน้ต: ชื่อโน้ต → ความถี่ (Hz) · C4 = โดกลาง
NOTES = {'C4': 261.6, 'D4': 293.7, 'E4': 329.6, 'F4': 349.2, 'G4': 392.0, 'A4': 440.0, 'B4': 493.9, 'C5': 523.3}
SCALE = ['C4', 'D4', 'E4', 'F4', 'G4', 'A4', 'B4', 'C5']    # โด เร มี ฟา ซอล ลา ที โด (ลำดับที่จะเล่น)


class Buzzer(Node):
    """node ที่ส่งคำสั่งเล่นเสียงไปที่ buzzer ของบอร์ด"""

    def __init__(self):
        super().__init__('example_buzzer')
        self.pub = self.create_publisher(Float32MultiArray, '/buzzer/play', 10)   # 10 = ขนาดคิว (QoS depth): เก็บข้อความรอได้ 10 ข้อความ

    def beep(self, freq_hz: float, dur_ms: float):
        """เล่นเสียง 1 ครั้ง: freq_hz = ความถี่ (สูง = แหลม) · dur_ms = ความยาวเป็นมิลลิวินาที
        ฟังก์ชันนี้ส่งคำสั่งแล้วกลับทันที (ไม่รอเสียงจบ) — ผู้เรียกต้อง sleep เอง"""
        self.pub.publish(Float32MultiArray(data=[float(freq_hz), float(dur_ms)]))   # ต้องเป็น float ทั้งคู่ ไม่งั้น error


def main():
    """ถ้าใส่ตัวเลข 2 ค่า = เล่นเสียงเดียว · ไม่ใส่ = เล่นสเกลโด–โด"""
    rclpy.init()
    node = Buzzer()
    time.sleep(1.0)                                           # รอ discovery
    try:
        if len(sys.argv) == 3:                                # มีค่าพิมพ์ต่อท้าย 2 ค่า → เสียงเดียว
            node.beep(float(sys.argv[1]), float(sys.argv[2]))
            time.sleep(float(sys.argv[2]) / 1000 + 0.1)       # รอให้เสียงเล่นจบก่อนปิดโปรแกรม (ms → วินาที)
        else:                                                 # ไม่ใส่ค่า → ไล่เล่นทีละโน้ตใน SCALE
            for n in SCALE:
                node.get_logger().info(f'{n} {NOTES[n]:.0f} Hz')
                node.beep(NOTES[n], 250)                      # โน้ตละ 250 ms
                time.sleep(0.30)                              # รอเสียงจบ (0.25 วิ) + เว้นช่วงนิดหนึ่ง
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

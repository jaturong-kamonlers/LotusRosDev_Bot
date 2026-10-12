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
from std_msgs.msg import Float32MultiArray

NOTES = {'C4': 261.6, 'D4': 293.7, 'E4': 329.6, 'F4': 349.2, 'G4': 392.0, 'A4': 440.0, 'B4': 493.9, 'C5': 523.3}
SCALE = ['C4', 'D4', 'E4', 'F4', 'G4', 'A4', 'B4', 'C5']    # โด เร มี ฟา ซอล ลา ที โด


class Buzzer(Node):
    def __init__(self):
        super().__init__('example_buzzer')
        self.pub = self.create_publisher(Float32MultiArray, '/buzzer/play', 10)

    def beep(self, freq_hz: float, dur_ms: float):
        self.pub.publish(Float32MultiArray(data=[float(freq_hz), float(dur_ms)]))   # ต้องเป็น float ทั้งคู่


def main():
    rclpy.init()
    node = Buzzer()
    time.sleep(1.0)
    try:
        if len(sys.argv) == 3:
            node.beep(float(sys.argv[1]), float(sys.argv[2]))
            time.sleep(float(sys.argv[2]) / 1000 + 0.1)
        else:
            for n in SCALE:
                node.get_logger().info(f'{n} {NOTES[n]:.0f} Hz')
                node.beep(NOTES[n], 250)
                time.sleep(0.30)                              # รอเสียงจบ + เว้นช่วงนิดหนึ่ง
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

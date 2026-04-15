#!/usr/bin/env python3
"""
lotus_buzzer_node.py — เล่นเสียง โด เร มี
Topic: /buzzer/play (std_msgs/Float32MultiArray) [freq_hz, duration_ms]

โน้ต:
  โด = C4 = 261.63 Hz
  เร = D4 = 293.66 Hz
  มี = E4 = 329.63 Hz
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

# ── โน้ต ─────────────────────────────────────────────────────
NOTE = {
    'C4': 261.63,   # โด
    'D4': 293.66,   # เร
    'E4': 329.63,   # มี
    'F4': 349.23,   # ฟา
    'G4': 392.00,   # ซอล
    'A4': 440.00,   # ลา
    'B4': 493.88,   # ที
    'C5': 523.25,   # โด (สูง)
    'REST': 0.0,    # หยุด
}

class LotusBuzzerNode(Node):
    def __init__(self):
        super().__init__('lotus_buzzer_node')
        self.pub = self.create_publisher(
            Float32MultiArray, '/buzzer/play', 10)
        self.get_logger().info('LotusBuzzerNode started')

    def play_note(self, note_name: str, duration_ms: float, gap_ms: float = 50):
        """เล่นโน้ตตัวเดียว แล้วหยุด gap_ms ก่อนโน้ตถัดไป"""
        freq = NOTE.get(note_name, 0.0)
        if freq > 0:
            msg = Float32MultiArray()
            msg.data = [freq, duration_ms]
            self.pub.publish(msg)
            self.get_logger().info(f'Note: {note_name} ({freq:.0f}Hz) {duration_ms:.0f}ms')
        time.sleep((duration_ms + gap_ms) / 1000.0)

    def play_melody(self, melody: list):
        """
        melody = [(note_name, duration_ms), ...]
        ตัวอย่าง: [('C4', 400), ('D4', 400), ('E4', 600)]
        """
        for note, dur in melody:
            self.play_note(note, dur)

def main(args=None):
    rclpy.init(args=args)
    node = LotusBuzzerNode()

    # รอให้ node เชื่อมต่อก่อน
    time.sleep(1.0)

    # ── ตัวอย่าง: โด เร มี ────────────────────────────────
    node.get_logger().info('Playing: โด เร มี')
    melody_do_re_mi = [
        ('C4', 400),    # โด
        ('D4', 400),    # เร
        ('E4', 600),    # มี
        ('REST', 200),  # หยุดสั้น
        ('E4', 300),    # มี
        ('E4', 300),    # มี
        ('E4', 600),    # มี (ยาว)
    ]
    node.play_melody(melody_do_re_mi)

    node.get_logger().info('Melody done')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

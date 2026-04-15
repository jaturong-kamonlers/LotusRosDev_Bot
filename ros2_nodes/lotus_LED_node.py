#!/usr/bin/env python3
"""
lotus_LED_node.py — สั่ง LED GPIO23 บน ESP32 ผ่าน ROS2
=======================================================
Topic: /led/state (std_msgs/Bool)
  True  = LED ติด
  False = LED ดับ

รันด้วย:
  python3 lotus_LED_node.py

คำสั่งจาก terminal โดยไม่ต้องรัน node นี้:
  ros2 topic pub /led/state std_msgs/msg/Bool "data: true"  --once
  ros2 topic pub /led/state std_msgs/msg/Bool "data: false" --once

หมายเหตุ:
  - firmware อาจ override LED ด้วย ROS2 status (twistCallback)
  - topic นี้ override ได้เสมอ แต่ถ้ารัน teleop พร้อมกัน LED จะกระพริบสลับ
  - ถ้าต้องการ LED เฉพาะทาง ควร disable LED_PIN ใน twistCallback ก่อน
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
import time


class LotusLEDNode(Node):

    def __init__(self):
        super().__init__('lotus_led_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.pub = self.create_publisher(Bool, '/led/state', qos)
        self.led_state = False

        self.get_logger().info(
            'LotusLEDNode started\n'
            '  topic: /led/state (std_msgs/Bool)\n'
            '  True=ON  False=OFF')

    def set_led(self, state: bool):
        """สั่ง LED ติด (True) หรือดับ (False)"""
        msg = Bool()
        msg.data = state
        self.pub.publish(msg)
        self.led_state = state
        self.get_logger().info(f'LED → {"ON" if state else "OFF"}')

    def blink(self, times: int = 3, on_ms: float = 500, off_ms: float = 500):
        """กระพริบ LED จำนวน times ครั้ง"""
        self.get_logger().info(f'Blink {times}x ({on_ms:.0f}ms on / {off_ms:.0f}ms off)')
        for i in range(times):
            self.set_led(True)
            time.sleep(on_ms / 1000.0)
            self.set_led(False)
            time.sleep(off_ms / 1000.0)


def main(args=None):
    rclpy.init(args=args)
    node = LotusLEDNode()

    # รอ node เชื่อมต่อ
    time.sleep(1.0)

    # ── ตัวอย่าง: กระพริบ 3 ครั้ง แล้วติดค้าง ───────────────
    node.get_logger().info('Demo: blink 3x then ON')
    node.blink(times=3, on_ms=300, off_ms=300)
    time.sleep(0.5)
    node.set_led(True)    # ติดค้าง

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_led(False)   # ดับก่อนออก
        node.get_logger().info('LED OFF — shutdown')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

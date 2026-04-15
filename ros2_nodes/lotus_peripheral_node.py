#!/usr/bin/env python3
"""
lotus_peripheral_node.py — ทดสอบ peripheral แยกส่วน
====================================================
ใช้สำหรับ: ทดสอบ IR/BTN/POT/Servo/Buzzer ก่อน integrate เข้า robot_node
ปลอดภัย: ไม่ publish /cmd_vel → รันคู่กับ robot_node ได้

Subscribe:
  /ir/raw         Float32MultiArray  [lv, rv, lraw, rraw]
  /button/state   Bool
  /pot/raw        Float32MultiArray  [v, raw]

Publish:
  /ir/config      Float32            threshold V
  /servo/cmd      Float32MultiArray  [s1, s2, s3] องศา
  /buzzer/play    Float32MultiArray  [freq_hz, duration_ms]

รันด้วย:
  python3 lotus_peripheral_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray, Bool


class LotusPeripheralNode(Node):

    def __init__(self):
        super().__init__('lotus_peripheral_node')

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ── Subscribers ──────────────────────────────────────
        self.create_subscription(Float32MultiArray, '/ir/raw',
                                 self._cb_ir,      qos_sensor)
        self.create_subscription(Bool,              '/button/state',
                                 self._cb_button,  qos_sensor)
        self.create_subscription(Float32MultiArray, '/pot/raw',
                                 self._cb_pot,     qos_sensor)

        # ── Publishers ───────────────────────────────────────
        self.pub_ir_config = self.create_publisher(Float32,             '/ir/config',   qos_cmd)
        self.pub_servo     = self.create_publisher(Float32MultiArray,   '/servo/cmd',   qos_cmd)
        self.pub_buzzer    = self.create_publisher(Float32MultiArray,   '/buzzer/play', qos_cmd)

        # ── State ─────────────────────────────────────────────
        self.ir_left_v    = 0.0
        self.ir_right_v   = 0.0
        self.ir_left_raw  = 0
        self.ir_right_raw = 0
        self.ir_threshold = 2.0

        self.get_logger().info(
            'LotusPeripheralNode started\n'
            '  Monitoring IR/BTN/POT — no motion commands')

    # ── Callbacks ─────────────────────────────────────────────
    def _cb_ir(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        self.ir_left_v    = msg.data[0]
        self.ir_right_v   = msg.data[1]
        self.ir_left_raw  = int(msg.data[2])
        self.ir_right_raw = int(msg.data[3])
        self.get_logger().info(
            f'IR  L:{self.ir_left_v:.2f}V[{self.ir_left_raw}]'
            f'  R:{self.ir_right_v:.2f}V[{self.ir_right_raw}]',
            throttle_duration_sec=0.5)

    def _cb_button(self, msg: Bool):
        state = 'PRESSED' if msg.data else 'released'
        self.get_logger().info(f'BTN: {state}')

    def _cb_pot(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        self.get_logger().info(
            f'POT: {msg.data[0]:.2f}V [{int(msg.data[1])}]',
            throttle_duration_sec=0.5)

    # ── Helpers ───────────────────────────────────────────────
    def set_ir_threshold(self, threshold_v: float):
        msg = Float32()
        msg.data = float(threshold_v)
        self.pub_ir_config.publish(msg)
        self.ir_threshold = threshold_v
        self.get_logger().info(f'IR threshold → {threshold_v:.2f}V')

    def set_servo(self, s1: float = 90.0, s2: float = 90.0, s3: float = 90.0):
        msg = Float32MultiArray()
        msg.data = [
            float(max(0.0, min(180.0, s1))),
            float(max(0.0, min(180.0, s2))),
            float(max(0.0, min(180.0, s3))),
        ]
        self.pub_servo.publish(msg)
        self.get_logger().info(f'Servo: [{msg.data[0]:.0f}, {msg.data[1]:.0f}, {msg.data[2]:.0f}] deg')

    def play_buzzer(self, freq_hz: float, duration_ms: float):
        msg = Float32MultiArray()
        msg.data = [float(freq_hz), float(duration_ms)]
        self.pub_buzzer.publish(msg)
        self.get_logger().info(f'Buzzer: {freq_hz:.0f}Hz {duration_ms:.0f}ms')


def main(args=None):
    rclpy.init(args=args)
    node = LotusPeripheralNode()

    import time
    time.sleep(1.0)
    node.set_ir_threshold(2.0)
    node.play_buzzer(1500.0, 150.0)
    node.set_servo(90.0, 90.0, 90.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

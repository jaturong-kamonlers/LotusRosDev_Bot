#!/usr/bin/env python3
"""
lotus_robot_node.py — Brain หลักของหุ่นยนต์ LotusRosDev Bot
=============================================================
รันตัวเดียว ครบทุกอย่าง ห้ามรัน node อื่นที่ publish /cmd_vel พร้อมกัน

Subscribe:
  /ir/raw         Float32MultiArray  [ir_left_v, ir_right_v, ir_left_raw, ir_right_raw]
  /button/state   Bool               True=กด
  /pot/raw        Float32MultiArray  [pot_v, pot_raw]
  /odom/unfiltered Odometry

Publish:
  /cmd_vel        Twist              สั่งความเร็ว
  /servo/cmd      Float32MultiArray  [s1, s2, s3] องศา
  /buzzer/play    Float32MultiArray  [freq_hz, duration_ms]
  /ir/config      Float32            threshold V

รันด้วย:
  python3 lotus_robot_node.py
  หรือ ros2 launch launch/robot.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray, Bool
import math


class LotusRobotNode(Node):

    def __init__(self):
        super().__init__('lotus_robot_node')

        # ── QoS ──────────────────────────────────────────────
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ── Publishers ───────────────────────────────────────
        self.pub_cmd    = self.create_publisher(Twist,             '/cmd_vel',     qos_cmd)
        self.pub_servo  = self.create_publisher(Float32MultiArray, '/servo/cmd',   qos_cmd)
        self.pub_buzzer = self.create_publisher(Float32MultiArray, '/buzzer/play', qos_cmd)
        self.pub_irconf = self.create_publisher(Float32,           '/ir/config',   qos_cmd)

        # ── Subscribers ──────────────────────────────────────
        self.create_subscription(Float32MultiArray, '/ir/raw',
                                 self._cb_ir,      qos_sensor)
        self.create_subscription(Bool,              '/button/state',
                                 self._cb_button,  qos_sensor)
        self.create_subscription(Float32MultiArray, '/pot/raw',
                                 self._cb_pot,     qos_sensor)
        self.create_subscription(Odometry,          '/odom/unfiltered',
                                 self._cb_odom,    qos_sensor)

        # ── State ─────────────────────────────────────────────
        self.running      = False
        self.ir_left_v    = 0.0
        self.ir_right_v   = 0.0
        self.ir_threshold = 2.0    # V — ปรับได้ผ่าน POT
        self.x = self.y = self.yaw = 0.0

        # ── Speed params ──────────────────────────────────────
        self.fwd_speed  = 0.15   # m/s
        self.turn_speed = 0.5    # rad/s

        # ── Control loop 10 Hz ────────────────────────────────
        self.create_timer(0.1, self._control_loop)

        # ── ส่ง threshold เริ่มต้น หลัง node พร้อม ──────────
        self.create_timer(2.0, self._init_once)
        self._inited = False

        self.get_logger().info(
            'LotusRobotNode started\n'
            '  Press BTN on robot to START/STOP\n'
            '  Rotate POT to adjust IR threshold\n'
            f'  IR threshold: {self.ir_threshold:.1f}V')

    # ── Init once ─────────────────────────────────────────────
    def _init_once(self):
        if self._inited:
            return
        self._inited = True
        cfg = Float32()
        cfg.data = self.ir_threshold
        self.pub_irconf.publish(cfg)
        self.set_servo(90.0, 90.0, 90.0)
        self.beep(1500.0, 200.0)
        self.get_logger().info('Robot initialized — IR threshold sent, servo centered')

    # ── Callbacks ─────────────────────────────────────────────
    def _cb_ir(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        self.ir_left_v  = msg.data[0]
        self.ir_right_v = msg.data[1]

    def _cb_button(self, msg: Bool):
        if not msg.data:
            return
        self.running = not self.running
        state = 'START' if self.running else 'STOP'
        self.get_logger().info(f'BTN → {state}')
        freq = 1500.0 if self.running else 800.0
        self.beep(freq, 200.0)
        if not self.running:
            self.stop()

    def _cb_pot(self, msg: Float32MultiArray):
        if len(msg.data) < 1:
            return
        # POT 0–3.3V → threshold 0.5–3.0V
        new_thr = round(0.5 + (msg.data[0] / 3.3) * 2.5, 2)
        if abs(new_thr - self.ir_threshold) > 0.05:
            self.ir_threshold = new_thr
            cfg = Float32()
            cfg.data = self.ir_threshold
            self.pub_irconf.publish(cfg)
            self.get_logger().info(f'IR threshold → {self.ir_threshold:.2f}V')

    def _cb_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    # ── Control loop ──────────────────────────────────────────
    def _control_loop(self):
        if not self.running:
            return
        bl = self.ir_left_v  > self.ir_threshold
        br = self.ir_right_v > self.ir_threshold

        if bl and br:
            self.stop()
            self.beep(600.0, 300.0)
            self.get_logger().warn(
                f'BLOCKED L:{self.ir_left_v:.2f}V R:{self.ir_right_v:.2f}V',
                throttle_duration_sec=1.0)
        elif bl:
            self.move(0.05, -self.turn_speed)
            self.get_logger().info('IR left → turn right', throttle_duration_sec=1.0)
        elif br:
            self.move(0.05, self.turn_speed)
            self.get_logger().info('IR right → turn left', throttle_duration_sec=1.0)
        else:
            self.move(self.fwd_speed, 0.0)

    # ── Helpers ───────────────────────────────────────────────
    def move(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd.publish(msg)

    def stop(self):
        self.move(0.0, 0.0)

    def beep(self, freq: float, dur: float):
        msg = Float32MultiArray()
        msg.data = [float(freq), float(dur)]
        self.pub_buzzer.publish(msg)

    def set_servo(self, s1: float, s2: float, s3: float):
        msg = Float32MultiArray()
        msg.data = [
            float(max(0.0, min(180.0, s1))),
            float(max(0.0, min(180.0, s2))),
            float(max(0.0, min(180.0, s3))),
        ]
        self.pub_servo.publish(msg)

    @property
    def pose_str(self):
        return (f'x:{self.x:.3f}m y:{self.y:.3f}m '
                f'yaw:{math.degrees(self.yaw):.1f}deg')


def main(args=None):
    rclpy.init(args=args)
    node = LotusRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
lotus_motion_node.py — Monitor odom/imu และ helper สำหรับ motion
=================================================================
โหมด 1 (default): MONITOR — subscribe odom/imu แสดงค่า log เท่านั้น
                   ปลอดภัยรันคู่กับ lotus_robot_node ได้
โหมด 2: CONTROL  — publish /cmd_vel ด้วย (ต้องรันคนเดียว ห้ามรันกับ robot_node)

Subscribe:
  /odom/unfiltered  Odometry
  /imu/data         Imu
  /imu/mag          MagneticField

Publish (โหมด CONTROL เท่านั้น):
  /cmd_vel  Twist

รันด้วย:
  python3 lotus_motion_node.py             # monitor mode
  python3 lotus_motion_node.py --control   # control mode (รันคนเดียว)
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField


class LotusMotionNode(Node):

    def __init__(self, control_mode: bool = False):
        super().__init__('lotus_motion_node')
        self.control_mode = control_mode

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ── Subscribers (ทุกโหมด) ────────────────────────────
        self.create_subscription(Odometry,      '/odom/unfiltered',
                                 self._cb_odom, qos)
        self.create_subscription(Imu,            '/imu/data',
                                 self._cb_imu,  qos)
        self.create_subscription(MagneticField,  '/imu/mag',
                                 self._cb_mag,  qos)

        # ── Publisher (โหมด CONTROL เท่านั้น) ────────────────
        if self.control_mode:
            qos_cmd = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10)
            self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', qos_cmd)
            self.get_logger().warn(
                'CONTROL MODE — do NOT run with lotus_robot_node!')
        else:
            self.pub_cmd = None
            self.get_logger().info('MONITOR MODE — safe to run alongside robot_node')

        # ── State ─────────────────────────────────────────────
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vz = 0.0
        self.accel_x = self.accel_y = self.accel_z = 0.0
        self.gyro_z  = 0.0
        self.heading = 0.0
        self.odom_ready = False

        # ── Log timer ─────────────────────────────────────────
        self.create_timer(1.0, self._log_status)

    # ── Callbacks ─────────────────────────────────────────────
    def _cb_odom(self, msg: Odometry):
        self.x  = msg.pose.pose.position.x
        self.y  = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.vz = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_ready = True

    def _cb_imu(self, msg: Imu):
        self.accel_x = msg.linear_acceleration.x
        self.accel_y = msg.linear_acceleration.y
        self.accel_z = msg.linear_acceleration.z
        self.gyro_z  = msg.angular_velocity.z

    def _cb_mag(self, msg: MagneticField):
        self.heading = math.degrees(
            math.atan2(msg.magnetic_field.y, msg.magnetic_field.x))

    def _log_status(self):
        if not self.odom_ready:
            self.get_logger().info('Waiting for odom...')
            return
        self.get_logger().info(
            f'Odom: x={self.x:.3f}m y={self.y:.3f}m '
            f'yaw={math.degrees(self.yaw):.1f}deg '
            f'vx={self.vx:.3f}m/s | '
            f'Heading:{self.heading:.1f}deg')

    # ── Motion helpers (ใช้ได้เฉพาะโหมด CONTROL) ─────────────
    def move(self, linear: float, angular: float):
        if not self.control_mode:
            self.get_logger().warn('move() called in MONITOR mode — ignored')
            return
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd.publish(msg)

    def stop(self):
        self.move(0.0, 0.0)

    @property
    def pose_str(self):
        return (f'x:{self.x:.3f}m y:{self.y:.3f}m '
                f'yaw:{math.degrees(self.yaw):.1f}deg')


def main(args=None):
    rclpy.init(args=args)
    control_mode = '--control' in (args or sys.argv)
    node = LotusMotionNode(control_mode=control_mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if control_mode:
            node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

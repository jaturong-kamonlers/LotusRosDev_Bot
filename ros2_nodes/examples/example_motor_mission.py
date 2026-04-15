#!/usr/bin/env python3
"""
lotus_motor_node.py — เดินหน้า 1 เมตร แล้วเลี้ยวขวา 90 องศา
Topics:
  Publish : /cmd_vel  (geometry_msgs/Twist)
  Subscribe: /odom/unfiltered (nav_msgs/Odometry)

หมายเหตุสำคัญ:
  ต้องรัน micro-ROS agent ก่อน:
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
  หุ่นต้องอยู่บนพื้นและล้อสัมผัสพื้น
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class LotusMotorNode(Node):
    def __init__(self):
        super().__init__('lotus_motor_node')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom/unfiltered', self._cb_odom, 10)

        # ── state ──────────────────────────────────────────
        self.x    = 0.0
        self.y    = 0.0
        self.yaw  = 0.0
        self.odom_ready = False

        # ── parameters ─────────────────────────────────────
        self.linear_speed  = 0.15   # m/s
        self.angular_speed = 0.5    # rad/s
        self.target_dist   = 1.0    # เมตร
        self.target_turn   = math.pi / 2  # 90 องศา

        self.get_logger().info('LotusMotorNode started')
        self.get_logger().info(f'Target: forward {self.target_dist}m, then turn right 90 deg')

    def _cb_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # quaternion → yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_ready = True

    def send_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.pub_cmd.publish(msg)

    def stop(self):
        self.send_cmd(0.0, 0.0)
        time.sleep(0.5)

    def drive_forward(self, distance_m: float):
        """เดินหน้าตามระยะทาง (ใช้ odom)"""
        self.get_logger().info(f'Driving forward {distance_m}m...')
        start_x, start_y = self.x, self.y

        rate = self.create_rate(20)  # 20 Hz
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            dist = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            self.get_logger().info(f'  dist: {dist:.3f}/{distance_m:.3f}m', throttle_duration_sec=0.5)
            if dist >= distance_m:
                break
            self.send_cmd(self.linear_speed, 0.0)

        self.stop()
        self.get_logger().info('Forward done')

    def turn_right(self, angle_rad: float):
        """เลี้ยวขวา (ใช้ yaw จาก odom)"""
        self.get_logger().info(f'Turning right {math.degrees(angle_rad):.0f} deg...')
        start_yaw = self.yaw
        turned    = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            delta = self.yaw - start_yaw
            # normalize -pi..pi
            while delta >  math.pi: delta -= 2 * math.pi
            while delta < -math.pi: delta += 2 * math.pi
            turned = abs(delta)
            self.get_logger().info(f'  turned: {math.degrees(turned):.1f}/{math.degrees(angle_rad):.0f} deg',
                                   throttle_duration_sec=0.5)
            if turned >= angle_rad:
                break
            # เลี้ยวขวา = angular.z ลบ
            self.send_cmd(0.0, -self.angular_speed)

        self.stop()
        self.get_logger().info('Turn done')

def main(args=None):
    rclpy.init(args=args)
    node = LotusMotorNode()

    # รอ odom พร้อม
    node.get_logger().info('Waiting for odom...')
    timeout = 10.0
    t0 = time.time()
    while not node.odom_ready and (time.time() - t0) < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not node.odom_ready:
        node.get_logger().error('Odom not available — is micro-ROS agent running?')
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info('Odom ready — starting mission')
    time.sleep(1.0)

    # ── Mission: เดินหน้า 1m → เลี้ยวขวา 90 deg ──────────
    try:
        node.drive_forward(node.target_dist)
        time.sleep(0.5)
        node.turn_right(node.target_turn)
        node.get_logger().info('Mission complete')
    except Exception as e:
        node.get_logger().error(f'Mission failed: {e}')
        node.stop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

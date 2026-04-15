"""
robot.launch.py — รัน LotusRosDev Bot แบบ production
=====================================================
รัน 2 node:
  1. lotus_robot_node  — brain หลัก (obstacle avoidance, BTN toggle)
  2. lotus_motion_node — monitor odom/imu (ไม่ publish /cmd_vel)

รันด้วย:
  ros2 launch launch/robot.launch.py

ก่อนรัน ต้องมี micro-ROS agent รันอยู่:
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
  หรือ
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([

        LogInfo(msg='Starting LotusRosDev Bot...'),
        LogInfo(msg='Make sure micro-ROS agent is running first!'),

        # ── Brain หลัก ────────────────────────────────────────
        Node(
            package='lotus_ros2',
            executable='lotus_robot_node',
            name='lotus_robot_node',
            output='screen',
            parameters=[{
                'ir_threshold': 2.0,
                'fwd_speed':    0.15,
                'turn_speed':   0.5,
            }]
        ),

        # ── Monitor odom/imu ──────────────────────────────────
        Node(
            package='lotus_ros2',
            executable='lotus_motion_node',
            name='lotus_motion_node',
            output='screen',
            arguments=['--ros-args'],
        ),

    ])

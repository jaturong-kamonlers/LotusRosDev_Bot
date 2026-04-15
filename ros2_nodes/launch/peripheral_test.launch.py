"""
peripheral_test.launch.py — ทดสอบ peripheral แยกส่วน
======================================================
รัน lotus_peripheral_node เพื่อทดสอบ IR/BTN/POT/Servo/Buzzer

รันด้วย:
  ros2 launch launch/peripheral_test.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([

        LogInfo(msg='Starting peripheral test node...'),

        Node(
            package='lotus_ros2',
            executable='lotus_peripheral_node',
            name='lotus_peripheral_node',
            output='screen',
        ),

    ])

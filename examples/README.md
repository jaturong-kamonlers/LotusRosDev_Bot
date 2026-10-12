# ตัวอย่าง node สั่งเฟิร์มแวร์ LotusRosDev Shield  (~/lotusbot/scripts/example)

รันบน 🟡 Jetson · ทุกตัวต้องเปิด agent ค้างไว้ก่อน (`bash ~/agent.sh` หรือ `start.sh`) แล้ว `source ~/ros_env.sh`

| ไฟล์ | ทำอะไร | topic | ขยับหุ่น? |
|---|---|---|---|
| 00_template_node.py | แม่แบบเริ่ม node ใหม่ (pub+sub+timer) | /led/state, /button/state | ไม่ |
| 01_led_blink.py | กะพริบ LED | /led/state (Bool) | ไม่ |
| 02_buzzer_melody.py | เล่นสเกลโด–โด / เสียงเดียว | /buzzer/play [Hz, ms] | ไม่ |
| 03_servo_pose.py | สั่งแขนกลด้วยชื่อท่า/มุม | /servo/cmd [s1,s2,s3] | แขนกล |
| 04_motor_drive.py | เดินหน้า-หมุน ตั้งเวลา | /cmd_vel (Twist) | **ล้อ** |
| 05_read_sensors.py | พิมพ์ค่าเซนเซอร์ทุกตัว | button, ir, pot, odom, imu | ไม่ |
| 06_button_beep_led.py | กดปุ่ม → บี๊บ + LED | button → buzzer, led | ไม่ |
| 07_ir_threshold.py | ตั้ง threshold IR + ดูผล | /ir/config, /ir/raw | ไม่ |
| 08_pot_to_servo.py | หมุน POT → servo หนีบหมุนตาม | /pot/raw → /servo/cmd | แขนกล |

⚠️ ตัวที่ขยับหุ่น/แขนกลต้องมีคนเฝ้า · ห้ามรันพร้อมภารกิจ / Tuner / Nav2 · คู่มือบทที่ 14.6 (วิธีใช้ VS Code: 14.4 · ทั้งบท 14 = สั่งงานบอร์ดด้วย ROS 2)

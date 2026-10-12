# ตัวอย่างแบบเรียงลำดับ (simple) — ~/lotusbot/scripts/example/simple

เขียนแบบอ่านจากบนลงล่าง ไม่มี class (คล้าย C / Arduino) · ผลลัพธ์เหมือนไฟล์แบบ class ในโฟลเดอร์ `../` ทุกตัว
โครงทุกไฟล์: เริ่ม ROS 2 → สร้าง node → สร้างช่องส่ง/รับ → loop หลัก (`rclpy.spin_once`) → ปิด

| simple | แบบ class | ทำอะไร |
|---|---|---|
| s00_template.py | 00_template_node.py | แม่แบบ: LED กะพริบ + อ่านปุ่ม |
| s01_led_blink.py | 01_led_blink.py | กะพริบ LED |
| s02_buzzer_melody.py | 02_buzzer_melody.py | เล่นสเกล / เสียงเดียว |
| s03_servo_pose.py | 03_servo_pose.py | แขนกลตามชื่อท่า / มุม |
| s04_motor_drive.py | 04_motor_drive.py | เดินหน้า-หมุน ตั้งเวลา (⚠️ ล้อ) |
| s05_read_sensors.py | 05_read_sensors.py | พิมพ์ค่าเซนเซอร์ |
| s06_button_beep_led.py | 06_button_beep_led.py | กดปุ่ม → บี๊บ + LED |
| s07_ir_threshold.py | 07_ir_threshold.py | IR threshold |
| s08_pot_to_servo.py | 08_pot_to_servo.py | POT → servo หนีบ |

รัน: agent ค้างไว้ (`bash ~/agent.sh`) · `source ~/ros_env.sh` · `python3 s01_led_blink.py 5 0.5` · คู่มือบทที่ 14

# LotusRosBot — เฟิร์มแวร์ ESP32

เฟิร์มแวร์ของบอร์ด ESP32 (LotusRosDev Shield) ที่คุยกับ ROS 2 Galactic บน Jetson Nano ผ่าน micro-ROS (สาย USB)

```
firmware/
├── platformio.ini              env: lotusrosbot
├── include/
│   ├── lotusrosbot_config.h    ค่าตั้งหุ่นทั้งหมด (ขา, PID, ล้อ, CPR, อุปกรณ์เสริม)
│   └── config.h                ชี้ไปที่ไฟล์บน (ไลบรารีเดิม include ชื่อนี้)
├── src/
│   ├── main.cpp                setup() + loop() · เลือกโหมดตอนบูต
│   ├── version.h               ชื่อ + เวอร์ชัน (ขึ้นบน OLED)
│   ├── ros_link.cpp/.h         micro-ROS: ต่อ agent, topic, callback, ส่งข้อมูล
│   ├── base.cpp/.h             มอเตอร์ + encoder + PID + odometry ล้อ
│   ├── heading.cpp/.h          IMU MPU6050 + มุมหมุนจาก gyro ที่รวมบนบอร์ด 50 Hz
│   ├── sensors.cpp/.h          IR, potentiometer, ปุ่ม
│   ├── actuators.cpp/.h        LED, buzzer, servo
│   ├── display.cpp/.h          จอ OLED
│   ├── netinfo.cpp/.h          รับ SSID/IP/SSH ของ Jetson ทาง USB
│   └── calib/                  โหมดคาลิเบรต 9 เมนู
│       ├── calib_menu.cpp      หน้าเลือกเมนู
│       ├── calib_common.*      เสียง ปุ่ม POT จอ
│       ├── calib_motor.cpp     1 Spin · 2 CPR · 3 CPR x5 · 7 Count ticks
│       ├── calib_io.cpp        4 Servo · 5 Sensor · 6 LED
│       └── calib_network.cpp   8 Network · 9 SSH
└── lib/                        ไลบรารี (เหลือเฉพาะที่หุ่นใช้)
```

## ROS 2 interface
| ทิศ | topic | ชนิด | หมายเหตุ |
|---|---|---|---|
| ส่ง | `/odom/unfiltered` | nav_msgs/Odometry | ~3–5 Hz |
| ส่ง | `/imu/data` | sensor_msgs/Imu | yaw = gyro รวมบนบอร์ด (ป้าย orientation_covariance[8] = 0.0004) |
| ส่ง | `/ir/raw` `/pot/raw` | std_msgs/Float32MultiArray | ทุก 200 ms |
| ส่ง | `/button/state` | std_msgs/Bool | เมื่อเปลี่ยน |
| (สร้างไว้) | `/imu/mag` | sensor_msgs/MagneticField | ไม่ส่ง — MPU6050 ไม่มีเข็มทิศ |
| รับ | `/cmd_vel` | geometry_msgs/Twist | เงียบเกิน 200 ms = หยุดล้อ |
| รับ | `/led/state` `/ir/config` `/servo/cmd` `/buzzer/play` | Bool / Float32 / Float32MultiArray | |

node: `lotusrosbot_base`

## ที่มาและลิขสิทธิ์
พัฒนาต่อจาก [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware)
(Apache License 2.0, © 2021 Juan Miguel Jimeno) — ไฟล์ใน `lib/` คงบรรทัดลิขสิทธิ์เดิมไว้
`lib/encoder` มาจาก PJRC Encoder library (MIT) · `lib/wifi` © 2023 Thomas Chou (Apache-2.0)
ส่วนที่ทีม LotusRosBot เขียนเพิ่ม: จอ OLED, โหมดคาลิเบรต, netinfo, การรวม gyro บนบอร์ด, IR/POT/ปุ่ม/servo/buzzer

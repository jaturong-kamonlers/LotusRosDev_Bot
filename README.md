# LotusRosDev Bot

**EN:** Unified ROS2 + Calibration firmware for the LotusRosDev Shield — a custom shield developed on ESP32 DevKit V1 30-pin, running micro-ROS with ROS2 Humble on Linorobot2 platform.

**TH:** เฟิร์มแวร์รวม ROS2 และ Calibration สำหรับบอร์ด LotusRosDev Shield — Shield ที่พัฒนาขึ้นเองจาก ESP32 DevKit V1 30-pin ทำงานร่วมกับ micro-ROS บน ROS2 Humble และแพลตฟอร์ม Linorobot2

---

## Hardware / ฮาร์ดแวร์

| ส่วนประกอบ | รายละเอียด |
|-----------|-----------|
| **บอร์ด** | **LotusRosDev Shield** (ESP32 DevKit V1 30-pin) |
| IMU | MPU6050 — I2C SDA=21, SCL=22 |
| Motor driver | Generic 2-IN (L298N / TB6612FNG) |
| จอแสดงผล | OLED SSD1306 128×64 — I2C |
| Encoder | Quadrature encoder ×2 |
| Servo | ×3 — GPIO 32, 33, 5 |
| Buzzer | Passive PWM — GPIO 18 (LEDC ch7) |
| IR Sensor | Analog ×2 — GPIO 36, 39 (ADC1) |
| Button | GPIO 27 — active LOW |
| Potentiometer | GPIO 35 — ADC1 |
| LED | GPIO 23 — status + `/led/state` topic |
| Host | Jetson Nano / PC — ROS2 Humble |

---

## Repository Structure / โครงสร้างโปรเจกต์

```
LotusRosDev_Bot/
│
├── firmware/                         ← *** upload ตัวนี้ตัวเดียว ***
│   ├── src/
│   │   ├── firmware.ino              ← unified firmware (ROS2 + boot mode select)
│   │   └── calib_menu.h              ← calibration menu ทั้งหมด (include โดย firmware.ino)
│   └── platformio.ini
│
├── calibration/                      ← standalone backup (upload แยกได้เสมอ)
│   ├── src/firmware.ino
│   └── platformio.ini
│
├── config/
│   ├── config.h
│   ├── lino_base_config.h
│   └── custom/
│       └── myrobot_config.h          ← *** แก้ค่าที่นี่ที่เดียว ***
│
├── ros2_nodes/
│   ├── lotus_robot_node.py           ← Brain หลัก (production)
│   ├── lotus_motion_node.py          ← Monitor odom/imu
│   ├── lotus_peripheral_node.py      ← ทดสอบ peripheral
│   ├── lotus_LED_node.py             ← สั่ง LED ผ่าน /led/state
│   ├── README.md
│   ├── launch/
│   │   ├── robot.launch.py
│   │   └── peripheral_test.launch.py
│   └── examples/
│       ├── example_motor_mission.py
│       ├── example_servo_test.py
│       └── example_buzzer_melody.py
│
├── README.md
├── .gitignore
└── LICENSE
```

---

## Boot Mode Selection / เลือกโหมดตอน boot

**EN:** After powering on, the LotusRosDev Shield shows a 5-second countdown. Press BTN to enter Calibration mode, or wait for ROS2 mode to start automatically.

**TH:** หลังเปิดไฟ LotusRosDev Shield จะแสดง countdown 5 วินาที กด BTN เพื่อเข้า Calibration หรือรอให้เข้า ROS2 mode อัตโนมัติ

```
┌──────────────────────┐
│  LotusRosDev Shield  │
├──────────────────────┤
│  Hold BTN = CALIB    │
│  Release  = ROS2     │
│                      │
│  Auto ROS2 in 4s     │  ← นับถอยหลัง
└──────────────────────┘

กด BTN → เสียง โด เร มี → CALIBRATION MODE
ไม่กด  → เสียงสั้น 1 ครั้ง → ROS2 MODE
```

---

## Quick Start / เริ่มต้นใช้งาน

### 1. ติดตั้ง PlatformIO

```bash
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
echo 'export PATH="$PATH:$HOME/.platformio/penv/bin"' >> ~/.bashrc
source ~/.bashrc
```

### 2. แก้ค่า config

**EN:** Edit `config/custom/myrobot_config.h` to match your robot's measurements.
**TH:** แก้ค่าใน `config/custom/myrobot_config.h` ให้ตรงกับหุ่นยนต์จริง

```cpp
// ค่าสำคัญที่ต้องตรวจ
#define WHEEL_DIAMETER      0.07   // เส้นผ่านศูนย์กลางล้อ (เมตร)
#define LR_WHEELS_DISTANCE  0.21   // ระยะห่างล้อซ้าย-ขวา (เมตร)
#define COUNTS_PER_REV1     920    // encoder M1 — ได้จาก calibration Menu2
#define COUNTS_PER_REV2     906    // encoder M2 — ได้จาก calibration Menu2
#define AGENT_IP    {192, 168, 0, 108}   // IP Jetson Nano
```

### 3. Upload Firmware

```bash
cd firmware
pio run -e myrobot -t upload
```

### 4. รัน micro-ROS Agent บน Jetson Nano

```bash
# USB Serial (default)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/tty_esp32

# WiFi UDP
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 5. รัน bringup

```bash
ros2 launch linorobot2_bringup bringup.launch.py base_serial_port:=/dev/tty_esp32
```

---

## Calibration Mode / โหมด Calibration

**EN:** Calibration mode runs entirely on the LotusRosDev Shield — no PC or ROS2 required. Use OLED display, POT, and BTN to operate.

**TH:** Calibration mode ทำงานบน LotusRosDev Shield ล้วน ๆ ไม่ต้องการ PC หรือ ROS2 ใช้ OLED + POT + BTN ควบคุม

### การควบคุม

| อุปกรณ์ | หน้าที่ |
|---------|---------|
| POT (GPIO35) | หมุนเลื่อน cursor เมนู / scroll ผล / ปรับ servo angle |
| BTN (GPIO27) | กดยืนยัน / หยุด / hold 3s = ออก sensor check |
| OLED 128×64 | แสดงเมนูและผลลัพธ์ทั้งหมด |
| Buzzer | เสียงยืนยัน / เตือน / จบภารกิจ |

### เมนู Calibration

| เมนู | ชื่อ | การทำงาน |
|------|------|----------|
| 1 | Spin motors | FWD→LEFT→RIGHT→STOP ทดสอบทิศทางมอเตอร์ |
| 2 | Sample CPR | M1 หมุน 20 วิ → M2 อัตโนมัติ คำนวณ COUNTS_PER_REV |
| 3 | Sample CPR x5 | 5 รอบ สลับอัตโนมัติ แสดง CPR แต่ละรอบ + avg |
| 4 | Servo check | POT ควบคุม 0–180° servo ทั้ง 3 พร้อมกัน |
| 5 | Sensor check | แสดงค่า sensor ทุกตัว real-time (hold BTN 3s = ออก) |
| 6 | LED test | LED กระพริบ POT ปรับความเร็ว 100–1000ms |

### ขั้นตอน Calibrate CPR

```
1. Upload firmware (unified)
2. ยก LotusRosDev Shield ให้ล้อลอย
3. เปิดไฟ → กด BTN ภายใน 5 วินาที → เข้า CALIBRATION MODE
4. เลือก Menu 2 หรือ Menu 3
5. กด BTN ยืนยัน → มอเตอร์หมุนอัตโนมัติ
6. อ่านค่า CPR จาก OLED (AVG M1:xxx M2:xxx)
7. แก้ใน myrobot_config.h:
     #define COUNTS_PER_REV1  xxx
     #define COUNTS_PER_REV2  xxx
8. Upload firmware ใหม่
```

---

## Topics / ช่องทางสื่อสาร

| Topic | ทิศทาง | Type | คำอธิบาย |
|-------|--------|------|----------|
| `/cmd_vel` | Host→Shield | Twist | สั่งความเร็ว |
| `/odom/unfiltered` | Shield→Host | Odometry | ตำแหน่งจาก encoder |
| `/imu/data` | Shield→Host | Imu | accelerometer + gyroscope |
| `/imu/mag` | Shield→Host | MagneticField | magnetometer |
| `/ir/raw` | Shield→Host | Float32MultiArray | [lv,rv,lraw,rraw] |
| `/button/state` | Shield→Host | Bool | สถานะปุ่มกด |
| `/pot/raw` | Shield→Host | Float32MultiArray | [voltage,raw] |
| `/ir/config` | Host→Shield | Float32 | ตั้งค่า threshold IR (V) |
| `/servo/cmd` | Host→Shield | Float32MultiArray | [s1,s2,s3] องศา 0–180 |
| `/buzzer/play` | Host→Shield | Float32MultiArray | [freq_hz,duration_ms] |
| `/led/state` | Host→Shield | Bool | True=ติด False=ดับ |

### ตัวอย่างคำสั่งจาก terminal

```bash
# LED ติด/ดับ
ros2 topic pub /led/state std_msgs/msg/Bool "data: true" --once
ros2 topic pub /led/state std_msgs/msg/Bool "data: false" --once

# บี๊บ 1 ครั้ง
ros2 topic pub /buzzer/play std_msgs/msg/Float32MultiArray "data: [1000.0, 300.0]" --once

# หมุน servo ทุกตัวไป 90°
ros2 topic pub /servo/cmd std_msgs/msg/Float32MultiArray "data: [90.0, 90.0, 90.0]" --once
```

---

## GPIO Pinout / แผนผัง Pin LotusRosDev Shield

```
LotusRosDev Shield (ESP32 DevKit V1 30-pin)
                    ┌──────────────────┐
              3V3 ──┤                  ├── GND
              GND ──┤                  ├── 23  LED (status + /led/state)
    VP(36) IR_R  ──┤                  ├── 22  SCL I2C (OLED+IMU)
    VN(39) IR_L  ──┤                  ├── TX0 micro-ROS Serial
           34    ──┤                  ├── RX0 micro-ROS Serial
           35 POT──┤                  ├── 21  SDA I2C (OLED+IMU)
       32 SERVO1 ──┤                  ├── 19  (ว่าง)
       33 SERVO2 ──┤                  ├── 18  BUZZER
       25 ENC2B  ──┤                  ├──  5  SERVO3
       26 ENC2A  ──┤                  ├── 17  M2_INB
       27 BTN    ──┤                  ├── 16  M2_INA
       14 ENC1B  ──┤                  ├──  4  M2_PWM
       12 ENC1A  ──┤                  ├──  2  M1_INA
           GND   ──┤                  ├── 15  M1_INB
       13 M1_PWM ──┤                  ├── GND
              5V ──┤                  ├── VIN
                    └──────────────────┘

GPIO ที่ยังว่าง: 19 (output), 34 (input-only)
```

---

## ROS2 Nodes / Python Nodes

**EN:** See `ros2_nodes/README.md` for detailed usage.
**TH:** ดูรายละเอียดการใช้งานใน `ros2_nodes/README.md`

```bash
# รัน robot (production)
source /opt/ros/humble/setup.bash   # หรือ galactic
python3 ros2_nodes/lotus_robot_node.py

# monitor odom/imu
python3 ros2_nodes/lotus_motion_node.py

# สั่ง LED
ros2 topic pub /led/state std_msgs/msg/Bool "data: true" --once
```

---

## Troubleshooting / แก้ปัญหา

### OLED ไม่แสดง countdown หลัง boot
- ตรวจสาย I2C: SDA=GPIO21, SCL=GPIO22
- ตรวจ address SSD1306 = 0x3C

### กด BTN แล้วไม่เข้า calibration
- กด **ค้างไว้** ให้ตรงกับช่วง countdown 5 วินาที
- ตรวจ BTN ใน Menu5 Sensor check ก่อน

### มอเตอร์ไม่หมุนใน Menu1 Spin
- ตรวจสายต่อ Motor driver: PWM, IN_A, IN_B
- ตรวจ `MOTOR1_INV` / `MOTOR2_INV` ใน `myrobot_config.h`
- Menu1 มี drain BTN แล้ว ถ้ายังไม่หมุน ตรวจ power supply

### micro-ROS ไม่ connect (ROS2 mode)
- ตรวจ micro-ROS agent รันอยู่บน Jetson
- ตรวจ Serial port: `ls /dev/tty*`
- ตรวจ transport ใน `myrobot_config.h` ตรงกับ `platformio.ini`

### IMU fail ตอน boot
- ตรวจสาย SDA/SCL และ power 3.3V
- address MPU6050 = 0x68

### Servo กระตุกตอน boot
- ปกติแล้ว servo จะ **คลายตัวอิสระ** ตอน boot
- ถ้ากระตุก ตรวจ power supply 5V servo

---

## License

Apache License 2.0 — based on [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) by Juan Miguel Jimeno

Modified and extended for **LotusRosDev Shield** by KLS / RAIL Platform

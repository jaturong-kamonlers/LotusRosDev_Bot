# LotusRosDev Bot — ROS2 Nodes

## โครงสร้าง

```
ros2_nodes/
├── lotus_robot_node.py        ← Brain หลัก (production)
├── lotus_motion_node.py       ← Monitor odom/imu
├── lotus_peripheral_node.py   ← ทดสอบ peripheral
├── launch/
│   ├── robot.launch.py        ← รัน production (robot + motion monitor)
│   └── peripheral_test.launch.py ← ทดสอบ peripheral
└── examples/
    ├── example_motor_mission.py  ← ตัวอย่าง: เดินหน้า 1m เลี้ยวขวา 90°
    ├── example_servo_test.py     ← ตัวอย่าง: servo 0→90→0
    └── example_buzzer_melody.py  ← ตัวอย่าง: เล่นเสียง โด เร มี
```

---

## ก่อนรัน — ต้องมี micro-ROS agent

```bash
# Serial transport
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# WiFi transport
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

---

## การรันตามสถานการณ์

### 1. รัน Robot จริง (production)
```bash
# terminal 1: agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# terminal 2: robot
python3 lotus_robot_node.py

# terminal 3: monitor (optional)
python3 lotus_motion_node.py   # monitor mode — ปลอดภัยรันคู่ได้
```

กด BTN บนหุ่น เพื่อ START/STOP หมุน POT เพื่อปรับ IR threshold

### 2. ทดสอบ peripheral
```bash
python3 lotus_peripheral_node.py
```

### 3. ทดสอบ motion (control mode) — รันคนเดียวเท่านั้น
```bash
python3 lotus_motion_node.py --control
```

### 4. ตัวอย่าง mission
```bash
# เดินหน้า 1m แล้วเลี้ยวขวา 90°
python3 examples/example_motor_mission.py

# ทดสอบ servo
python3 examples/example_servo_test.py

# เล่นเสียง
python3 examples/example_buzzer_melody.py
```

---

## Topics สรุป

| Topic | ทิศทาง | Type | ใช้โดย |
|-------|--------|------|--------|
| `/cmd_vel` | →ESP32 | Twist | robot_node เท่านั้น |
| `/odom/unfiltered` | ESP32→ | Odometry | motion_node, robot_node |
| `/imu/data` | ESP32→ | Imu | motion_node |
| `/imu/mag` | ESP32→ | MagneticField | motion_node |
| `/ir/raw` | ESP32→ | Float32MultiArray | robot_node, peripheral_node |
| `/button/state` | ESP32→ | Bool | robot_node, peripheral_node |
| `/pot/raw` | ESP32→ | Float32MultiArray | robot_node, peripheral_node |
| `/ir/config` | →ESP32 | Float32 | robot_node, peripheral_node |
| `/servo/cmd` | →ESP32 | Float32MultiArray | robot_node, peripheral_node |
| `/buzzer/play` | →ESP32 | Float32MultiArray | robot_node, peripheral_node |

---

## ข้อควรระวัง

- `/cmd_vel` มีได้แค่ 1 publisher ต่อครั้ง ห้ามรัน robot_node + motion_node(control) + example_motor_mission พร้อมกัน
- `/buzzer/play` หลาย node publish ได้ แต่เสียงจะทับกัน
- lotus_motion_node.py ในโหมด monitor ปลอดภัยรันคู่กับทุก node

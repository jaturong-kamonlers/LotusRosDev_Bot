// =============================================================================
//  lotusrosbot_config.h — ค่าตั้งของหุ่น LotusRosBot (ไฟล์เดียวที่ต้องแก้)
//  บอร์ด: LotusRosDev Shield (ESP32 DevKit V1 30-pin)
//  ขา, PID, ล้อ, encoder, อุปกรณ์เสริม — แก้ที่นี่แล้ว build/flash ใหม่ (tools/flash.sh upload)
// =============================================================================

#ifndef LOTUSROSBOT_CONFIG_H
#define LOTUSROSBOT_CONFIG_H

// ─────────────────────────────────────────────────────────────────────────────
//  การเชื่อมต่อกับ ROS 2 (micro-ROS)
// ─────────────────────────────────────────────────────────────────────────────
// #define USE_WIFI_TRANSPORT     // WiFi UDP — ต้องแก้ platformio.ini และ WIFI_AP_LIST ด้านล่างด้วย
#define USE_SERIAL_TRANSPORT      // USB Serial ไป Jetson (ค่าที่ใช้จริง)

#define BAUDRATE   115200         // ต้องตรงกับ agent (-b 115200) · 460800/921600 เคยลองแล้ว handshake ไม่ติด (2026-07-04)
#define NODE_NAME  "lotusrosbot_base"   // ชื่อ node ของบอร์ดใน ROS 2 (ros2 node list)

#ifndef ESP32
#define ESP32
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  ฐานหุ่น / Base
// ─────────────────────────────────────────────────────────────────────────────
#define ROBOT_BASE  DIFFERENTIAL_DRIVE      // หุ่นขับล้อ 2 ข้าง (ค่าใน Kinematics)
#define USE_GENERIC_2_IN_MOTOR_DRIVER       // ไดรเวอร์มอเตอร์ L298N (PWM + 2 ขาทิศ)

// ─────────────────────────────────────────────────────────────────────────────
//  IMU — MPU6050 ผ่าน I2C (ใช้สาย I2C ร่วมกับจอ OLED)
// ─────────────────────────────────────────────────────────────────────────────
#define USE_MPU6050_IMU
#define SDA_PIN   21
#define SCL_PIN   22

// ─────────────────────────────────────────────────────────────────────────────
//  PID ล้อซ้าย (L) / ล้อขวา (R) — เดินสั่น: ลด K_P · ตอบสนองช้า: เพิ่ม K_I
// ─────────────────────────────────────────────────────────────────────────────
#define K_P_L   0.35
#define K_I_L   0.40
#define K_D_L   0.10

#define K_P_R   0.35
#define K_I_R   0.40
#define K_D_R   0.10

// ─────────────────────────────────────────────────────────────────────────────
//  มอเตอร์
// ─────────────────────────────────────────────────────────────────────────────
#define MOTOR_MAX_RPM               110   // RPM สูงสุดที่แรงดันใช้งาน
#define MAX_RPM_RATIO_L            0.85   // ใช้ได้ไม่เกิน 85% ของ RPM สูงสุด (ล้อซ้าย)
#define MAX_RPM_RATIO_R            0.85   // (ล้อขวา)
#define MOTOR_OPERATING_VOLTAGE      12   // แรงดันตามสเปกมอเตอร์ (V)
#define MOTOR_POWER_MAX_VOLTAGE      12   // แรงดันที่จ่ายให้มอเตอร์จริง (V)
#define MOTOR_POWER_MEASURED_VOLTAGE 12.0 // วัดด้วยมัลติมิเตอร์ (V) — ใช้ในเมนูคาลิเบรต 2–3

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder — จำนวน tick ต่อรอบล้อ (วัดจากเมนูคาลิเบรต 7 "Count ticks" แม่นที่สุด)
// ─────────────────────────────────────────────────────────────────────────────
#define COUNTS_PER_REV1   958   // ล้อซ้าย (M1) — CPR x5 avg=958 (2026-07-17)
#define COUNTS_PER_REV2   925   // ล้อขวา (M2) — CPR x5 avg=925 (2026-07-17)
#define COUNTS_PER_REV3   240   // ไม่ใช้
#define COUNTS_PER_REV4   240   // ไม่ใช้

// ─────────────────────────────────────────────────────────────────────────────
//  ขนาดล้อ (เมตร)
// ─────────────────────────────────────────────────────────────────────────────
#define WHEEL_DIAMETER      0.07   // 7 cm
#define LR_WHEELS_DISTANCE  0.21   // ระยะกึ่งกลางล้อซ้าย-ขวา 21 cm

// ─────────────────────────────────────────────────────────────────────────────
//  PWM มอเตอร์ — ปกติไม่ต้องแก้
// ─────────────────────────────────────────────────────────────────────────────
#define PWM_BITS        10       // 0–1023
#define PWM_FREQUENCY   20000    // 20 kHz (เกินช่วงที่หูได้ยิน)
#define PWM_MAX  pow(2, PWM_BITS) - 1
#define PWM_MIN -(pow(2, PWM_BITS) - 1)

// =============================================================================
//  ขา GPIO — ⚠️ = strapping pin ของ ESP32 (มีผลตอนบูต/flash) อย่าย้ายถ้าไม่จำเป็น
// =============================================================================
#define LED_PIN   23               // ไฟสถานะ ROS2 + รับคำสั่ง /led/state

// มอเตอร์ซ้าย (M1)
#define MOTOR1_PWM    13
#define MOTOR1_IN_A    2           // ⚠️
#define MOTOR1_IN_B   15           // ⚠️
#define MOTOR1_INV   true          // กลับทิศ

// มอเตอร์ขวา (M2)
#define MOTOR2_PWM    4
#define MOTOR2_IN_A  16
#define MOTOR2_IN_B  17
#define MOTOR2_INV   true

// Encoder ซ้าย — ⚠️ GPIO12 ถ้าถูกดึง high ตอนบูต flash จะเป็น 1.8V แล้วบูตไม่ขึ้น (start.sh ตรวจให้)
#define MOTOR1_ENCODER_A   12
#define MOTOR1_ENCODER_B   14
#define MOTOR1_ENCODER_INV false

// Encoder ขวา
#define MOTOR2_ENCODER_A   26
#define MOTOR2_ENCODER_B   25
#define MOTOR2_ENCODER_INV true

// =============================================================================
//  อุปกรณ์เสริม — ปิดได้ด้วยการใส่ // หน้า #define USE_xxx
// =============================================================================
#define USE_BUZZER                 // /buzzer/play [freq_hz, duration_ms]
#define BUZZER_PIN      18
#define BUZZER_CHANNEL   7         // LEDC channel 7 (ไม่ชนกับมอเตอร์/servo)

#define USE_SERVO                  // /servo/cmd [s1, s2, s3] องศา 0–180
#define SERVO1_PIN  32             // หนีบ
#define SERVO2_PIN  33             // ยก
#define SERVO3_PIN   5             // ⚠️ ตัวที่ 3

#define USE_IR_SENSORS             // /ir/raw [left_V, right_V, left_raw, right_raw]
#define IR_LEFT_PIN   36           // ADC1 input-only
#define IR_RIGHT_PIN  39

#define USE_POTENTIOMETER          // /pot/raw [V, raw]
#define POT_PIN   35

#define USE_BUTTON                 // /button/state (ส่งเมื่อเปลี่ยน) · กดค้างตอนบูต = โหมดคาลิเบรต
#define BUTTON_PIN  27

// =============================================================================
//  ใช้เฉพาะโหมด WiFi / syslog (ตอนนี้ปิดอยู่ — ไม่มีผลกับเฟิร์มแวร์ที่ build)
// =============================================================================
#define AGENT_IP    {192, 168, 31, 186}   // Jetson (micro-ROS agent แบบ UDP)
#define AGENT_PORT  8888

#ifdef USE_WIFI_TRANSPORT
#define WIFI_AP_LIST  {{"YOUR_SSID", "YOUR_PASSWORD"}, {NULL}}   // ⚠️ อย่า commit รหัสจริง
#define WIFI_MONITOR  2
#endif

// #define USE_ARDUINO_OTA          // อัปเดตเฟิร์มแวร์ผ่าน WiFi
// #define USE_SYSLOG               // ส่ง log ผ่าน UDP
#define SYSLOG_SERVER   {192, 168, 31, 147}   // HOST
#define SYSLOG_PORT     514
#define DEVICE_HOSTNAME "lotusrosbot"
#define APP_NAME        "firmware"

#endif // LOTUSROSBOT_CONFIG_H

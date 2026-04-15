// =============================================================================
//  myrobot_config.h — LotusRosDev Bot Configuration
//  บอร์ด: ESP32 DevKit V1 30-pin
//  แก้ไขที่ไฟล์นี้ที่เดียว ไม่ต้องแตะไฟล์อื่น
// =============================================================================

#ifndef MYROBOT_CONFIG_H
#define MYROBOT_CONFIG_H

// ─────────────────────────────────────────────────────────────────────────────
//  Transport — เลือก 1 อัน comment อีกอัน
//  แก้ platformio.ini ให้ตรงกันด้วย (ดูหัวข้อ platformio.ini ด้านล่าง)
// ─────────────────────────────────────────────────────────────────────────────
// #define USE_WIFI_TRANSPORT     // WiFi UDP — ต้องกำหนด WIFI_AP_LIST ด้านล่าง
  #define USE_SERIAL_TRANSPORT   // USB Serial — ค่าเริ่มต้น ใช้สาย USB

// platformio.ini สำหรับ Serial:
//   board_microros_transport = serial
//   build_flags = -I ../config -D USE_MYROBOT_CONFIG
//
// platformio.ini สำหรับ WiFi:
//   board_microros_transport = wifi
//   build_flags = -I ../config -D USE_MYROBOT_CONFIG -D USE_WIFI_TRANSPORT

// ─────────────────────────────────────────────────────────────────────────────
//  ระบบ / System
// ─────────────────────────────────────────────────────────────────────────────
#ifndef ESP32
#define ESP32                    // บอร์ด ESP32
#endif

#define BAUDRATE        115200   // baud rate สำหรับ Serial monitor
#define LINO_BASE       DIFFERENTIAL_DRIVE   // ประเภทหุ่น: ขับล้อ 2 ข้าง
#define USE_GENERIC_2_IN_MOTOR_DRIVER        // Motor driver แบบ 2 สาย direction (L298N)

// ─────────────────────────────────────────────────────────────────────────────
//  IMU — MPU6050 ผ่าน I2C
// ─────────────────────────────────────────────────────────────────────────────
#define USE_MPU6050_IMU          // ใช้ MPU6050 (ถ้าเปลี่ยน IMU ให้แก้ที่นี่)
#define SDA_PIN   21             // ขา SDA ของ I2C (ร่วมกับ OLED)
#define SCL_PIN   22             // ขา SCL ของ I2C (ร่วมกับ OLED)

// ─────────────────────────────────────────────────────────────────────────────
//  PID Controller — มอเตอร์ซ้าย (L) และขวา (R)
//  ปรับค่าหลังจาก Calibrate CPR แล้ว ถ้าหุ่นเดินสั่น ลด K_P ลง
//  ถ้าตอบสนองช้า เพิ่ม K_I ขึ้น
// ─────────────────────────────────────────────────────────────────────────────
#define K_P_L   0.35   // Proportional gain มอเตอร์ซ้าย
#define K_I_L   0.40   // Integral gain มอเตอร์ซ้าย
#define K_D_L   0.10   // Derivative gain มอเตอร์ซ้าย

#define K_P_R   0.35   // Proportional gain มอเตอร์ขวา
#define K_I_R   0.40   // Integral gain มอเตอร์ขวา
#define K_D_R   0.10   // Derivative gain มอเตอร์ขวา

// ─────────────────────────────────────────────────────────────────────────────
//  มอเตอร์ / Motor Specification
//  ดูสเปคจากผู้ผลิตมอเตอร์
// ─────────────────────────────────────────────────────────────────────────────
#define MOTOR_MAX_RPM               110   // RPM สูงสุดของมอเตอร์ที่แรงดันใช้งาน
#define MAX_RPM_RATIO_L            0.85   // % ของ RPM สูงสุดที่ PID ใช้ (ซ้าย)
#define MAX_RPM_RATIO_R            0.85   // % ของ RPM สูงสุดที่ PID ใช้ (ขวา)
#define MOTOR_OPERATING_VOLTAGE      12   // แรงดันใช้งานมอเตอร์ (V) ตามสเปค
#define MOTOR_POWER_MAX_VOLTAGE      12   // แรงดัน supply จริงที่ให้มอเตอร์ (V)
#define MOTOR_POWER_MEASURED_VOLTAGE 12.0 // แรงดันที่วัดได้จริง (V) — วัดด้วย multimeter

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder — จำนวน pulse ต่อรอบ
//  ได้จากการ Calibrate ด้วย calibration/src/firmware.ino Menu 2 หรือ 3
//  ค่าเริ่มต้นนี้วัดจากหุ่นจริงแล้ว
// ─────────────────────────────────────────────────────────────────────────────
#define COUNTS_PER_REV1   997   // ล้อซ้าย (M1) — วัดจาก Sample CPR
#define COUNTS_PER_REV2   979   // ล้อขวา (M2) — วัดจาก Sample CPR
#define COUNTS_PER_REV3   240   // ไม่ใช้ (สำรอง)
#define COUNTS_PER_REV4   240   // ไม่ใช้ (สำรอง)

// ─────────────────────────────────────────────────────────────────────────────
//  ขนาดล้อและระยะห่าง / Wheel Geometry
//  วัดจากหุ่นจริง หน่วยเป็นเมตร
// ─────────────────────────────────────────────────────────────────────────────
#define WHEEL_DIAMETER      0.07   // เส้นผ่านศูนย์กลางล้อ (เมตร) = 7 cm
#define LR_WHEELS_DISTANCE  0.21   // ระยะห่างกึ่งกลางล้อซ้าย-ขวา (เมตร) = 21 cm

// ─────────────────────────────────────────────────────────────────────────────
//  PWM Settings
//  ปกติไม่ต้องแก้
// ─────────────────────────────────────────────────────────────────────────────
#define PWM_BITS        10       // ความละเอียด PWM (10-bit = 0–1023)
#define PWM_FREQUENCY   20000   // ความถี่ PWM (Hz) — 20kHz เกิน range ได้ยินมนุษย์

// =============================================================================
//  GPIO Pinout — ESP32 DevKit V1 30-pin
//  ⚠️  ห้ามเปลี่ยน pin ที่มี * ถ้าไม่จำเป็น เพราะเป็น strapping pin
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
//  LED Status — แสดงสถานะ ROS2 และรับคำสั่งจาก /led/state
// ─────────────────────────────────────────────────────────────────────────────
#define LED_PIN   23   // GPIO23 — LED ติดแสดง ROS2 connected, กระพริบตาม /cmd_vel

// ─────────────────────────────────────────────────────────────────────────────
//  มอเตอร์ซ้าย / Motor Left (M1)
//  ⚠️  GPIO2, GPIO15 เป็น strapping pin — ระวังตอน upload
// ─────────────────────────────────────────────────────────────────────────────
#define MOTOR1_PWM    13   // PWM ความเร็วมอเตอร์ซ้าย
#define MOTOR1_IN_A    2   // ⚠️ ทิศทาง A (strapping pin — ระวังตอน flash)
#define MOTOR1_IN_B   15   // ⚠️ ทิศทาง B (strapping pin)
#define MOTOR1_INV   true  // กลับทิศมอเตอร์ซ้าย (true = กลับ)

// ─────────────────────────────────────────────────────────────────────────────
//  มอเตอร์ขวา / Motor Right (M2)
// ─────────────────────────────────────────────────────────────────────────────
#define MOTOR2_PWM    4    // PWM ความเร็วมอเตอร์ขวา
#define MOTOR2_IN_A  16    // ทิศทาง A
#define MOTOR2_IN_B  17    // ทิศทาง B
#define MOTOR2_INV   true  // กลับทิศมอเตอร์ขวา (true = กลับ)

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder มอเตอร์ซ้าย / Left Motor Encoder
//  ⚠️  GPIO12 เป็น strapping pin MTDI — ถอดสายตอน upload ถ้า upload ไม่ติด
// ─────────────────────────────────────────────────────────────────────────────
#define MOTOR1_ENCODER_A   12   // ⚠️ Encoder channel A (strapping pin)
#define MOTOR1_ENCODER_B   14   // Encoder channel B
#define MOTOR1_ENCODER_INV false // ทิศทาง encoder ซ้าย (false = ปกติ)

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder มอเตอร์ขวา / Right Motor Encoder
// ─────────────────────────────────────────────────────────────────────────────
#define MOTOR2_ENCODER_A   26   // Encoder channel A
#define MOTOR2_ENCODER_B   25   // Encoder channel B
#define MOTOR2_ENCODER_INV true  // กลับทิศ encoder ขวา (true = กลับ)

// ─────────────────────────────────────────────────────────────────────────────
//  PWM Limits — คำนวณจาก PWM_BITS อัตโนมัติ ไม่ต้องแก้
// ─────────────────────────────────────────────────────────────────────────────
#define PWM_MAX  pow(2, PWM_BITS) - 1    //  1023 (10-bit)
#define PWM_MIN -(pow(2, PWM_BITS) - 1)  // -1023

// =============================================================================
//  อุปกรณ์เสริม / Peripheral Devices
//  เปิด/ปิดโดยการ comment/uncomment #define USE_xxx
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
//  Buzzer Passive — เล่นเสียงผ่าน LEDC PWM
//  รับคำสั่งจาก /buzzer/play [freq_hz, duration_ms]
// ─────────────────────────────────────────────────────────────────────────────
#define USE_BUZZER
#define BUZZER_PIN      18   // GPIO18 — Passive buzzer
#define BUZZER_CHANNEL   7   // LEDC channel 7 (หลีกเลี่ยงชนกับ motor/servo)

// ─────────────────────────────────────────────────────────────────────────────
//  Servo — ควบคุม 3 ตัวผ่าน ESP32Servo library
//  รับคำสั่งจาก /servo/cmd [s1_deg, s2_deg, s3_deg] ช่วง 0–180 องศา
// ─────────────────────────────────────────────────────────────────────────────
#define USE_SERVO
#define SERVO1_PIN  32   // GPIO32 — Servo ตัวที่ 1
#define SERVO2_PIN  33   // GPIO33 — Servo ตัวที่ 2
#define SERVO3_PIN   5   // ⚠️ GPIO5 — Servo ตัวที่ 3 (strapping pin — เตือนตอน boot)

// ─────────────────────────────────────────────────────────────────────────────
//  IR Analog Sensors — อ่าน analog 2 ช่อง
//  Publish ไปที่ /ir/raw [left_V, right_V, left_raw, right_raw]
//  ถ้าต้องการเปลี่ยน pin → แก้ IR_LEFT_PIN และ IR_RIGHT_PIN ที่นี่ที่เดียว
// ─────────────────────────────────────────────────────────────────────────────
#define USE_IR_SENSORS
#define IR_LEFT_PIN   36   // GPIO36 — ADC1 ch0 input-only (IR ซ้าย)
#define IR_RIGHT_PIN  39   // GPIO39 — ADC1 ch3 input-only (IR ขวา)

// ─────────────────────────────────────────────────────────────────────────────
//  Potentiometer — อ่าน analog 1 ช่อง
//  Publish ไปที่ /pot/raw [voltage_V, raw_0_4095]
// ─────────────────────────────────────────────────────────────────────────────
#define USE_POTENTIOMETER
#define POT_PIN   35   // GPIO35 — ADC1 ch7 input-only

// ─────────────────────────────────────────────────────────────────────────────
//  Button — ปุ่มกด active LOW (มี internal pull-up)
//  Publish ไปที่ /button/state (Bool) เมื่อสถานะเปลี่ยน
// ─────────────────────────────────────────────────────────────────────────────
#define USE_BUTTON
#define BUTTON_PIN  27   // GPIO27 — ปุ่มกด (ต่อ GND ฝั่งหนึ่ง)

// =============================================================================
//  Network — micro-ROS Agent และ WiFi
//  ใช้เฉพาะเมื่อ USE_WIFI_TRANSPORT เปิดอยู่
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
//  micro-ROS Agent IP — เครื่อง Jetson Nano หรือ PC ที่รัน agent
// ─────────────────────────────────────────────────────────────────────────────
#define AGENT_IP    {192, 168, 0, 108}   // ⚠️ แก้เป็น IP ของ Jetson/PC ตัวเอง
#define AGENT_PORT  8888                  // port ของ micro-ROS agent (default 8888)

// ─────────────────────────────────────────────────────────────────────────────
//  WiFi Access Point — ใช้เมื่อ USE_WIFI_TRANSPORT เปิด
//  ⚠️  อย่า commit password จริงขึ้น GitHub
// ─────────────────────────────────────────────────────────────────────────────
#define WIFI_AP_LIST  {{"YOUR_SSID", "YOUR_PASSWORD"}, {NULL}}
#define WIFI_MONITOR  2   // ตรวจสัญญาณ WiFi ทุก N วินาที

// ─────────────────────────────────────────────────────────────────────────────
//  OTA Update — อัปเดต firmware ผ่าน WiFi (ไม่ต้องต่อสาย USB)
//  เปิดใช้หลังจาก connect WiFi ได้แล้วครั้งแรก
// ─────────────────────────────────────────────────────────────────────────────
// #define USE_ARDUINO_OTA   // uncomment เพื่อเปิด OTA

// ─────────────────────────────────────────────────────────────────────────────
//  Syslog — ส่ง log ไปยัง server ผ่าน UDP (debug แบบ wireless)
// ─────────────────────────────────────────────────────────────────────────────
// #define USE_SYSLOG
#define SYSLOG_SERVER   {192, 168, 0, 108}   // IP ของ syslog server
#define SYSLOG_PORT     514                   // UDP port ของ syslog
#define DEVICE_HOSTNAME "linorobot2"          // ชื่ออุปกรณ์ใน log
#define APP_NAME        "hardware"            // ชื่อ app ใน log

#endif // MYROBOT_CONFIG_H

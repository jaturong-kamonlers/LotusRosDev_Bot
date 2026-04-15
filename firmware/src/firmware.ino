// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// ════════════════════════════════════════════════════════════════
//  LotusRosDev_Bot — Unified Firmware v3
//  บอร์ด: LotusRosDev Shield (พัฒนาจาก ESP32 DevKit V1 30-pin)
//
//  โหมดการทำงาน (เลือกตอน boot):
//    CALIBRATION — กด BTN ภายใน 5 วินาที → เข้า calibration menu
//    ROS2        — ไม่กด BTN → เข้า micro-ROS mode อัตโนมัติ
//
//  Topics:
//    Publishers : /odom/unfiltered, /imu/data, /imu/mag
//                 /ir/raw, /button/state, /pot/raw
//    Subscribers: /cmd_vel, /ir/config, /servo/cmd,
//                 /buzzer/play, /led/state
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>

#include "config.h"
#include "syslog.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#include "mag.h"

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

#include "wifis.h"
#include "ota.h"

#include <WiFi.h>
#include <Wire.h>
#include "SSD1306Wire.h"

#ifdef USE_SERVO
#include <ESP32Servo.h>
#endif

SSD1306Wire display(0x3c, SDA_PIN, SCL_PIN);


#ifdef WDT_TIMEOUT
#include <esp_task_wdt.h>
#endif

#ifdef USE_WIFI_TRANSPORT
static inline void set_microros_net_transports(IPAddress agent_ip, uint16_t agent_port)
{
    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;
    rmw_uros_set_custom_transport(
        false, (void *) &locator,
        platformio_transport_open, platformio_transport_close,
        platformio_transport_write, platformio_transport_read
    );
}
#endif

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif
#ifndef NODE_NAME
#define NODE_NAME "linorobot_base_node"
#endif
#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif
#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// ════════════════════════════════════════════════════════════════
//  ROS2 Publishers
// ════════════════════════════════════════════════════════════════
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_publisher_t ir_publisher;        // /ir/raw
rcl_publisher_t button_publisher;    // /button/state
rcl_publisher_t pot_publisher;       // /pot/raw
rcl_subscription_t led_subscriber;   // /led/state → GPIO23

// ════════════════════════════════════════════════════════════════
//  ROS2 Subscribers
// ════════════════════════════════════════════════════════════════
rcl_subscription_t twist_subscriber;
rcl_subscription_t ir_config_subscriber;   // /ir/config  (threshold V)
rcl_subscription_t servo_subscriber;       // /servo/cmd  [s1_deg, s2_deg, s3_deg]
rcl_subscription_t buzzer_subscriber;      // /buzzer/play [freq_hz, duration_ms]

// ════════════════════════════════════════════════════════════════
//  ROS2 Messages
// ════════════════════════════════════════════════════════════════
nav_msgs__msg__Odometry           odom_msg;
sensor_msgs__msg__Imu             imu_msg;
sensor_msgs__msg__MagneticField   mag_msg;
geometry_msgs__msg__Twist         twist_msg;

// IR: [ir_left_v, ir_right_v, ir_left_raw, ir_right_raw]
std_msgs__msg__Float32MultiArray  ir_msg;
float ir_msg_data[4];

// Pot: [pot_v, pot_raw]
std_msgs__msg__Float32MultiArray  pot_msg;
float pot_msg_data[2];

std_msgs__msg__Bool               button_msg;
std_msgs__msg__Bool               led_msg;         // /led/state
bool                              led_user_ctrl = false; // true = ผู้ใช้สั่งผ่าน /led/state
std_msgs__msg__Float32            ir_config_msg;      // threshold V
std_msgs__msg__Float32MultiArray  servo_cmd_msg;      // [s1, s2, s3] degrees
float servo_cmd_data[3];
std_msgs__msg__Float32MultiArray  buzzer_cmd_msg;     // [freq_hz, duration_ms]
float buzzer_cmd_data[2];

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     control_timer;

unsigned long long time_offset    = 0;
unsigned long      prev_cmd_time  = 0;
unsigned long      prev_odom_update = 0;

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// ════════════════════════════════════════════════════════════════
//  Hardware objects
// ════════════════════════════════════════════════════════════════
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P_L, K_I_L, K_D_L);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P_R, K_I_R, K_D_R);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO_L, MAX_RPM_RATIO_R,
    MOTOR_OPERATING_VOLTAGE, MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER, LR_WHEELS_DISTANCE
);

// ── Pointer arrays สำหรับ calibration menu ───────────────────
Motor   *motors[2]   = {&motor1_controller, &motor2_controller};
Encoder *encoders[2] = {&motor1_encoder,    &motor2_encoder};
PID     *pids[2]     = {&motor1_pid,        &motor2_pid};

Odometry odometry;
IMU imu;
MAG mag;

#ifdef USE_SERVO
Servo servo1, servo2, servo3;
#endif

// ── Calibration menu — include หลัง objects ทั้งหมด ─────────
// calib_menu.h ใช้: motor1/2_controller, encoders[], motors[],
//                   pids[], kinematics, servo1/2/3, display
#include "calib_menu.h"

// ── Calibration mode flag ─────────────────────────────────────
static bool calib_mode = false;

// ════════════════════════════════════════════════════════════════
//  Sensor state variables
// ════════════════════════════════════════════════════════════════
#ifdef USE_IR_SENSORS
float ir_left_v    = 0.0f;
float ir_right_v   = 0.0f;
int   ir_left_raw  = 0;
int   ir_right_raw = 0;
float ir_threshold = 2.0f;   // default threshold V — ปรับได้ผ่าน /ir/config
#endif

#ifdef USE_POTENTIOMETER
float pot_v   = 0.0f;
int   pot_raw = 0;
#endif

#ifdef USE_BUTTON
bool button_pressed = false;
bool button_prev    = false;
#endif

#ifdef USE_BUZZER
unsigned long buzzer_stop_ms = 0;   // millis() ที่ต้องหยุดเสียง (0 = ไม่มีเสียง)
#endif

// ════════════════════════════════════════════════════════════════
//  Function declarations
// ════════════════════════════════════════════════════════════════
void updateDisplay();
void updateDisplaySensors();
void readSensors();
void buzzerCheck();
void moveBase();
void publishData();
bool syncTime();
struct timespec getTime();
void rclErrorLoop();
void flashLED(int n_times);
void fullStop();
bool createEntities();
bool destroyEntities();

// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup()
{
#ifdef BOARD_INIT
    BOARD_INIT;
#endif

    Serial.begin(BAUDRATE);
    pinMode(LED_PIN, OUTPUT);

#ifdef SDA_PIN
#ifdef ESP32
    Wire.begin(SDA_PIN, SCL_PIN);
#else
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
#endif
#endif

#ifdef WDT_TIMEOUT
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
#endif

    // ── OLED init ────────────────────────────────────────────
    display.init();
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 10, "Init IMU...");
    display.display();
    delay(100);

    // ── IMU ──────────────────────────────────────────────────
    bool imu_ok = imu.init();
    if (!imu_ok) {
        display.drawString(0, 25, "--IMU FAIL--");
        display.drawString(0, 35, "ESP32 Stop process");
        display.display();
        while (1) { flashLED(3); }
    }

    display.clear();
    display.drawString(0, 0,  "IMU......OK");
    display.drawString(0, 10, "Wifi connecting...");
    display.display();

    WiFi.setSleep(false);
    initWifis();
    initOta();

    // ── Mag ──────────────────────────────────────────────────
    mag.init();

    // ── Servo ────────────────────────────────────────────────
#ifdef USE_SERVO
    // attach แล้ว detach ทันที → servo คลายตัวอิสระ ไม่ล็อคองศา
    // จะ attach กลับเมื่อรับ /servo/cmd ครั้งแรก
    servo1.attach(SERVO1_PIN); servo1.detach();
    servo2.attach(SERVO2_PIN); servo2.detach();
    servo3.attach(SERVO3_PIN); servo3.detach();
#endif

    // ── Buzzer ───────────────────────────────────────────────
#ifdef USE_BUZZER
    ledcSetup(BUZZER_CHANNEL, 2000, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWriteTone(BUZZER_CHANNEL, 0);
#endif

    // ── Button ───────────────────────────────────────────────
#ifdef USE_BUTTON
    pinMode(BUTTON_PIN, INPUT_PULLUP);
#endif

    // ── IR message memory ────────────────────────────────────
#ifdef USE_IR_SENSORS
    ir_msg.data.data     = ir_msg_data;
    ir_msg.data.size     = 4;
    ir_msg.data.capacity = 4;
#endif

    // ── Pot message memory ───────────────────────────────────
#ifdef USE_POTENTIOMETER
    pot_msg.data.data     = pot_msg_data;
    pot_msg.data.size     = 2;
    pot_msg.data.capacity = 2;
#endif

    // ── Servo cmd message memory ─────────────────────────────
    servo_cmd_msg.data.data     = servo_cmd_data;
    servo_cmd_msg.data.size     = 3;
    servo_cmd_msg.data.capacity = 3;

    // ── Buzzer cmd message memory ────────────────────────────
    buzzer_cmd_msg.data.data     = buzzer_cmd_data;
    buzzer_cmd_msg.data.size     = 2;
    buzzer_cmd_msg.data.capacity = 2;

    // ── Startup beep ─────────────────────────────────────────
#ifdef USE_BUZZER
    ledcWriteTone(BUZZER_CHANNEL, 1000);
    delay(100);
    ledcWriteTone(BUZZER_CHANNEL, 0);
#endif

    // ════════════════════════════════════════════════════════
    //  CALIBRATION COUNTDOWN — กด BTN ภายใน 5 วินาที
    //  ถ้าไม่กด → เข้า ROS2 mode อัตโนมัติ
    // ════════════════════════════════════════════════════════
    calib_mode = false;
    for (int i = 5; i > 0 && !calib_mode; i--) {
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 0,  "LotusRosDev Shield");
        display.drawLine(0, 11, 127, 11);
        display.drawString(64, 18, "Hold BTN = CALIB");
        display.drawString(64, 32, "Release  = ROS2");
        char cbuf[20];
        snprintf(cbuf, sizeof(cbuf), "Auto ROS2 in %ds", i);
        display.drawString(64, 48, cbuf);
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.display();

        // รอ 1 วินาที ตรวจ BTN ทุก 20ms
        unsigned long t0 = millis();
        while (millis() - t0 < 1000) {
            if (digitalRead(BUTTON_PIN) == LOW) {
                calib_mode = true;
                break;
            }
            delay(20);
        }
    }

    if (calib_mode) {
        // ── เข้า CALIBRATION MODE ──────────────────────────
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 16, "CALIBRATION MODE");
        display.drawString(64, 36, "Starting...");
        display.display();
        calib_beepEnterCalib();   // โด เร มี
        delay(500);
        // ไม่ init micro-ROS — Serial ยังว่างสำหรับ debug
        return;   // ออกจาก setup() ไปที่ loop()
    }

    // ── เข้า ROS2 MODE ─────────────────────────────────────
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 16, "ROS2 MODE");
    display.drawString(64, 36, "Connecting...");
    display.display();

#ifdef USE_BUZZER
    // beep สั้น = เข้า ROS2 mode
    ledcWriteTone(BUZZER_CHANNEL, 1800); delay(100);
    ledcWriteTone(BUZZER_CHANNEL, 0);
#endif

    state = WAITING_AGENT;

#ifdef USE_WIFI_TRANSPORT
    set_microros_net_transports(AGENT_IP, AGENT_PORT);
#else
    set_microros_serial_transports(Serial);
#endif

    updateDisplay();
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
}

// ════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════
void loop()
{
    // ── CALIBRATION MODE ─────────────────────────────────────
    if (calib_mode) {
        runCalibration();   // ใน calib_menu.h — วน menu ไม่ออก
        return;
    }

    // ── ROS2 MODE ────────────────────────────────────────────
    switch (state)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500,
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                        ? AGENT_AVAILABLE : WAITING_AGENT;
            );
            break;

        case AGENT_AVAILABLE:
            syslog(LOG_INFO, "%s agent available %lu", __FUNCTION__, millis());
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroyEntities();
            } else if (state == AGENT_CONNECTED) {
                updateDisplay();
            }
            break;

        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200,
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                        ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            );
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;

        case AGENT_DISCONNECTED:
            updateDisplay();
            syslog(LOG_INFO, "%s agent disconnected %lu", __FUNCTION__, millis());
            fullStop();
            destroyEntities();
            state = WAITING_AGENT;
            break;

        default:
            break;
    }

    // ── Sensor reads (non-blocking, throttled) ───────────────
    EXECUTE_EVERY_N_MS(50,  readSensors());
    EXECUTE_EVERY_N_MS(200, updateDisplaySensors());

    // ── Buzzer stop check ────────────────────────────────────
#ifdef USE_BUZZER
    EXECUTE_EVERY_N_MS(10, buzzerCheck());
#endif

    runWifis();
    runOta();

#ifdef WDT_TIMEOUT
    esp_task_wdt_reset();
#endif
}

// ════════════════════════════════════════════════════════════════
//  Callbacks — ROS2
// ════════════════════════════════════════════════════════════════
void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        moveBase();
        publishData();
    }
}

void twistCallback(const void * msgin)
{
    // LED ควบคุมผ่าน /led/state topic เท่านั้น
    // ไม่ toggle ที่นี่อีกต่อไป
    prev_cmd_time = millis();
}

void ledCallback(const void * msgin)
{
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    led_user_ctrl = true;                              // ผู้ใช้ควบคุม LED แล้ว
    digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
}

void irConfigCallback(const void * msgin)
{
#ifdef USE_IR_SENSORS
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    ir_threshold = msg->data;
    syslog(LOG_INFO, "ir_threshold=%.2f", ir_threshold);
#endif
}

void servoCallback(const void * msgin)
{
#ifdef USE_SERVO
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size < 3) return;
    // clamp 0–180 degrees
    int a1 = constrain((int)msg->data.data[0], 0, 180);
    int a2 = constrain((int)msg->data.data[1], 0, 180);
    int a3 = constrain((int)msg->data.data[2], 0, 180);
    // attach กลับก่อน write ทุกครั้ง (เพราะ detach ตอน boot)
    if (!servo1.attached()) servo1.attach(SERVO1_PIN);
    if (!servo2.attached()) servo2.attach(SERVO2_PIN);
    if (!servo3.attached()) servo3.attach(SERVO3_PIN);
    servo1.write(a1);
    servo2.write(a2);
    servo3.write(a3);
#endif
}

void buzzerCallback(const void * msgin)
{
#ifdef USE_BUZZER
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size < 2) return;
    uint32_t freq = (uint32_t)constrain(msg->data.data[0], 20.0f, 20000.0f);
    uint32_t dur  = (uint32_t)constrain(msg->data.data[1], 1.0f, 10000.0f);
    ledcWriteTone(BUZZER_CHANNEL, freq);
    buzzer_stop_ms = millis() + dur;
#endif
}

// ════════════════════════════════════════════════════════════════
//  Sensor read — non-blocking, called every 50ms
// ════════════════════════════════════════════════════════════════
void readSensors()
{
#ifdef USE_IR_SENSORS
    // dummy read เพื่อ settle ADC channel หลังสลับจาก POT (GPIO35)
    // ESP32 SAR ADC ใช้ multiplexer ตัวเดียว ค่าแรกหลังสลับ channel มักผิด
    analogRead(IR_LEFT_PIN);
    delayMicroseconds(50);   // 50µs settle — ไม่บล็อก control loop (20ms)

    // average 8 samples เพื่อลด noise และป้องกันค่า 0 หลุดมา
    int sum_l = 0, sum_r = 0;
    for (int i = 0; i < 8; i++) {
        sum_l += analogRead(IR_LEFT_PIN);
        sum_r += analogRead(IR_RIGHT_PIN);
    }
    ir_left_raw  = sum_l / 8;
    ir_right_raw = sum_r / 8;
    ir_left_v    = ir_left_raw  * (3.3f / 4095.0f);
    ir_right_v   = ir_right_raw * (3.3f / 4095.0f);

    // publish /ir/raw
    if (state == AGENT_CONNECTED) {
        ir_msg.data.data[0] = ir_left_v;
        ir_msg.data.data[1] = ir_right_v;
        ir_msg.data.data[2] = (float)ir_left_raw;
        ir_msg.data.data[3] = (float)ir_right_raw;
        RCSOFTCHECK(rcl_publish(&ir_publisher, &ir_msg, NULL));
    }
#endif

#ifdef USE_POTENTIOMETER
    int sum_p = 0;
    for (int i = 0; i < 4; i++) { sum_p += analogRead(POT_PIN); }
    pot_raw = sum_p / 4;
    pot_v   = pot_raw * (3.3f / 4095.0f);

    if (state == AGENT_CONNECTED) {
        pot_msg.data.data[0] = pot_v;
        pot_msg.data.data[1] = (float)pot_raw;
        RCSOFTCHECK(rcl_publish(&pot_publisher, &pot_msg, NULL));
    }
#endif

#ifdef USE_BUTTON
    bool current = (digitalRead(BUTTON_PIN) == LOW);   // active LOW (pull-up)
    if (current != button_prev) {
        button_pressed = current;
        button_prev    = current;
        if (state == AGENT_CONNECTED) {
            button_msg.data = button_pressed;
            RCSOFTCHECK(rcl_publish(&button_publisher, &button_msg, NULL));
        }
    }
#endif
}

// ════════════════════════════════════════════════════════════════
//  Buzzer stop check — non-blocking, called every 10ms
// ════════════════════════════════════════════════════════════════
void buzzerCheck()
{
#ifdef USE_BUZZER
    if (buzzer_stop_ms > 0 && millis() >= buzzer_stop_ms) {
        ledcWriteTone(BUZZER_CHANNEL, 0);
        buzzer_stop_ms = 0;
    }
#endif
}

// ════════════════════════════════════════════════════════════════
//  OLED — status only (เรียกตอน state เปลี่ยน)
// ════════════════════════════════════════════════════════════════
void updateDisplay()
{
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "IP: " + String(WiFi.localIP().toString().c_str()));
    if (state == AGENT_CONNECTED) {
        display.drawString(0, 13, "ROS2: OK");
    } else {
        display.drawString(0, 13, "ROS2: WAIT");
    }
    display.display();
}

// ════════════════════════════════════════════════════════════════
//  OLED — sensor values แบบที่ 3 (progress bar + ตัวเลข)
//  เรียกทุก 200ms จาก loop()
// ════════════════════════════════════════════════════════════════
void updateDisplaySensors()
{
    // ════ OLED 128×64 layout ════════════════════════════════
    // y= 0  ROS2 status + IP octet สุดท้าย
    // y=10  ─── divider
    // y=13  L: xx.xxV [xxxx]
    // y=25  [bar L — 100px wide]
    // y=37  R: xx.xxV [xxxx]
    // y=49  [bar R — 100px wide]  THR: xx.xxV (ขวา)
    // ════════════════════════════════════════════════════════

    display.clear();
    display.setFont(ArialMT_Plain_10);

    // ── row 0: ROS2 status + IP ──────────────────────────────
    String ip_short = WiFi.localIP().toString();
    int last_dot = ip_short.lastIndexOf('.');
    String status_str = (state == AGENT_CONNECTED) ? "OK" : "--";
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "ROS2:" + status_str);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(127, 0, ip_short.substring(last_dot + 1));
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    // ── divider ───────────────────────────────────────────────
    display.drawLine(0, 10, 127, 10);

#ifdef USE_IR_SENSORS
    // ── IR LEFT label + ค่า (บรรทัดข้อความ) ─────────────────
    // "L: 2.15V [2672]" — ซ้ายจอ
    char buf_l[24];
    snprintf(buf_l, sizeof(buf_l), "L:%4.2fV[%4d]", ir_left_v, ir_left_raw);
    display.drawString(0, 13, buf_l);

    // bar L (y=25, 100px กว้าง, 6px สูง) — ต่ำกว่าข้อความ
    int pct_l = (int)constrain(ir_left_v / 3.3f * 100.0f, 0, 100);
    display.drawRect(0, 25, 100, 6);
    display.fillRect(0, 25, pct_l, 6);

    // ── IR RIGHT label + ค่า ─────────────────────────────────
    char buf_r[24];
    snprintf(buf_r, sizeof(buf_r), "R:%4.2fV[%4d]", ir_right_v, ir_right_raw);
    display.drawString(0, 37, buf_r);

    // bar R (y=49)
    int pct_r = (int)constrain(ir_right_v / 3.3f * 100.0f, 0, 100);
    display.drawRect(0, 49, 100, 6);
    display.fillRect(0, 49, pct_r, 6);

    // ── threshold ขวาล่าง ────────────────────────────────────
    char buf_t[16];
    snprintf(buf_t, sizeof(buf_t), "T:%4.2f", ir_threshold);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(127, 49, buf_t);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

#else
    // ถ้าไม่มี IR แสดง pot และ button แทน
#ifdef USE_POTENTIOMETER
    display.drawString(0, 20, "POT:" + String(pot_v, 2) + "V [" + String(pot_raw) + "]");
#endif
#ifdef USE_BUTTON
    display.drawString(0, 36, "BTN:" + String(button_pressed ? "1" : "0"));
#endif
#endif

    display.display();
}

// ════════════════════════════════════════════════════════════════
//  createEntities — executor 7 handles
//  (1 timer + 4 subscribers + 2 publishers ไม่นับ = handles = sub+timer)
//  twist(1) + ir_config(1) + servo(1) + buzzer(1) + timer(1) = 5
//  + ir_pub, pot_pub, button_pub = publishers ไม่ใช้ handle
// ════════════════════════════════════════════════════════════════
bool createEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

    // ── Publishers ───────────────────────────────────────────
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_PREFIX "odom/unfiltered"));

    RCCHECK(rclc_publisher_init_default(
        &imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        TOPIC_PREFIX "imu/data"));

    RCCHECK(rclc_publisher_init_default(
        &mag_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        TOPIC_PREFIX "imu/mag"));

#ifdef USE_IR_SENSORS
    RCCHECK(rclc_publisher_init_default(
        &ir_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        TOPIC_PREFIX "ir/raw"));
#endif

#ifdef USE_BUTTON
    RCCHECK(rclc_publisher_init_default(
        &button_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        TOPIC_PREFIX "button/state"));
#endif

#ifdef USE_POTENTIOMETER
    RCCHECK(rclc_publisher_init_default(
        &pot_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        TOPIC_PREFIX "pot/raw"));
#endif

    // ── Subscribers ──────────────────────────────────────────
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_PREFIX "cmd_vel"));

#ifdef USE_IR_SENSORS
    RCCHECK(rclc_subscription_init_default(
        &ir_config_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        TOPIC_PREFIX "ir/config"));
#endif

#ifdef USE_SERVO
    RCCHECK(rclc_subscription_init_default(
        &servo_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        TOPIC_PREFIX "servo/cmd"));
#endif

#ifdef USE_BUZZER
    RCCHECK(rclc_subscription_init_default(
        &buzzer_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        TOPIC_PREFIX "buzzer/play"));
#endif

    // ── LED subscriber ───────────────────────────────────────
    RCCHECK(rclc_subscription_init_default(
        &led_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        TOPIC_PREFIX "led/state"));

    // ── Timer 50Hz ───────────────────────────────────────────
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));

    // ── Executor — handle count: 1 timer + 5 subs = 6 ───────
    // twist(1) + ir_config(1) + servo(1) + buzzer(1) + led(1) + timer(1)
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA));

#ifdef USE_IR_SENSORS
    RCCHECK(rclc_executor_add_subscription(
        &executor, &ir_config_subscriber, &ir_config_msg, &irConfigCallback, ON_NEW_DATA));
#endif

#ifdef USE_SERVO
    RCCHECK(rclc_executor_add_subscription(
        &executor, &servo_subscriber, &servo_cmd_msg, &servoCallback, ON_NEW_DATA));
#endif

#ifdef USE_BUZZER
    RCCHECK(rclc_executor_add_subscription(
        &executor, &buzzer_subscriber, &buzzer_cmd_msg, &buzzerCallback, ON_NEW_DATA));
#endif

    RCCHECK(rclc_executor_add_subscription(
        &executor, &led_subscriber, &led_msg, &ledCallback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();
    if (!led_user_ctrl) digitalWrite(LED_PIN, HIGH);  // ไม่ override ถ้า user สั่งแล้ว
    return true;
}

// ════════════════════════════════════════════════════════════════
//  destroyEntities
// ════════════════════════════════════════════════════════════════
bool destroyEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher,    &node);
    rcl_publisher_fini(&imu_publisher,     &node);
    rcl_publisher_fini(&mag_publisher,     &node);
#ifdef USE_IR_SENSORS
    rcl_publisher_fini(&ir_publisher,      &node);
#endif
#ifdef USE_BUTTON
    rcl_publisher_fini(&button_publisher,  &node);
#endif
#ifdef USE_POTENTIOMETER
    rcl_publisher_fini(&pot_publisher,     &node);
#endif

    rcl_subscription_fini(&led_subscriber, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
#ifdef USE_IR_SENSORS
    rcl_subscription_fini(&ir_config_subscriber, &node);
#endif
#ifdef USE_SERVO
    rcl_subscription_fini(&servo_subscriber, &node);
#endif
#ifdef USE_BUZZER
    rcl_subscription_fini(&buzzer_subscriber, &node);
#endif

    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    if (!led_user_ctrl) digitalWrite(LED_PIN, HIGH);  // ไม่ override ถ้า user สั่งแล้ว
    return true;
}

// ════════════════════════════════════════════════════════════════
//  fullStop
// ════════════════════════════════════════════════════════════════
void fullStop()
{
    twist_msg.linear.x  = 0.0;
    twist_msg.linear.y  = 0.0;
    twist_msg.angular.z = 0.0;
    motor1_controller.brake();
    motor2_controller.brake();
}

// ════════════════════════════════════════════════════════════════
//  moveBase
// ════════════════════════════════════════════════════════════════
void moveBase()
{
    if ((millis() - prev_cmd_time) >= 200) {
        twist_msg.linear.x  = 0.0;
        twist_msg.linear.y  = 0.0;
        twist_msg.angular.z = 0.0;
        if (!led_user_ctrl) digitalWrite(LED_PIN, HIGH);  // ไม่ override ถ้า user สั่งแล้ว
    }

    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z
    );

    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();

    if (twist_msg.linear.x == 0.0 && twist_msg.angular.z == 0.0) {
        fullStop();
        motor1_pid.reset();
        motor2_pid.reset();
    } else {
        motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
        motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    }

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1, current_rpm2, 0.0f, 0.0f);

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0f;
    prev_odom_update = now;
    odometry.update(vel_dt,
        current_vel.linear_x, current_vel.linear_y, current_vel.angular_z);
}

// ════════════════════════════════════════════════════════════════
//  publishData — odom, imu, mag
// ════════════════════════════════════════════════════════════════
void publishData()
{
    odom_msg = odometry.getData();
    imu_msg  = imu.getData();
    mag_msg  = mag.getData();

#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg.magnetic_field.x -= mag_bias[0];
    mag_msg.magnetic_field.y -= mag_bias[1];
    mag_msg.magnetic_field.z -= mag_bias[2];
#endif

    double roll, pitch, yaw;
    roll  = atan2(imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
    pitch = atan2(-imu_msg.linear_acceleration.y,
                  sqrt(imu_msg.linear_acceleration.y * imu_msg.linear_acceleration.y +
                       imu_msg.linear_acceleration.z * imu_msg.linear_acceleration.z));
    yaw   = atan2(mag_msg.magnetic_field.y, mag_msg.magnetic_field.x);

    double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5),  sr = sin(roll * 0.5);
    imu_msg.orientation.x = cy * cp * sr - sy * sp * cr;
    imu_msg.orientation.y = sy * cp * sr + cy * sp * cr;
    imu_msg.orientation.z = sy * cp * cr - cy * sp * sr;
    imu_msg.orientation.w = cy * cp * cr + sy * sp * sr;

    struct timespec ts = getTime();
    odom_msg.header.stamp.sec  = ts.tv_sec;
    odom_msg.header.stamp.nanosec = ts.tv_nsec;
    imu_msg.header.stamp.sec   = ts.tv_sec;
    imu_msg.header.stamp.nanosec  = ts.tv_nsec;
    mag_msg.header.stamp.sec   = ts.tv_sec;
    mag_msg.header.stamp.nanosec  = ts.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher,  &imu_msg,  NULL));
    RCSOFTCHECK(rcl_publish(&mag_publisher,  &mag_msg,  NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

// ════════════════════════════════════════════════════════════════
//  Time sync
// ════════════════════════════════════════════════════════════════
bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true;
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        int64_t time_ns = rmw_uros_epoch_nanos();
        timespec tp;
        tp.tv_sec  = time_ns / 1000000000;
        tp.tv_nsec = time_ns % 1000000000;
        clock_settime(CLOCK_REALTIME, &tp);
#else
        time_offset = rmw_uros_epoch_millis() - millis();
#endif
        return true;
    }
    return false;
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    unsigned long long now = millis() + time_offset;
    tp.tv_sec  = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

// ════════════════════════════════════════════════════════════════
//  Utilities
// ════════════════════════════════════════════════════════════════
void rclErrorLoop()
{
    while (true) { flashLED(2); runOta(); }
}

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++) {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);  delay(150);
    }
    delay(1000);
}

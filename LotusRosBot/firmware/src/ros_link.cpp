// ros_link.cpp — ดูคำอธิบายใน ros_link.h
// ส่วน state machine / createEntities / syncTime พัฒนาต่อจาก linorobot2_hardware (Apache-2.0, Juan Miguel Jimeno)
#include "ros_link.h"
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>

#include "config.h"
#include "syslog.h"
#include "ota.h"
#include "base.h"
#include "heading.h"
#include "sensors.h"
#include "actuators.h"
#include "display.h"
#include "netinfo.h"

#ifdef USE_WIFI_TRANSPORT
#include <WiFi.h>
#endif

#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif
#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

static void rclErrorLoop();

// ── publishers / subscribers ─────────────────────────────────
static rcl_publisher_t odom_publisher;
static rcl_publisher_t imu_publisher;
static rcl_publisher_t mag_publisher;
static rcl_publisher_t ir_publisher;         // /ir/raw
static rcl_publisher_t button_publisher;     // /button/state
static rcl_publisher_t pot_publisher;        // /pot/raw

static rcl_subscription_t twist_subscriber;      // /cmd_vel
static rcl_subscription_t led_subscriber;        // /led/state → GPIO23
static rcl_subscription_t ir_config_subscriber;  // /ir/config  (threshold V)
static rcl_subscription_t servo_subscriber;      // /servo/cmd  [s1_deg, s2_deg, s3_deg]
static rcl_subscription_t buzzer_subscriber;     // /buzzer/play [freq_hz, duration_ms]

// ── ข้อความ ──────────────────────────────────────────────────
static nav_msgs__msg__Odometry          odom_msg;
static sensor_msgs__msg__Imu            imu_msg;
static sensor_msgs__msg__MagneticField  mag_msg;
static std_msgs__msg__Float32MultiArray ir_msg;     static float ir_msg_data[4];    // [L_V, R_V, L_raw, R_raw]
static std_msgs__msg__Float32MultiArray pot_msg;    static float pot_msg_data[2];   // [V, raw]
static std_msgs__msg__Bool              button_msg;
static std_msgs__msg__Bool              led_msg;
static std_msgs__msg__Float32           ir_config_msg;
static std_msgs__msg__Float32MultiArray servo_cmd_msg;   static float servo_cmd_data[3];
static std_msgs__msg__Float32MultiArray buzzer_cmd_msg;  static float buzzer_cmd_data[2];

static rclc_executor_t executor;
static rclc_support_t  support;
static rcl_allocator_t allocator;
static rcl_node_t      node;
static rcl_timer_t     control_timer;

static unsigned long long time_offset = 0;

enum RosState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static RosState state = WAITING_AGENT;

bool rosConnected() { return state == AGENT_CONNECTED; }

static bool createEntities();
static bool destroyEntities();
static bool syncTime();
static struct timespec getTime();
static void publishData();

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

// ════════════════════════════════════════════════════════════════
//  เริ่มต้น
// ════════════════════════════════════════════════════════════════
void rosLinkInitMessages()
{
#ifdef USE_IR_SENSORS
    ir_msg.data.data = ir_msg_data;   ir_msg.data.size = 4;   ir_msg.data.capacity = 4;
#endif
#ifdef USE_POTENTIOMETER
    pot_msg.data.data = pot_msg_data; pot_msg.data.size = 2;  pot_msg.data.capacity = 2;
#endif
    servo_cmd_msg.data.data  = servo_cmd_data;  servo_cmd_msg.data.size  = 3; servo_cmd_msg.data.capacity  = 3;
    buzzer_cmd_msg.data.data = buzzer_cmd_data; buzzer_cmd_msg.data.size = 2; buzzer_cmd_msg.data.capacity = 2;
}

void rosLinkBegin()
{
    state = WAITING_AGENT;
#ifdef USE_WIFI_TRANSPORT
    set_microros_net_transports(AGENT_IP, AGENT_PORT);
#else
    while (Serial.available() > 0) Serial.read();   // ล้างข้อความ NET ที่ค้าง ก่อนให้ micro-ROS ใช้ serial
    set_microros_serial_transports(Serial);
#endif
}

// ════════════════════════════════════════════════════════════════
//  state machine — เรียกทุกรอบ loop()
// ════════════════════════════════════════════════════════════════
void rosLinkSpin()
{
    switch (state)
    {
        case WAITING_AGENT:
            netPollSerial(false);   // รับ IP จาก Jetson ระหว่างรอ agent
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
                baseBeginEncoders();   // หลังสร้าง entity เสร็จ (ไม่งั้น micro-ROS init ทับ PCNT — 2026-07-04)
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
}

// ════════════════════════════════════════════════════════════════
//  Callbacks
// ════════════════════════════════════════════════════════════════
static void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        moveBase();        // PID คุมล้อทุกรอบ 50 Hz
        updateGyroYaw();   // รวม gyro เป็นมุมทุกรอบ 50 Hz
        // ส่ง odom + imu ทุกรอบที่ 10 (~5 Hz) — ส่ง 50 Hz จะเกินสาย 115200 (~36 KB/s > 11.5 KB/s)
        static uint8_t pub_div = 0;
        if (++pub_div >= 10) { pub_div = 0; publishData(); }
    }
}

static void twistCallback(const void * msgin)
{
    (void) msgin;                  // ข้อความถูกเขียนลง twist_msg (base.cpp) โดย executor แล้ว
    prev_cmd_time = millis();
}

static void ledCallback(const void * msgin)
{
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    ledSetByUser(msg->data);
}

static void irConfigCallback(const void * msgin)
{
#ifdef USE_IR_SENSORS
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    ir_threshold = msg->data;
    syslog(LOG_INFO, "ir_threshold=%.2f", ir_threshold);
#endif
}

static void servoCallback(const void * msgin)
{
#ifdef USE_SERVO
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size < 3) return;
    servoWriteAll(constrain((int)msg->data.data[0], 0, 180),
                  constrain((int)msg->data.data[1], 0, 180),
                  constrain((int)msg->data.data[2], 0, 180));
#endif
}

static void buzzerCallback(const void * msgin)
{
#ifdef USE_BUZZER
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size < 2) return;
    buzzerPlay(msg->data.data[0], msg->data.data[1]);
#endif
}

// ════════════════════════════════════════════════════════════════
//  ส่งข้อมูล
// ════════════════════════════════════════════════════════════════
static void publishData()
{
    odom_msg = baseGetOdom();
    imuBuildMessages(&imu_msg, &mag_msg);

    struct timespec ts = getTime();
    odom_msg.header.stamp.sec     = ts.tv_sec;
    odom_msg.header.stamp.nanosec = ts.tv_nsec;
    imu_msg.header.stamp.sec      = ts.tv_sec;
    imu_msg.header.stamp.nanosec  = ts.tv_nsec;
    mag_msg.header.stamp.sec      = ts.tv_sec;
    mag_msg.header.stamp.nanosec  = ts.tv_nsec;

    // ส่งแค่ imu + odom · /imu/mag ไม่ส่ง (สายตัน และ MPU6050 ไม่มีเข็มทิศอยู่แล้ว)
    RCSOFTCHECK(rcl_publish(&imu_publisher,  &imu_msg,  NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void rosPublishIr(float left_v, float right_v, int left_raw, int right_raw)
{
#ifdef USE_IR_SENSORS
    if (state != AGENT_CONNECTED) return;
    ir_msg.data.data[0] = left_v;
    ir_msg.data.data[1] = right_v;
    ir_msg.data.data[2] = (float)left_raw;
    ir_msg.data.data[3] = (float)right_raw;
    RCSOFTCHECK(rcl_publish(&ir_publisher, &ir_msg, NULL));
#endif
}

void rosPublishPot(float v, int raw)
{
#ifdef USE_POTENTIOMETER
    if (state != AGENT_CONNECTED) return;
    pot_msg.data.data[0] = v;
    pot_msg.data.data[1] = (float)raw;
    RCSOFTCHECK(rcl_publish(&pot_publisher, &pot_msg, NULL));
#endif
}

void rosPublishButton(bool pressed)
{
#ifdef USE_BUTTON
    if (state != AGENT_CONNECTED) return;
    button_msg.data = pressed;
    RCSOFTCHECK(rcl_publish(&button_publisher, &button_msg, NULL));
#endif
}

// ════════════════════════════════════════════════════════════════
//  สร้าง / ลบ entity — executor: 1 timer + 5 subscriber = 6 handle
// ════════════════════════════════════════════════════════════════
static bool createEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

    // ── publishers ───────────────────────────────────────────
    RCCHECK(rclc_publisher_init_default(&odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), TOPIC_PREFIX "odom/unfiltered"));
    RCCHECK(rclc_publisher_init_default(&imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), TOPIC_PREFIX "imu/data"));
    RCCHECK(rclc_publisher_init_default(&mag_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), TOPIC_PREFIX "imu/mag"));
#ifdef USE_IR_SENSORS
    RCCHECK(rclc_publisher_init_default(&ir_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), TOPIC_PREFIX "ir/raw"));
#endif
#ifdef USE_BUTTON
    RCCHECK(rclc_publisher_init_default(&button_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_PREFIX "button/state"));
#endif
#ifdef USE_POTENTIOMETER
    RCCHECK(rclc_publisher_init_default(&pot_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), TOPIC_PREFIX "pot/raw"));
#endif

    // ── subscribers: best effort ทั้งหมด ─────────────────────
    //   reliable stream ของ /cmd_vel เคยเต็ม window แล้วหยุดรับที่ ~7 วินาที (มอเตอร์หยุด)
    //   best effort ไม่มี ack/window จึงไม่ค้าง · ยังรับจาก publisher แบบ reliable ได้ (2026-07-04)
    RCCHECK(rclc_subscription_init_best_effort(&twist_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), TOPIC_PREFIX "cmd_vel"));
#ifdef USE_IR_SENSORS
    RCCHECK(rclc_subscription_init_best_effort(&ir_config_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), TOPIC_PREFIX "ir/config"));
#endif
#ifdef USE_SERVO
    RCCHECK(rclc_subscription_init_best_effort(&servo_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), TOPIC_PREFIX "servo/cmd"));
#endif
#ifdef USE_BUZZER
    RCCHECK(rclc_subscription_init_best_effort(&buzzer_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), TOPIC_PREFIX "buzzer/play"));
#endif
    RCCHECK(rclc_subscription_init_best_effort(&led_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_PREFIX "led/state"));

    // ── timer 50 Hz ──────────────────────────────────────────
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(&control_timer, &support,
        RCL_MS_TO_NS(control_timeout), controlCallback));

    // ── executor ─────────────────────────────────────────────
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twistCallback, ON_NEW_DATA));
#ifdef USE_IR_SENSORS
    RCCHECK(rclc_executor_add_subscription(&executor, &ir_config_subscriber, &ir_config_msg, &irConfigCallback, ON_NEW_DATA));
#endif
#ifdef USE_SERVO
    RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_cmd_msg, &servoCallback, ON_NEW_DATA));
#endif
#ifdef USE_BUZZER
    RCCHECK(rclc_executor_add_subscription(&executor, &buzzer_subscriber, &buzzer_cmd_msg, &buzzerCallback, ON_NEW_DATA));
#endif
    RCCHECK(rclc_executor_add_subscription(&executor, &led_subscriber, &led_msg, &ledCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();
    ledDefaultOn();
    return true;
}

static bool destroyEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher,  &node));
    RCSOFTCHECK(rcl_publisher_fini(&mag_publisher,  &node));
#ifdef USE_IR_SENSORS
    RCSOFTCHECK(rcl_publisher_fini(&ir_publisher, &node));
#endif
#ifdef USE_BUTTON
    RCSOFTCHECK(rcl_publisher_fini(&button_publisher, &node));
#endif
#ifdef USE_POTENTIOMETER
    RCSOFTCHECK(rcl_publisher_fini(&pot_publisher, &node));
#endif

    RCSOFTCHECK(rcl_subscription_fini(&led_subscriber,   &node));
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
#ifdef USE_IR_SENSORS
    RCSOFTCHECK(rcl_subscription_fini(&ir_config_subscriber, &node));
#endif
#ifdef USE_SERVO
    RCSOFTCHECK(rcl_subscription_fini(&servo_subscriber, &node));
#endif
#ifdef USE_BUZZER
    RCSOFTCHECK(rcl_subscription_fini(&buzzer_subscriber, &node));
#endif

    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    ledDefaultOn();
    return true;
}

// ════════════════════════════════════════════════════════════════
//  เวลา — ซิงก์นาฬิกากับ agent เพื่อใส่ timestamp ในข้อความ
// ════════════════════════════════════════════════════════════════
static bool syncTime()
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

static struct timespec getTime()
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

static void rclErrorLoop()
{
    while (true) { flashLED(2); runOta(); }
}

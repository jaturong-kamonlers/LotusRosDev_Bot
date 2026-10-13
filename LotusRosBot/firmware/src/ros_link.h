// ════════════════════════════════════════════════════════════════
//  ros_link — micro-ROS: ต่อ agent บน Jetson, สร้าง topic, รับคำสั่ง, ส่งข้อมูล
//
//  node: NODE_NAME (lotusrosbot_base)
//  ส่ง  : /odom/unfiltered  /imu/data   (~5 Hz จาก timer 50 Hz ทุกรอบที่ 10)
//         /ir/raw  /pot/raw (ทุก 200 ms)  /button/state (เมื่อเปลี่ยน)  /imu/mag (สร้างไว้ ไม่ส่ง)
//  รับ  : /cmd_vel  /led/state  /ir/config  /servo/cmd  /buzzer/play   (best effort ทั้งหมด)
//
//  ⚠️ สาย USB 115200 รับได้ ~11.5 KB/s — ส่งถี่กว่านี้ /cmd_vel ขาเข้าจะโดนแย่งจนมอเตอร์หยุด (2026-07-04)
// ════════════════════════════════════════════════════════════════
#ifndef LRB_ROS_LINK_H
#define LRB_ROS_LINK_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

// ทำ X ทุก MS มิลลิวินาที (ไม่บล็อก loop)
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

void rosLinkInitMessages();       // ผูกหน่วยความจำของข้อความ array (เรียกครั้งเดียวใน setup)
void rosLinkBegin();              // เลือก transport (USB serial) แล้วเริ่มรอ agent
void rosLinkSpin();               // เรียกทุกรอบ loop: state machine รอ agent → ต่อ → ทำงาน → หลุด
bool rosConnected();

// ส่งค่าเซนเซอร์ (ส่งเฉพาะตอนต่อ agent อยู่)
void rosPublishIr(float left_v, float right_v, int left_raw, int right_raw);
void rosPublishPot(float v, int raw);
void rosPublishButton(bool pressed);

#endif

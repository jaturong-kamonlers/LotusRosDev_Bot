// ════════════════════════════════════════════════════════════════
//  base — ฐานหุ่น: มอเตอร์ 2 ล้อ + encoder + PID + odometry ของล้อ
//  moveBase() ถูกเรียกทุก 20 ms (50 Hz) จาก ros_link (controlCallback)
// ════════════════════════════════════════════════════════════════
#ifndef LRB_BASE_H
#define LRB_BASE_H

#include <Arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

extern geometry_msgs__msg__Twist twist_msg;   // คำสั่งล่าสุดจาก /cmd_vel (ros_link เขียนลงที่นี่)
extern unsigned long prev_cmd_time;           // millis() ตอนได้ /cmd_vel ล่าสุด · เงียบเกิน 200 ms = หยุดล้อ

void baseBeginEncoders();        // เริ่มนับ encoder (ต้องเรียกหลัง micro-ROS สร้าง entity เสร็จ)
void moveBase();                 // PID คุมล้อตามคำสั่ง + อัปเดต odometry
void fullStop();                 // ล้างคำสั่ง + เบรกทั้งสองล้อ
nav_msgs__msg__Odometry baseGetOdom();
void baseLastRpm(float &rpm1, float &rpm2);   // rpm ล้อล่าสุด (ใช้ตรวจว่าจอดนิ่ง)

// ── ใช้ในโหมดคาลิเบรต (m = 0 ล้อซ้าย, 1 ล้อขวา) ─────────────
void    baseMotorSpin(int m, int pwm);
void    baseMotorBrake(int m);
int32_t baseEncRead(int m);
void    baseEncWrite(int m, int32_t ticks);
float   baseMaxRPM();
float   baseLinearAt(float rpm);  // ความเร็วเดินหน้า (m/s) เมื่อทั้งสองล้อหมุน rpm นี้

#endif

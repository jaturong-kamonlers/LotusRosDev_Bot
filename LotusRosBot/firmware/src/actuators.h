// ════════════════════════════════════════════════════════════════
//  actuators — ไฟ LED, buzzer, servo
// ════════════════════════════════════════════════════════════════
#ifndef LRB_ACTUATORS_H
#define LRB_ACTUATORS_H

#include <Arduino.h>
#include "config.h"

// ── LED (GPIO23) ─────────────────────────────────────────────
extern bool led_user_ctrl;          // true = ผู้ใช้สั่งผ่าน /led/state แล้ว (ระบบจะไม่ไปแตะ)
void ledInit();
void ledDefaultOn();                // ติดค้าง = ROS2 พร้อม (ถ้าผู้ใช้ยังไม่สั่งเอง)
void ledSetByUser(bool on);         // จาก /led/state
void flashLED(int n_times);         // กะพริบ n ครั้ง แล้วพัก 1 วินาที (ใช้แจ้ง error)

// ── Buzzer (GPIO18, LEDC ch7) ────────────────────────────────
void buzzerInit();
void buzzerTone(uint32_t freq);     // 0 = เงียบ
void buzzerBeep(uint32_t freq, uint32_t ms);   // ดังแล้วรอจนจบ (ใช้ตอนบูต/คาลิเบรต)
void buzzerPlay(float freq, float duration_ms);   // จาก /buzzer/play — ไม่รอ
void buzzerCheck();                 // เรียกทุก 10 ms: หยุดเสียงเมื่อครบเวลา

// ── Servo (GPIO32/33/5) ──────────────────────────────────────
void servoInitDetached();           // attach แล้ว detach → servo คลายตัว ไม่ล็อกองศาตอนบูต
void servoAttachAll();
void servoDetachAll();
void servoWriteAll(int a1, int a2, int a3);   // องศา 0–180 (attach ให้เองถ้ายังไม่ได้ attach)

#endif

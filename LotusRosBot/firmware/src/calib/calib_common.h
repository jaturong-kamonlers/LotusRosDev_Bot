// ════════════════════════════════════════════════════════════════
//  calib_common — ของที่ทุกเมนูคาลิเบรตใช้ร่วมกัน (เสียง, ปุ่ม, POT, จอ)
//
//  โหมดคาลิเบรต: กดปุ่ม (GPIO27) ค้างภายใน 5 วินาทีหลังเปิดบอร์ด
//  ไม่มี micro-ROS และไม่ใช้ Serial — ทุกอย่างแสดงบน OLED
//    POT  GPIO35 — เลื่อนเมนู / คุม servo / เลื่อนผล
//    BTN  GPIO27 — ยืนยัน / หยุด (เมนู 5: กดค้าง 3 วินาที = ออก)
// ════════════════════════════════════════════════════════════════
#ifndef LRB_CALIB_COMMON_H
#define LRB_CALIB_COMMON_H

#include <Arduino.h>
#include "config.h"
#include "../display.h"

#define CALIB_SAMPLE_TIME    20      // วินาที ต่อการวัด CPR 1 ครั้ง
#define CALIB_CPR5_ROUNDS     5      // จำนวนรอบของเมนู CPR x5
#define CALIB_WARN_STDDEV    20.0f   // เตือนถ้าค่าแกว่งเกินนี้
#define CALIB_MENU_ITEMS      9
#define CALIB_HAND_REVS      10      // หมุนล้อด้วยมือกี่รอบ (เมนู 7)
#define CALIB_OLED_ROWS       4
#define CALIB_OLED_ROW_H     12
#define CALIB_OLED_CONTENT_Y 14

// ── เสียง ────────────────────────────────────────────────────
void calib_beepShort();
void calib_beepConfirm();
void calib_beepDone();
void calib_beepWarn();
void calib_beepEnterCalib();          // โด เร มี — เข้าโหมดคาลิเบรต

// ── ปุ่ม ─────────────────────────────────────────────────────
bool calib_btnPressed();              // true ครั้งเดียวต่อการกด (กันเด้ง 120 ms + บี๊บสั้น)
void calib_btnResetEdge();
void calib_waitRelease();             // รอปล่อยปุ่ม แล้วล้างสถานะ (ก่อนเริ่มงานแต่ละเมนู)
bool calib_btnDown();                 // อ่านปุ่มตรง ๆ

// ── POT ──────────────────────────────────────────────────────
int   calib_potRaw();
int   calib_potMap(int range);        // 0 … range-1
float calib_potDeg();                 // 0 … 180

// ── จอ ───────────────────────────────────────────────────────
void calib_oledHeader(const char *title);
void calib_oledLine(int row, const char *txt);
void calib_oledLineR(int row, const char *txt);
void calib_oledBar(int row, float pct);
bool calib_confirmScreen(const char *runName);   // BTN = เริ่ม · หมุน POT สุด = กลับ

// ── เมนู (แต่ละไฟล์) ─────────────────────────────────────────
void calib_runSpin();          // 1  calib_motor.cpp
void calib_runSampleCPR();     // 2  calib_motor.cpp
void calib_runSampleCPR5();    // 3  calib_motor.cpp
void calib_runServoCheck();    // 4  calib_io.cpp
void calib_runSensorCheck();   // 5  calib_io.cpp
void calib_runLEDTest();       // 6  calib_io.cpp
void calib_runCountTicks();    // 7  calib_motor.cpp
void calib_runNetInfo();       // 8  calib_network.cpp
void calib_runSshInfo();       // 9  calib_network.cpp

void runCalibration();         // calib_menu.cpp — เรียกจาก loop() ในโหมดคาลิเบรต

#endif

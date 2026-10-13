// ════════════════════════════════════════════════════════════════
//  display — จอ OLED 128×64 (SSD1306, I2C 0x3C)
// ════════════════════════════════════════════════════════════════
#ifndef LRB_DISPLAY_H
#define LRB_DISPLAY_H

#include "SSD1306Wire.h"

extern SSD1306Wire display;

void displayInit();
void displayBootScreen();                 // ชื่อ + เวอร์ชัน + "Init IMU..."
void displayImuFail();
void displayImuOk();
void displayCountdown(int seconds_left);  // เลือกโหมดตอนบูต
void displayModeBanner(const char *mode, const char *msg);   // "CALIBRATION MODE" / "ROS2 MODE"
void updateDisplay();                     // สถานะ ROS2 (เรียกตอนสถานะเปลี่ยน)
void updateDisplaySensors();              // ค่าเซนเซอร์ + แถบ (เรียกทุก 200 ms)

#endif

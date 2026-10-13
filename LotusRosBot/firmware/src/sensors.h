// ════════════════════════════════════════════════════════════════
//  sensors — IR 2 ช่อง, potentiometer, ปุ่ม (อ่านทุก 200 ms แล้วส่งไป ROS)
// ════════════════════════════════════════════════════════════════
#ifndef LRB_SENSORS_H
#define LRB_SENSORS_H

#include "config.h"

#ifdef USE_IR_SENSORS
extern float ir_left_v, ir_right_v;
extern int   ir_left_raw, ir_right_raw;
extern float ir_threshold;        // V — ปรับได้ผ่าน /ir/config
#endif
#ifdef USE_POTENTIOMETER
extern float pot_v;
extern int   pot_raw;
#endif
#ifdef USE_BUTTON
extern bool button_pressed;
#endif

void sensorsInit();
void readSensors();               // อ่าน + ส่ง /ir/raw /pot/raw (+ /button/state เมื่อเปลี่ยน)
bool buttonIsDown();              // อ่านปุ่มตรง ๆ (active LOW)

#endif

// sensors.cpp — ดูคำอธิบายใน sensors.h
#include <Arduino.h>
#include "sensors.h"
#include "ros_link.h"

#ifdef USE_IR_SENSORS
float ir_left_v    = 0.0f;
float ir_right_v   = 0.0f;
int   ir_left_raw  = 0;
int   ir_right_raw = 0;
float ir_threshold = 2.0f;   // ค่าเริ่มต้น — ปรับได้ผ่าน /ir/config
#endif

#ifdef USE_POTENTIOMETER
float pot_v   = 0.0f;
int   pot_raw = 0;
#endif

#ifdef USE_BUTTON
bool button_pressed = false;
static bool button_prev = false;
#endif

void sensorsInit() {
#ifdef USE_BUTTON
    pinMode(BUTTON_PIN, INPUT_PULLUP);
#endif
}

bool buttonIsDown() {
    return digitalRead(BUTTON_PIN) == LOW;
}

void readSensors()
{
#ifdef USE_IR_SENSORS
    // อ่านทิ้ง 1 ครั้งให้ ADC นิ่งหลังสลับจาก POT (ESP32 ใช้ multiplexer ตัวเดียว ค่าแรกหลังสลับมักผิด)
    analogRead(IR_LEFT_PIN);
    delayMicroseconds(50);

    // เฉลี่ย 8 ครั้ง ลด noise
    int sum_l = 0, sum_r = 0;
    for (int i = 0; i < 8; i++) {
        sum_l += analogRead(IR_LEFT_PIN);
        sum_r += analogRead(IR_RIGHT_PIN);
    }
    ir_left_raw  = sum_l / 8;
    ir_right_raw = sum_r / 8;
    ir_left_v    = ir_left_raw  * (3.3f / 4095.0f);
    ir_right_v   = ir_right_raw * (3.3f / 4095.0f);
    rosPublishIr(ir_left_v, ir_right_v, ir_left_raw, ir_right_raw);
#endif

#ifdef USE_POTENTIOMETER
    int sum_p = 0;
    for (int i = 0; i < 4; i++) { sum_p += analogRead(POT_PIN); }
    pot_raw = sum_p / 4;
    pot_v   = pot_raw * (3.3f / 4095.0f);
    rosPublishPot(pot_v, pot_raw);
#endif

#ifdef USE_BUTTON
    bool current = buttonIsDown();
    if (current != button_prev) {
        button_pressed = current;
        button_prev    = current;
        rosPublishButton(button_pressed);
    }
#endif
}

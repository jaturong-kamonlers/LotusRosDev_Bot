// calib_pid.h — ทดสอบ step ของ PID (ใช้ทั้งเมนู 10 บน OLED และหน้าเว็บผ่าน calib_remote)
#ifndef LRB_CALIB_PID_H
#define LRB_CALIB_PID_H

#include <Arduino.h>

// รับผลทุก 20 ms: เวลา (ms), เป้า (rpm), ล้อซ้าย rpm/pwm, ล้อขวา rpm/pwm
typedef void (*StepSink)(uint32_t t_ms, float tgt, float rpm1, int pwm1, float rpm2, int pwm2);

// side: 'L' ล้อซ้าย · 'R' ล้อขวา · 'B' ทั้งสองล้อ · คืน false ถ้าถูกหยุดกลางทาง (BTN / abortCheck)
bool calib_pidStep(char side, float rpm, uint32_t hold_ms, StepSink sink, bool (*abortCheck)());

// rise = เวลา 0→90% (ms) · overshoot = พุ่งเกินเป้า (%) · ss_err = ขาดจากเป้าช่วงท้าย (%)
void calib_stepMetrics(const float *t_ms, const float *tgt, const float *rpm, int n, float target,
                       float &rise_ms, float &overshoot_pct, float &ss_err_pct);

void calib_runPidTune();         // เมนู 10

#endif

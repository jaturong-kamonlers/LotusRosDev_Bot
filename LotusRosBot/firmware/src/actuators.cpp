// actuators.cpp — ดูคำอธิบายใน actuators.h
#include "actuators.h"

#ifdef USE_SERVO
#include <ESP32Servo.h>
static Servo servo1, servo2, servo3;
#endif

// ── LED ──────────────────────────────────────────────────────
bool led_user_ctrl = false;

void ledInit() { pinMode(LED_PIN, OUTPUT); }

void ledDefaultOn() {
    if (!led_user_ctrl) digitalWrite(LED_PIN, HIGH);   // ไม่ override ถ้าผู้ใช้สั่งแล้ว
}

void ledSetByUser(bool on) {
    led_user_ctrl = true;
    digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void flashLED(int n_times) {
    for (int i = 0; i < n_times; i++) {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);  delay(150);
    }
    delay(1000);
}

// ── Buzzer ───────────────────────────────────────────────────
#ifdef USE_BUZZER
static unsigned long buzzer_stop_ms = 0;   // millis() ที่ต้องหยุดเสียง (0 = ไม่มีเสียง)
#endif

void buzzerInit() {
#ifdef USE_BUZZER
    ledcSetup(BUZZER_CHANNEL, 2000, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWriteTone(BUZZER_CHANNEL, 0);
#endif
}

void buzzerTone(uint32_t freq) {
#ifdef USE_BUZZER
    ledcWriteTone(BUZZER_CHANNEL, freq);
#endif
}

void buzzerBeep(uint32_t freq, uint32_t ms) {
    buzzerTone(freq); delay(ms);
    buzzerTone(0);
}

void buzzerPlay(float freq_hz, float duration_ms) {
#ifdef USE_BUZZER
    uint32_t freq = (uint32_t)constrain(freq_hz, 20.0f, 20000.0f);
    uint32_t dur  = (uint32_t)constrain(duration_ms, 1.0f, 10000.0f);
    ledcWriteTone(BUZZER_CHANNEL, freq);
    buzzer_stop_ms = millis() + dur;
#endif
}

void buzzerCheck() {
#ifdef USE_BUZZER
    if (buzzer_stop_ms > 0 && millis() >= buzzer_stop_ms) {
        ledcWriteTone(BUZZER_CHANNEL, 0);
        buzzer_stop_ms = 0;
    }
#endif
}

// ── Servo ────────────────────────────────────────────────────
void servoInitDetached() {
#ifdef USE_SERVO
    servo1.attach(SERVO1_PIN); servo1.detach();
    servo2.attach(SERVO2_PIN); servo2.detach();
    servo3.attach(SERVO3_PIN); servo3.detach();
#endif
}

void servoAttachAll() {
#ifdef USE_SERVO
    if (!servo1.attached()) servo1.attach(SERVO1_PIN);
    if (!servo2.attached()) servo2.attach(SERVO2_PIN);
    if (!servo3.attached()) servo3.attach(SERVO3_PIN);
#endif
}

void servoDetachAll() {
#ifdef USE_SERVO
    servo1.detach(); servo2.detach(); servo3.detach();
#endif
}

void servoWriteAll(int a1, int a2, int a3) {
#ifdef USE_SERVO
    servoAttachAll();                     // detach ไว้ตอนบูต → attach ก่อนเขียนทุกครั้ง
    servo1.write(a1);
    servo2.write(a2);
    servo3.write(a3);
#endif
}

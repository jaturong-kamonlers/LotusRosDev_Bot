// LotusRosDev_Bot — Calibration Firmware v2
// ESP32 DevKit V1 30-pin | No ROS2 | Standalone
//
// Controls:
//   POT  GPIO35 — เลื่อน menu / ควบคุม servo
//   BTN  GPIO27 — ยืนยัน / หยุด / กลับ menu (beep ทุกครั้งที่กด)
//   OLED SDA21/SCL22
//   BUZZER GPIO18 LEDC ch7
//
// Menu:
//   1. Spin motors    — FWD/LEFT/RIGHT/STOP แสดงสถานะบน OLED
//   2. Sample CPR     — M1 แล้ว M2 อัตโนมัติ ไม่ต้องกด BTN ระหว่างกัน
//   3. Sample CPR x5  — 5 รอบ สลับซ้าย-ขวา แสดง CPR แต่ละรอบ + avg สุดท้าย
//   4. Servo check    — POT ควบคุม angle 0-180 องศา servo ทั้ง 3 พร้อมกัน
//   5. Sensor check   — แสดง sensor ทั้งหมด BTN ไม่ออก menu
//   6. LED test       — LED กระพริบ

#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "pid.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "kinematics.h"
#include <Wire.h>
#include "SSD1306Wire.h"
#ifdef USE_SERVO
#include <ESP32Servo.h>
#endif

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif
#ifndef BUZZER_CHANNEL
#define BUZZER_CHANNEL 7
#endif

struct SimpleTwist {
    struct { float x, y, z; } linear;
    struct { float x, y, z; } angular;
};

// ── Hardware ─────────────────────────────────────────────────
SSD1306Wire display(0x3c, SDA_PIN, SCL_PIN);

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Motor   motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor   motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
PID     motor1_pid(PWM_MIN, PWM_MAX, K_P_L, K_I_L, K_D_L);
PID     motor2_pid(PWM_MIN, PWM_MAX, K_P_R, K_I_R, K_D_R);
Kinematics kinematics(
    Kinematics::LINO_BASE, MOTOR_MAX_RPM,
    MAX_RPM_RATIO_L, MAX_RPM_RATIO_R,
    MOTOR_OPERATING_VOLTAGE, MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER, LR_WHEELS_DISTANCE);

Motor   *motors[2]   = {&motor1_controller, &motor2_controller};
Encoder *encoders[2] = {&motor1_encoder,    &motor2_encoder};
PID     *pids[2]     = {&motor1_pid,        &motor2_pid};

#ifdef USE_SERVO
Servo servo1, servo2, servo3;
#endif

// ── Constants ─────────────────────────────────────────────────
#define SAMPLE_TIME     20
#define CPR5_ROUNDS      5
#define CPR_WARN_STDDEV 20.0f
#define PWM_TICKS_TEST  200
#define MENU_ITEMS        6
#define OLED_ROWS         4
#define OLED_ROW_H       12
#define OLED_CONTENT_Y   14

// ── Buzzer ────────────────────────────────────────────────────
void beepShort() {
    ledcWriteTone(BUZZER_CHANNEL, 1500); delay(60);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}
void beepConfirm() {
    ledcWriteTone(BUZZER_CHANNEL, 1500); delay(60);
    ledcWriteTone(BUZZER_CHANNEL, 0);    delay(60);
    ledcWriteTone(BUZZER_CHANNEL, 1800); delay(80);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}
void beepDone() {
    ledcWriteTone(BUZZER_CHANNEL, 2000); delay(300);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}
void beepWarn() {
    ledcWriteTone(BUZZER_CHANNEL, 600); delay(400);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}
void beepStartup() {
    // โด เร มี
    ledcWriteTone(BUZZER_CHANNEL, 523); delay(150);
    ledcWriteTone(BUZZER_CHANNEL, 0);   delay(50);
    ledcWriteTone(BUZZER_CHANNEL, 587); delay(150);
    ledcWriteTone(BUZZER_CHANNEL, 0);   delay(50);
    ledcWriteTone(BUZZER_CHANNEL, 659); delay(200);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}

// ── Button ────────────────────────────────────────────────────
bool btnPressed() {
    static bool prev = HIGH;
    static unsigned long lastMs = 0;
    bool cur = (digitalRead(BUTTON_PIN) == LOW);
    if (cur && !prev && (millis() - lastMs > 120)) {
        prev = cur; lastMs = millis();
        beepShort();
        return true;
    }
    prev = cur;
    return false;
}
bool btnHeld() { return (digitalRead(BUTTON_PIN) == LOW); }

// reset static prev ใน btnPressed เพื่อล้าง edge ที่ค้าง
void btnResetEdge() {
    // อ่านสถานะจริงแล้ว warm-up btnPressed 2 cycles
    for (int i = 0; i < 3; i++) {
        bool cur = (digitalRead(BUTTON_PIN) == LOW);
        (void)cur;
        delay(5);
    }
    // เรียก btnPressed ให้ sync prev กับสถานะปัจจุบัน
    btnPressed(); btnPressed();
}

// ── POT ───────────────────────────────────────────────────────
int potRaw() {
    int s = 0;
    for (int i = 0; i < 4; i++) s += analogRead(POT_PIN);
    return s / 4;
}
int potMap(int range) {
    return map(constrain(potRaw(), 0, 4090), 0, 4090, 0, range - 1);
}
float potDeg() {
    // ใช้ arithmetic float โดยตรง ไม่ใช้ map() ที่ return long
    int raw = constrain(potRaw(), 0, 4090);
    return (raw / 4090.0f) * 180.0f;
}

// ── OLED ──────────────────────────────────────────────────────
void oledHeader(const char *title) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    // "LotusRosDev" บรรทัดเล็ก ซ้าย
    display.drawString(0, 0, "LotusRosDev");
    // title ขวา
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(127, 0, String(title));
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawLine(0, 11, 127, 11);
}
void oledLine(int row, const char *txt) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, OLED_CONTENT_Y + row * OLED_ROW_H, String(txt));
}
void oledLineR(int row, const char *txt) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(127, OLED_CONTENT_Y + row * OLED_ROW_H, String(txt));
    display.setTextAlignment(TEXT_ALIGN_LEFT);
}
void oledBar(int row, float pct) {
    int y = OLED_CONTENT_Y + row * OLED_ROW_H + 1;
    display.drawRect(0, y, 110, 8);
    display.fillRect(0, y, (int)(constrain(pct, 0, 1) * 110), 8);
}

// ── Confirm screen ────────────────────────────────────────────
bool confirmScreen(const char *runName) {
    while (true) {
        int pv = potRaw();
        display.clear();
        oledHeader("CONFIRM?");
        oledLine(0, runName);
        oledLine(1, "BTN = start");
        oledLine(2, "Rotate POT = back");
        display.display();
        if (btnPressed()) { beepConfirm(); return true; }
        if (pv < 150 || pv > 3900) return false;
        delay(30);
    }
}

// ── MENU — แถบสีขาวเลื่อน ─────────────────────────────────────
const char *menuItems[MENU_ITEMS] = {
    "Spin motors",
    "Sample CPR",
    "Sample CPR x5",
    "Servo check",
    "Sensor check",
    "LED test"
};

int showMenu() {
    while (true) {
        int sel = potMap(MENU_ITEMS);
        display.clear();
        // header
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 0, "LotusRosDev Menu");
        display.drawLine(0, 11, 127, 11);
        display.setTextAlignment(TEXT_ALIGN_LEFT);

        int start = constrain(sel - 1, 0, MENU_ITEMS - OLED_ROWS);
        for (int i = 0; i < OLED_ROWS; i++) {
            int idx = start + i;
            if (idx >= MENU_ITEMS) break;
            int y = OLED_CONTENT_Y + i * OLED_ROW_H;
            if (idx == sel) {
                // แถบขาว = selected
                display.fillRect(0, y - 1, 128, OLED_ROW_H);
                display.setColor(BLACK);
                char buf[22];
                snprintf(buf, sizeof(buf), " %d.%s", idx + 1, menuItems[idx]);
                display.drawString(0, y, String(buf));
                display.setColor(WHITE);
            } else {
                char buf[22];
                snprintf(buf, sizeof(buf), " %d.%s", idx + 1, menuItems[idx]);
                display.drawString(0, y, String(buf));
            }
        }
        display.display();
        if (btnPressed()) return sel;
        delay(40);
    }
}

// ══════════════════════════════════════════════════════════════
//  MENU 1 — Spin motors
//  FWD 2s → LEFT 2s → RIGHT 2s → STOP แสดงสถานะบน OLED
// ══════════════════════════════════════════════════════════════
void spinPhase(const char *label, int pwm1, int pwm2, unsigned long ms) {
    unsigned long t0 = millis();
    unsigned long lastDisp = 0;
    while (millis() - t0 < ms) {
        // ── spin ก่อนเสมอ อย่าให้ display ขัด ──
        motor1_controller.spin(pwm1);
        motor2_controller.spin(pwm2);
        // ── update OLED ทุก 150ms เท่านั้น ──
        if (millis() - lastDisp >= 150) {
            lastDisp = millis();
            float pct = (millis() - t0) / (float)ms;
            display.clear();
            oledHeader("SPIN MOTORS");
            oledLine(0, label);
            oledBar(1, pct);
            char tb[22];
            snprintf(tb, sizeof(tb), "M1:%4d  M2:%4d", pwm1, pwm2);
            oledLine(2, tb);
            char rem[16];
            snprintf(rem, sizeof(rem), "%.1fs", (ms-(millis()-t0))/1000.0f);
            oledLineR(2, rem);
            oledLine(3, "BTN=stop");
            display.display();
        }
        if (btnPressed()) break;
        delay(10);
    }
    motor1_controller.brake();
    motor2_controller.brake();
}

void runSpin() {
    if (!confirmScreen("Spin test 4 phase")) return;
    // รอปล่อยนิ้ว + reset static edge ใน btnPressed()
    while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    delay(150);
    btnResetEdge();
    Serial.println("\n=== SPIN MOTORS ===");

    // FWD — ทั้ง 2 มอเตอร์ไปข้างหน้า
    Serial.println("FWD 2s");
    spinPhase("FWD (forward)", PWM_MAX, PWM_MAX, 2000);
    delay(500);

    // TURN LEFT — M1 หยุด M2 หมุน
    Serial.println("TURN LEFT 2s");
    spinPhase("LEFT (turn left)", 0, PWM_MAX, 2000);
    delay(500);

    // TURN RIGHT — M1 หมุน M2 หยุด
    Serial.println("TURN RIGHT 2s");
    spinPhase("RIGHT (turn right)", PWM_MAX, 0, 2000);
    delay(500);

    // STOP
    motor1_controller.brake();
    motor2_controller.brake();
    Serial.println("STOP");

    display.clear();
    oledHeader("SPIN DONE");
    oledLine(0, "FWD: OK");
    oledLine(1, "LEFT: OK");
    oledLine(2, "RIGHT: OK");
    oledLine(3, "BTN=menu");
    display.display();
    beepDone();
    while (!btnPressed()) delay(30);
}

// ══════════════════════════════════════════════════════════════
//  MENU 2 — Sample CPR
//  M1 เสร็จ → รอ 2 วิ → M2 อัตโนมัติ ไม่ต้องกด BTN
// ══════════════════════════════════════════════════════════════
void runSampleCPR() {
    if (!confirmScreen("Sample CPR 20s")) return;
    while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    delay(150);
    btnResetEdge();
    float measured_v = constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE);
    float scaled_rpm = (measured_v / MOTOR_OPERATING_VOLTAGE) * MOTOR_MAX_RPM;
    float total_rev  = scaled_rpm * (SAMPLE_TIME / 60.0f);
    long long cpr[2] = {0, 0};

    Serial.println("\n=== SAMPLE CPR ===");

    for (int m = 0; m < 2; m++) {
        if (m == 1) {
            // รอ 2 วิ แสดง countdown ก่อน M2
            Serial.println("Waiting 2s before M2...");
            for (int c = 2; c > 0; c--) {
                display.clear();
                oledHeader("SAMPLE CPR");
                char wb[22];
                snprintf(wb, sizeof(wb), "M2 starts in %ds", c);
                oledLine(1, wb);
                oledLine(2, "M1 CPR done");
                char c1[16]; snprintf(c1, sizeof(c1), "M1:%lld", cpr[0]);
                oledLine(3, c1);
                display.display();
                delay(1000);
            }
        }

        encoders[m]->write(0);
        unsigned long t0 = millis();
        Serial.printf("Spinning M%d...\n", m + 1);

        while (millis() - t0 < SAMPLE_TIME * 1000UL) {
            float elapsed = (millis() - t0) / 1000.0f;
            long ticks = encoders[m]->read();
            display.clear();
            oledHeader("SAMPLE CPR");
            char h[22]; snprintf(h, sizeof(h), "M%d spinning", m + 1);
            oledLine(0, h);
            oledBar(1, elapsed / SAMPLE_TIME);
            char tb[22]; snprintf(tb, sizeof(tb), "t:%.0f/%ds tks:%ld", elapsed, SAMPLE_TIME, ticks);
            oledLine(2, tb);
            oledLine(3, "BTN=stop");
            display.display();
            motors[m]->spin(PWM_MAX);
            delay(50);
            if (btnPressed()) break;
        }
        motors[m]->spin(0); motors[m]->brake();
        cpr[m] = (long long)(encoders[m]->read() / total_rev);
        Serial.printf("M%d CPR = %lld\n", m + 1, cpr[m]);
        beepShort();
    }

    float max_rpm = kinematics.getMaxRPM();
    Kinematics::velocities vl = kinematics.getVelocities(max_rpm, max_rpm, 0, 0);
    Kinematics::velocities va = kinematics.getVelocities(-max_rpm, max_rpm, 0, 0);
    Serial.printf("Lin:%.2fm/s  Ang:%.2frad/s\n", vl.linear_x, va.angular_z);

    display.clear();
    oledHeader("CPR RESULT");
    char b0[22]; snprintf(b0, sizeof(b0), "M1 CPR: %lld", cpr[0]);
    oledLine(0, b0);
    char b1[22]; snprintf(b1, sizeof(b1), "M2 CPR: %lld", cpr[1]);
    oledLine(1, b1);
    char b2[22]; snprintf(b2, sizeof(b2), "Lin:%.2fm/s", vl.linear_x);
    oledLine(2, b2);
    oledLine(3, "BTN=menu");
    display.display();
    beepDone();
    while (!btnPressed()) delay(30);
}

// ══════════════════════════════════════════════════════════════
//  MENU 3 — Sample CPR x5
//  สลับซ้าย-ขวา ทุกรอบ อัตโนมัติหลังกด confirm
//  แสดง CPR แต่ละรอบ + avg บรรทัดสุดท้าย
// ══════════════════════════════════════════════════════════════
void runSampleCPR5() {
    if (!confirmScreen("CPR x5 auto")) return;
    while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    delay(150);
    btnResetEdge();

    float measured_v = constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE);
    float scaled_rpm = (measured_v / MOTOR_OPERATING_VOLTAGE) * MOTOR_MAX_RPM;
    float total_rev  = scaled_rpm * (SAMPLE_TIME / 60.0f);

    float cprLog[2][CPR5_ROUNDS];
    int   done[2] = {0, 0};
    bool  stopped = false;

    Serial.println("\n=== CPR x5 AUTO ===");

    for (int r = 0; r < CPR5_ROUNDS && !stopped; r++) {
        // สลับ: รอบคี่ M1 ก่อน รอบคู่ M2 ก่อน
        int order[2] = {r % 2 == 0 ? 0 : 1, r % 2 == 0 ? 1 : 0};

        for (int oi = 0; oi < 2 && !stopped; oi++) {
            int m = order[oi];
            encoders[m]->write(0);
            unsigned long t0 = millis();
            Serial.printf("Round %d M%d spinning...\n", r + 1, m + 1);

            while (millis() - t0 < SAMPLE_TIME * 1000UL) {
                float elapsed = (millis() - t0) / 1000.0f;
                long ticks = encoders[m]->read();
                display.clear();
                oledHeader("CPR x5");
                char h[22];
                snprintf(h, sizeof(h), "r%d/5 M%d auto", r + 1, m + 1);
                oledLine(0, h);
                oledBar(1, elapsed / SAMPLE_TIME);
                char tb[22];
                snprintf(tb, sizeof(tb), "t:%.0f/%ds", elapsed, SAMPLE_TIME);
                oledLine(2, tb);
                char tk[22];
                snprintf(tk, sizeof(tk), "ticks:%ld", ticks);
                oledLine(3, tk);
                display.display();
                motors[m]->spin(PWM_MAX);
                delay(50);
                if (btnPressed()) { stopped = true; break; }
            }
            motors[m]->spin(0); motors[m]->brake();

            float thisCPR = (float)(encoders[m]->read()) / total_rev;
            cprLog[m][done[m]] = thisCPR;
            done[m]++;
            Serial.printf("  r%d M%d CPR=%.0f\n", r + 1, m + 1, thisCPR);
            beepShort();
            delay(500); // พักระหว่าง motor
        }
    }

    // ── คำนวณ avg + stddev ────────────────────────────────────
    float avg[2] = {0, 0}, stdv[2] = {0, 0};
    for (int m = 0; m < 2; m++) {
        int n = done[m];
        if (n == 0) continue;
        float sum = 0;
        for (int r = 0; r < n; r++) sum += cprLog[m][r];
        avg[m] = sum / n;
        float sq = 0;
        for (int r = 0; r < n; r++) sq += (cprLog[m][r] - avg[m]) * (cprLog[m][r] - avg[m]);
        stdv[m] = (n > 1) ? sqrtf(sq / (n - 1)) : 0;
        Serial.printf("M%d AVG=%.0f STDDEV=%.1f\n", m + 1, avg[m], stdv[m]);
        if (stdv[m] > CPR_WARN_STDDEV) {
            Serial.printf("!! M%d HIGH VARIANCE !!\n", m + 1);
            beepWarn();
        }
    }

    // ── Result screen scroll ──────────────────────────────────
    // สร้าง content: แต่ละรอบ แล้ว avg สุดท้าย
    const int CMAX = 20;
    char content[CMAX][22];
    int  nc = 0;

    // warn lines
    for (int m = 0; m < 2; m++) {
        if (stdv[m] > CPR_WARN_STDDEV) {
            char w[22];
            snprintf(w, sizeof(w), "!!M%d HI-VAR %.0f", m + 1, stdv[m]);
            strncpy(content[nc++], w, 22);
        }
    }
    // CPR แต่ละรอบ — แสดง M1 และ M2 คู่กัน
    int maxR = max(done[0], done[1]);
    for (int r = 0; r < maxR && nc < CMAX - 2; r++) {
        char ll[22];
        float v1 = (r < done[0]) ? cprLog[0][r] : 0;
        float v2 = (r < done[1]) ? cprLog[1][r] : 0;
        snprintf(ll, sizeof(ll), "r%d M1:%.0f M2:%.0f", r + 1, v1, v2);
        strncpy(content[nc++], ll, 22);
    }
    // avg บรรทัดสุดท้าย
    char avgL[22];
    snprintf(avgL, sizeof(avgL), "AVG M1:%.0f M2:%.0f", avg[0], avg[1]);
    strncpy(content[nc++], avgL, 22);
    strncpy(content[nc++], "BTN=menu", 22);

    int maxScroll = max(0, nc - OLED_ROWS);
    while (true) {
        int scroll = (maxScroll > 0) ? potMap(maxScroll + 1) : 0;
        scroll = constrain(scroll, 0, maxScroll);
        display.clear();
        oledHeader("CPR x5 RESULT");
        for (int i = 0; i < OLED_ROWS; i++) {
            int idx = scroll + i;
            if (idx < nc) oledLine(i, content[idx]);
        }
        display.display();
        if (btnPressed()) break;
        delay(40);
    }
}

// ══════════════════════════════════════════════════════════════
//  MENU 4 — Servo check
//  POT → 0-180 องศา → servo 1,2,3 พร้อมกัน
// ══════════════════════════════════════════════════════════════
void runServoCheck() {
#ifdef USE_SERVO
    Serial.println("\n=== SERVO CHECK (BTN=exit) ===");
    while (true) {
        float deg = potDeg();
        int   ideg = (int)deg;
        servo1.write(ideg);
        servo2.write(ideg);
        servo3.write(ideg);

        display.clear();
        oledHeader("SERVO CHECK");
        char d0[22]; snprintf(d0, sizeof(d0), "Angle: %d deg", ideg);
        oledLine(0, d0);
        // progress bar แสดง angle
        display.drawRect(0, OLED_CONTENT_Y + OLED_ROW_H + 1, 110, 8);
        display.fillRect(0, OLED_CONTENT_Y + OLED_ROW_H + 1, (int)(deg / 180.0f * 110), 8);
        char sv[22]; snprintf(sv, sizeof(sv), "S1:%d S2:%d S3:%d", ideg, ideg, ideg);
        oledLine(2, sv);
        oledLine(3, "BTN=menu");
        display.display();

        Serial.printf("Servo all: %d deg\n", ideg);
        if (btnPressed()) break;
        delay(20);
    }
    servo1.write(90); servo2.write(90); servo3.write(90);
#else
    display.clear(); oledHeader("SERVO CHECK");
    oledLine(1, "USE_SERVO not set");
    oledLine(3, "BTN=menu");
    display.display();
    while (!btnPressed()) delay(30);
#endif
}

// ══════════════════════════════════════════════════════════════
//  MENU 5 — Sensor check
//  แสดง SensorName:GPIO:ค่า — BTN ไม่ออก menu
// ══════════════════════════════════════════════════════════════
void runSensorCheck() {
    Serial.println("\n=== SENSOR CHECK (BTN=stays, hold 3s=exit) ===");
    unsigned long holdStart = 0;

    while (true) {
        // ── อ่านค่า sensor ──────────────────────────────────
        int   potRawV  = potRaw();
        float potV     = potRawV * (3.3f / 4095.0f);
        bool  btnState = (digitalRead(BUTTON_PIN) == LOW);

#ifdef USE_IR_SENSORS
        // dummy read เพื่อ settle ADC channel หลังจาก POT
        analogRead(IR_LEFT_PIN);
        delay(2);
        int irLSum = 0, irRSum = 0;
        for (int _i = 0; _i < 8; _i++) {
            irLSum += analogRead(IR_LEFT_PIN);
            irRSum += analogRead(IR_RIGHT_PIN);
        }
        int   irLRaw = irLSum / 8;
        int   irRRaw = irRSum / 8;
        float irLV   = irLRaw * (3.3f / 4095.0f);
        float irRV   = irRRaw * (3.3f / 4095.0f);
#endif

        // ── สร้าง content lines ──────────────────────────────
        const int SC_MAX = 10;
        char sc[SC_MAX][22];
        int  nsc = 0;

        // POT: GPIO35: V [raw]
        char p0[22];
        snprintf(p0, sizeof(p0), "POT:35:%.2fV[%d]", potV, potRawV);
        strncpy(sc[nsc++], p0, 22);

        // BTN: GPIO27: state
        char p1[22];
        snprintf(p1, sizeof(p1), "BTN:27:%d", btnState ? 1 : 0);
        strncpy(sc[nsc++], p1, 22);

#ifdef USE_IR_SENSORS
        char p2[22];
        snprintf(p2, sizeof(p2), "IRL:39:%.2fV[%d]", irLV, irLRaw);
        strncpy(sc[nsc++], p2, 22);
        char p3[22];
        snprintf(p3, sizeof(p3), "IRR:36:%.2fV[%d]", irRV, irRRaw);
        strncpy(sc[nsc++], p3, 22);
#endif

#ifdef USE_SERVO
        char p4[22];
        snprintf(p4, sizeof(p4), "SRV:32/33/5:%ddeg", servo1.read());
        strncpy(sc[nsc++], p4, 22);
#endif
        char p5[22];
        snprintf(p5, sizeof(p5), "LED:GPIO%d:out", LED_PIN);
        strncpy(sc[nsc++], p5, 22);

        char p6[22];
        snprintf(p6, sizeof(p6), "BUZ:GPIO%d:pwm", BUZZER_PIN);
        strncpy(sc[nsc++], p6, 22);

        strncpy(sc[nsc++], "Hold BTN 3s=exit", 22);

        // ── scroll ──────────────────────────────────────────
        int maxScroll = max(0, nsc - OLED_ROWS);
        int scroll = (maxScroll > 0) ? potMap(maxScroll + 1) : 0;
        scroll = constrain(scroll, 0, maxScroll);

        display.clear();
        oledHeader("SENSOR CHECK");
        for (int i = 0; i < OLED_ROWS; i++) {
            int idx = scroll + i;
            if (idx < nsc) oledLine(i, sc[idx]);
        }
        display.display();

        Serial.printf("POT:%.2fV[%d] BTN:%d", potV, potRawV, btnState ? 1 : 0);
#ifdef USE_IR_SENSORS
        Serial.printf(" IRL:%.2fV IRR:%.2fV", irLV, irRV);
#endif
        Serial.println();

        // hold BTN 3 วินาที = ออก
        if (btnState) {
            if (holdStart == 0) holdStart = millis();
            if (millis() - holdStart >= 3000) {
                beepConfirm();
                break;
            }
        } else {
            holdStart = 0;
        }
        delay(200);
    }
}

// ══════════════════════════════════════════════════════════════
//  MENU 6 — LED test กระพริบ
// ══════════════════════════════════════════════════════════════
void runLEDTest() {
    Serial.println("\n=== LED TEST (BTN=exit) ===");
    int blinkMs = 500;
    bool ledState = false;
    unsigned long lastBlink = 0;

    while (true) {
        if (millis() - lastBlink >= (unsigned long)blinkMs) {
            lastBlink = millis();
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        }
        display.clear();
        oledHeader("LED TEST");
        char s0[22];
        snprintf(s0, sizeof(s0), "LED GPIO%d: %s", LED_PIN, ledState ? "ON " : "OFF");
        oledLine(0, s0);
        char s1[22];
        snprintf(s1, sizeof(s1), "Blink: %dms", blinkMs);
        oledLine(1, s1);
        oledLine(2, "POT=speed");
        oledLine(3, "BTN=menu");
        display.display();

        // POT ปรับความเร็วกระพริบ 100–1000ms
        blinkMs = map(constrain(potRaw(), 0, 4090), 0, 4090, 100, 1000);

        if (btnPressed()) {
            digitalWrite(LED_PIN, LOW);
            break;
        }
        delay(20);
    }
}

// ══════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(BAUDRATE);
    while (!Serial) {}

    Wire.begin(SDA_PIN, SCL_PIN);
    display.init();
    display.setFont(ArialMT_Plain_10);

    pinMode(LED_PIN,    OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);

#ifdef USE_BUZZER
    ledcSetup(BUZZER_CHANNEL, 2000, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWriteTone(BUZZER_CHANNEL, 0);
#endif

#ifdef USE_SERVO
    servo1.attach(SERVO1_PIN); servo1.write(90);
    servo2.attach(SERVO2_PIN); servo2.write(90);
    servo3.attach(SERVO3_PIN); servo3.write(90);
#endif

    // startup screen
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 10, "LotusRosDev Bot");
    display.drawString(64, 28, "Calibration v2");
    display.drawString(64, 46, "Initializing...");
    display.display();

    beepStartup();

    Serial.println("\n=================================");
    Serial.println("  LotusRosDev Bot Calibration v2");
    Serial.println("  Rotate POT to select");
    Serial.println("  Press BTN to confirm");
    Serial.println("=================================\n");

    delay(1500);
}

// ══════════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════════
void loop() {
    int sel = showMenu();
    switch (sel) {
        case 0: runSpin();        break;
        case 1: runSampleCPR();   break;
        case 2: runSampleCPR5();  break;
        case 3: runServoCheck();  break;
        case 4: runSensorCheck(); break;
        case 5: runLEDTest();     break;
        default: break;
    }
}

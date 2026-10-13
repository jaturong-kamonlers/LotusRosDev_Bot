// ════════════════════════════════════════════════════════════════
//  calib_io — เมนู 4 Servo check · 5 Sensor check · 6 LED test
// ════════════════════════════════════════════════════════════════
#include "calib_common.h"
#include "../actuators.h"

// ── เมนู 4 — Servo check: หมุน POT = องศา servo ทั้ง 3 ตัว ─────
void calib_runServoCheck() {
#ifdef USE_SERVO
    servoAttachAll();
    while (true) {
        float deg  = calib_potDeg();
        int   ideg = (int)deg;
        servoWriteAll(ideg, ideg, ideg);
        display.clear();
        calib_oledHeader("SERVO CHECK");
        char d0[22]; snprintf(d0, sizeof(d0), "Angle: %d deg", ideg);
        calib_oledLine(0, d0);
        display.drawRect(0, CALIB_OLED_CONTENT_Y + CALIB_OLED_ROW_H + 1, 110, 8);
        display.fillRect(0, CALIB_OLED_CONTENT_Y + CALIB_OLED_ROW_H + 1, (int)(deg / 180.0f * 110), 8);
        char sv[22]; snprintf(sv, sizeof(sv), "S1:%d S2:%d S3:%d", ideg, ideg, ideg);
        calib_oledLine(2, sv);
        calib_oledLine(3, "BTN=menu");
        display.display();
        if (calib_btnPressed()) break;
        delay(20);
    }
    servoDetachAll();   // ออกจากเมนู → ปล่อย servo คลายตัว
#else
    display.clear(); calib_oledHeader("SERVO CHECK");
    calib_oledLine(1, "USE_SERVO not set");
    calib_oledLine(3, "BTN=menu");
    display.display();
    while (!calib_btnPressed()) delay(30);
#endif
}

// ── เมนู 5 — Sensor check: ค่าทุกขา real-time (กดปุ่มค้าง 3 วินาที = ออก) ──
void calib_runSensorCheck() {
    unsigned long holdStart = 0;
    while (true) {
        int   potRawV  = calib_potRaw();
        float potV     = potRawV * (3.3f / 4095.0f);
        bool  btnState = calib_btnDown();

#ifdef USE_IR_SENSORS
        analogRead(IR_LEFT_PIN);
        delayMicroseconds(50);
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

        const int SC_MAX = 10;
        char sc[SC_MAX][22];
        int  nsc = 0;

        char p0[22]; snprintf(p0, sizeof(p0), "POT:35:%.2fV[%d]", potV, potRawV);
        strncpy(sc[nsc++], p0, 22);
        char p1[22]; snprintf(p1, sizeof(p1), "BTN:27:%d", btnState ? 1 : 0);
        strncpy(sc[nsc++], p1, 22);
#ifdef USE_IR_SENSORS
        char p2[22]; snprintf(p2, sizeof(p2), "IRL:39:%.2fV[%d]", irLV, irLRaw);
        strncpy(sc[nsc++], p2, 22);
        char p3[22]; snprintf(p3, sizeof(p3), "IRR:36:%.2fV[%d]", irRV, irRRaw);
        strncpy(sc[nsc++], p3, 22);
#endif
#ifdef USE_SERVO
        char p4[22]; snprintf(p4, sizeof(p4), "SRV:32/33/5:detach");
        strncpy(sc[nsc++], p4, 22);
#endif
        char p5[22]; snprintf(p5, sizeof(p5), "LED:GPIO%d:out", LED_PIN);
        strncpy(sc[nsc++], p5, 22);
        char p6[22]; snprintf(p6, sizeof(p6), "BUZ:GPIO%d:ch%d", BUZZER_PIN, BUZZER_CHANNEL);
        strncpy(sc[nsc++], p6, 22);
        strncpy(sc[nsc++], "Hold BTN 3s=exit", 22);

        int maxScroll = max(0, nsc - CALIB_OLED_ROWS);
        int scroll = constrain((maxScroll > 0) ? calib_potMap(maxScroll + 1) : 0, 0, maxScroll);
        display.clear();
        calib_oledHeader("SENSOR CHECK");
        for (int i = 0; i < CALIB_OLED_ROWS; i++) {
            if (scroll + i < nsc) calib_oledLine(i, sc[scroll + i]);
        }
        display.display();

        if (btnState) {
            if (holdStart == 0) holdStart = millis();
            if (millis() - holdStart >= 3000) { calib_beepConfirm(); break; }
        } else { holdStart = 0; }
        delay(200);
    }
}

// ── เมนู 6 — LED test: กะพริบ · หมุน POT = ความเร็ว ────────────
void calib_runLEDTest() {
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
        calib_oledHeader("LED TEST");
        char s0[22]; snprintf(s0, sizeof(s0), "LED GPIO%d: %s", LED_PIN, ledState ? "ON " : "OFF");
        calib_oledLine(0, s0);
        char s1[22]; snprintf(s1, sizeof(s1), "Blink: %dms", blinkMs);
        calib_oledLine(1, s1);
        calib_oledLine(2, "POT=speed");
        calib_oledLine(3, "BTN=menu");
        display.display();
        blinkMs = map(constrain(calib_potRaw(), 0, 4090), 0, 4090, 100, 1000);
        if (calib_btnPressed()) {
            digitalWrite(LED_PIN, LOW);
            break;
        }
        delay(20);
    }
}

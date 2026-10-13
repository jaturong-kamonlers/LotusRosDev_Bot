// calib_common.cpp — ดูคำอธิบายใน calib_common.h
#include "calib_common.h"
#include "../version.h"
#include "../actuators.h"

// ── เสียง ────────────────────────────────────────────────────
void calib_beepShort()   { buzzerBeep(1500, 60); }
void calib_beepConfirm() { buzzerBeep(1500, 60); delay(60); buzzerBeep(1800, 80); }
void calib_beepDone()    { buzzerBeep(2000, 300); }
void calib_beepWarn()    { buzzerBeep(600, 400); }
void calib_beepEnterCalib() {
    buzzerBeep(523, 150); delay(50);    // โด
    buzzerBeep(587, 150); delay(50);    // เร
    buzzerBeep(659, 200);               // มี
}

// ── ปุ่ม ─────────────────────────────────────────────────────
bool calib_btnDown() { return digitalRead(BUTTON_PIN) == LOW; }

bool calib_btnPressed() {
    static bool prev = HIGH;
    static unsigned long lastMs = 0;
    bool cur = calib_btnDown();
    if (cur && !prev && (millis() - lastMs > 120)) {
        prev = cur; lastMs = millis();
        calib_beepShort();
        return true;
    }
    prev = cur;
    return false;
}

void calib_btnResetEdge() {
    for (int i = 0; i < 3; i++) {
        (void)digitalRead(BUTTON_PIN);
        delay(5);
    }
    calib_btnPressed();
    calib_btnPressed();
}

void calib_waitRelease() {
    while (calib_btnDown()) delay(10);
    delay(150);
    calib_btnResetEdge();
}

// ── POT ──────────────────────────────────────────────────────
int calib_potRaw() {
    int s = 0;
    for (int i = 0; i < 4; i++) s += analogRead(POT_PIN);
    return s / 4;
}
int calib_potMap(int range) {
    return map(constrain(calib_potRaw(), 0, 4090), 0, 4090, 0, range - 1);
}
float calib_potDeg() {
    int raw = constrain(calib_potRaw(), 0, 4090);
    return (raw / 4090.0f) * 180.0f;
}

// ── จอ ───────────────────────────────────────────────────────
void calib_oledHeader(const char *title) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, LRB_NAME);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(127, 0, String(title));
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawLine(0, 11, 127, 11);
}
void calib_oledLine(int row, const char *txt) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, CALIB_OLED_CONTENT_Y + row * CALIB_OLED_ROW_H, String(txt));
}
void calib_oledLineR(int row, const char *txt) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(127, CALIB_OLED_CONTENT_Y + row * CALIB_OLED_ROW_H, String(txt));
    display.setTextAlignment(TEXT_ALIGN_LEFT);
}
void calib_oledBar(int row, float pct) {
    int y = CALIB_OLED_CONTENT_Y + row * CALIB_OLED_ROW_H + 1;
    display.drawRect(0, y, 110, 8);
    display.fillRect(0, y, (int)(constrain(pct, 0, 1) * 110), 8);
}

bool calib_confirmScreen(const char *runName) {
    while (true) {
        int pv = calib_potRaw();
        display.clear();
        calib_oledHeader("CONFIRM?");
        calib_oledLine(0, runName);
        calib_oledLine(1, "BTN = start");
        calib_oledLine(2, "Rotate POT = back");
        display.display();
        if (calib_btnPressed()) { calib_beepConfirm(); return true; }
        if (pv < 150 || pv > 3900) return false;
        delay(30);
    }
}

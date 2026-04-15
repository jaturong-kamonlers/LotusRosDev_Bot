// ════════════════════════════════════════════════════════════════
//  calib_menu.h — LotusRosDev Calibration Menu
//  บอร์ด: LotusRosDev Shield (ESP32 DevKit V1 30-pin)
//
//  *** include ไฟล์นี้ใน firmware.ino เท่านั้น ***
//  *** ไม่มี Serial ทั้งหมด — แสดงผลผ่าน OLED ***
//
//  Controls:
//    POT  GPIO35 — เลื่อน menu / ควบคุม servo / scroll ผล
//    BTN  GPIO27 — ยืนยัน / หยุด (hold 3s = ออก sensor check)
//    OLED SDA21/SCL22 — แสดงผลทั้งหมด
//    BUZZER GPIO18 ch7 — feedback เสียง
//
//  Menus:
//    1. Spin motors   — FWD/LEFT/RIGHT/STOP ทดสอบทิศทาง
//    2. Sample CPR    — M1→M2 อัตโนมัติ วัด COUNTS_PER_REV
//    3. CPR x5        — 5 รอบ สลับอัตโนมัติ avg+stddev
//    4. Servo check   — POT 0-180° ควบคุม servo ทั้ง 3
//    5. Sensor check  — แสดงค่า sensor ทุกตัว real-time
//    6. LED test      — LED กระพริบ POT ปรับความเร็ว
// ════════════════════════════════════════════════════════════════

#ifndef CALIB_MENU_H
#define CALIB_MENU_H

// ── Constants ────────────────────────────────────────────────
#define CALIB_SAMPLE_TIME    20      // วินาที ต่อ CPR sample
#define CALIB_CPR5_ROUNDS     5      // จำนวนรอบ CPR x5
#define CALIB_WARN_STDDEV    20.0f   // เตือนถ้า stddev เกินนี้
#define CALIB_MENU_ITEMS      6
#define CALIB_OLED_ROWS       4
#define CALIB_OLED_ROW_H     12
#define CALIB_OLED_CONTENT_Y 14

// ════════════════════════════════════════════════════════════════
//  BUZZER helpers — ใช้งานใน calibration mode เท่านั้น
// ════════════════════════════════════════════════════════════════
static void calib_beepShort() {
    ledcWriteTone(BUZZER_CHANNEL, 1500); delay(60);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}
static void calib_beepConfirm() {
    ledcWriteTone(BUZZER_CHANNEL, 1500); delay(60);
    ledcWriteTone(BUZZER_CHANNEL, 0);    delay(60);
    ledcWriteTone(BUZZER_CHANNEL, 1800); delay(80);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}
static void calib_beepDone() {
    ledcWriteTone(BUZZER_CHANNEL, 2000); delay(300);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}
static void calib_beepWarn() {
    ledcWriteTone(BUZZER_CHANNEL, 600); delay(400);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}
static void calib_beepEnterCalib() {
    // โด เร มี — เข้า calibration mode
    ledcWriteTone(BUZZER_CHANNEL, 523); delay(150);
    ledcWriteTone(BUZZER_CHANNEL, 0);   delay(50);
    ledcWriteTone(BUZZER_CHANNEL, 587); delay(150);
    ledcWriteTone(BUZZER_CHANNEL, 0);   delay(50);
    ledcWriteTone(BUZZER_CHANNEL, 659); delay(200);
    ledcWriteTone(BUZZER_CHANNEL, 0);
}

// ════════════════════════════════════════════════════════════════
//  BUTTON helpers
// ════════════════════════════════════════════════════════════════
static bool calib_btnPressed() {
    static bool prev = HIGH;
    static unsigned long lastMs = 0;
    bool cur = (digitalRead(BUTTON_PIN) == LOW);
    if (cur && !prev && (millis() - lastMs > 120)) {
        prev = cur; lastMs = millis();
        calib_beepShort();
        return true;
    }
    prev = cur;
    return false;
}

static void calib_btnResetEdge() {
    for (int i = 0; i < 3; i++) {
        (void)digitalRead(BUTTON_PIN);
        delay(5);
    }
    calib_btnPressed();
    calib_btnPressed();
}

// ════════════════════════════════════════════════════════════════
//  POT helpers
// ════════════════════════════════════════════════════════════════
static int calib_potRaw() {
    int s = 0;
    for (int i = 0; i < 4; i++) s += analogRead(POT_PIN);
    return s / 4;
}
static int calib_potMap(int range) {
    return map(constrain(calib_potRaw(), 0, 4090), 0, 4090, 0, range - 1);
}
static float calib_potDeg() {
    int raw = constrain(calib_potRaw(), 0, 4090);
    return (raw / 4090.0f) * 180.0f;
}

// ════════════════════════════════════════════════════════════════
//  OLED helpers
// ════════════════════════════════════════════════════════════════
static void calib_oledHeader(const char *title) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "LotusRosDev");
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(127, 0, String(title));
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawLine(0, 11, 127, 11);
}
static void calib_oledLine(int row, const char *txt) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, CALIB_OLED_CONTENT_Y + row * CALIB_OLED_ROW_H, String(txt));
}
static void calib_oledLineR(int row, const char *txt) {
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(127, CALIB_OLED_CONTENT_Y + row * CALIB_OLED_ROW_H, String(txt));
    display.setTextAlignment(TEXT_ALIGN_LEFT);
}
static void calib_oledBar(int row, float pct) {
    int y = CALIB_OLED_CONTENT_Y + row * CALIB_OLED_ROW_H + 1;
    display.drawRect(0, y, 110, 8);
    display.fillRect(0, y, (int)(constrain(pct, 0, 1) * 110), 8);
}

// ════════════════════════════════════════════════════════════════
//  CONFIRM screen
// ════════════════════════════════════════════════════════════════
static bool calib_confirmScreen(const char *runName) {
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

// ════════════════════════════════════════════════════════════════
//  MENU — แถบสีขาวเลื่อน
// ════════════════════════════════════════════════════════════════
static const char *calib_menuItems[CALIB_MENU_ITEMS] = {
    "Spin motors",
    "Sample CPR",
    "Sample CPR x5",
    "Servo check",
    "Sensor check",
    "LED test"
};

static int calib_showMenu() {
    while (true) {
        int sel = calib_potMap(CALIB_MENU_ITEMS);
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 0, "LotusRosDev CALIB");
        display.drawLine(0, 11, 127, 11);
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        int start = constrain(sel - 1, 0, CALIB_MENU_ITEMS - CALIB_OLED_ROWS);
        for (int i = 0; i < CALIB_OLED_ROWS; i++) {
            int idx = start + i;
            if (idx >= CALIB_MENU_ITEMS) break;
            int y = CALIB_OLED_CONTENT_Y + i * CALIB_OLED_ROW_H;
            if (idx == sel) {
                display.fillRect(0, y - 1, 128, CALIB_OLED_ROW_H);
                display.setColor(BLACK);
                char buf[22];
                snprintf(buf, sizeof(buf), " %d.%s", idx + 1, calib_menuItems[idx]);
                display.drawString(0, y, String(buf));
                display.setColor(WHITE);
            } else {
                char buf[22];
                snprintf(buf, sizeof(buf), " %d.%s", idx + 1, calib_menuItems[idx]);
                display.drawString(0, y, String(buf));
            }
        }
        display.display();
        if (calib_btnPressed()) return sel;
        delay(40);
    }
}

// ════════════════════════════════════════════════════════════════
//  MENU 1 — Spin motors
// ════════════════════════════════════════════════════════════════
static void calib_spinPhase(const char *label, int pwm1, int pwm2, unsigned long ms) {
    unsigned long t0 = millis();
    unsigned long lastDisp = 0;
    while (millis() - t0 < ms) {
        motor1_controller.spin(pwm1);
        motor2_controller.spin(pwm2);
        if (millis() - lastDisp >= 150) {
            lastDisp = millis();
            float pct = (millis() - t0) / (float)ms;
            display.clear();
            calib_oledHeader("SPIN MOTORS");
            calib_oledLine(0, label);
            calib_oledBar(1, pct);
            char tb[22];
            snprintf(tb, sizeof(tb), "M1:%4d  M2:%4d", pwm1, pwm2);
            calib_oledLine(2, tb);
            char rem[16];
            snprintf(rem, sizeof(rem), "%.1fs", (ms - (millis() - t0)) / 1000.0f);
            calib_oledLineR(2, rem);
            calib_oledLine(3, "BTN=stop");
            display.display();
        }
        if (calib_btnPressed()) break;
        delay(10);
    }
    motor1_controller.brake();
    motor2_controller.brake();
}

static void calib_runSpin() {
    if (!calib_confirmScreen("Spin test 4 phase")) return;
    while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    delay(150);
    calib_btnResetEdge();

    calib_spinPhase("FWD (forward)",    PWM_MAX, PWM_MAX, 2000); delay(500);
    calib_spinPhase("LEFT (turn left)",       0, PWM_MAX, 2000); delay(500);
    calib_spinPhase("RIGHT (turn right)", PWM_MAX,     0, 2000); delay(500);

    motor1_controller.brake();
    motor2_controller.brake();

    display.clear();
    calib_oledHeader("SPIN DONE");
    calib_oledLine(0, "FWD:   OK");
    calib_oledLine(1, "LEFT:  OK");
    calib_oledLine(2, "RIGHT: OK");
    calib_oledLine(3, "BTN=menu");
    display.display();
    calib_beepDone();
    while (!calib_btnPressed()) delay(30);
}

// ════════════════════════════════════════════════════════════════
//  MENU 2 — Sample CPR
// ════════════════════════════════════════════════════════════════
static void calib_runSampleCPR() {
    if (!calib_confirmScreen("Sample CPR 20s")) return;
    while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    delay(150);
    calib_btnResetEdge();

    float measured_v = constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE);
    float scaled_rpm = (measured_v / MOTOR_OPERATING_VOLTAGE) * MOTOR_MAX_RPM;
    float total_rev  = scaled_rpm * (CALIB_SAMPLE_TIME / 60.0f);
    long long cpr[2] = {0, 0};

    for (int m = 0; m < 2; m++) {
        if (m == 1) {
            for (int c = 2; c > 0; c--) {
                display.clear();
                calib_oledHeader("SAMPLE CPR");
                char wb[22];
                snprintf(wb, sizeof(wb), "M2 starts in %ds", c);
                calib_oledLine(1, wb);
                calib_oledLine(2, "M1 done");
                char c1[16]; snprintf(c1, sizeof(c1), "M1:%lld", cpr[0]);
                calib_oledLine(3, c1);
                display.display();
                delay(1000);
            }
        }
        encoders[m]->write(0);
        unsigned long t0 = millis();
        while (millis() - t0 < CALIB_SAMPLE_TIME * 1000UL) {
            float elapsed = (millis() - t0) / 1000.0f;
            long ticks = encoders[m]->read();
            display.clear();
            calib_oledHeader("SAMPLE CPR");
            char h[22]; snprintf(h, sizeof(h), "M%d spinning", m + 1);
            calib_oledLine(0, h);
            calib_oledBar(1, elapsed / CALIB_SAMPLE_TIME);
            char tb[22]; snprintf(tb, sizeof(tb), "t:%.0f/%ds tks:%ld", elapsed, CALIB_SAMPLE_TIME, ticks);
            calib_oledLine(2, tb);
            calib_oledLine(3, "BTN=stop");
            display.display();
            motors[m]->spin(PWM_MAX);
            delay(50);
            if (calib_btnPressed()) break;
        }
        motors[m]->spin(0); motors[m]->brake();
        cpr[m] = (long long)(encoders[m]->read() / total_rev);
        calib_beepShort();
    }

    float max_rpm = kinematics.getMaxRPM();
    Kinematics::velocities vl = kinematics.getVelocities(max_rpm, max_rpm, 0, 0);

    display.clear();
    calib_oledHeader("CPR RESULT");
    char b0[22]; snprintf(b0, sizeof(b0), "M1 CPR: %lld", cpr[0]);
    calib_oledLine(0, b0);
    char b1[22]; snprintf(b1, sizeof(b1), "M2 CPR: %lld", cpr[1]);
    calib_oledLine(1, b1);
    char b2[22]; snprintf(b2, sizeof(b2), "Lin:%.2fm/s", vl.linear_x);
    calib_oledLine(2, b2);
    calib_oledLine(3, "BTN=menu");
    display.display();
    calib_beepDone();
    while (!calib_btnPressed()) delay(30);
}

// ════════════════════════════════════════════════════════════════
//  MENU 3 — Sample CPR x5
// ════════════════════════════════════════════════════════════════
static void calib_runSampleCPR5() {
    if (!calib_confirmScreen("CPR x5 auto")) return;
    while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    delay(150);
    calib_btnResetEdge();

    float measured_v = constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE);
    float scaled_rpm = (measured_v / MOTOR_OPERATING_VOLTAGE) * MOTOR_MAX_RPM;
    float total_rev  = scaled_rpm * (CALIB_SAMPLE_TIME / 60.0f);
    float cprLog[2][CALIB_CPR5_ROUNDS];
    int   done[2] = {0, 0};
    bool  stopped = false;

    for (int r = 0; r < CALIB_CPR5_ROUNDS && !stopped; r++) {
        int order[2] = {r % 2 == 0 ? 0 : 1, r % 2 == 0 ? 1 : 0};
        for (int oi = 0; oi < 2 && !stopped; oi++) {
            int m = order[oi];
            encoders[m]->write(0);
            unsigned long t0 = millis();
            while (millis() - t0 < CALIB_SAMPLE_TIME * 1000UL) {
                float elapsed = (millis() - t0) / 1000.0f;
                long ticks = encoders[m]->read();
                display.clear();
                calib_oledHeader("CPR x5");
                char h[22]; snprintf(h, sizeof(h), "r%d/5 M%d auto", r + 1, m + 1);
                calib_oledLine(0, h);
                calib_oledBar(1, elapsed / CALIB_SAMPLE_TIME);
                char tb[22]; snprintf(tb, sizeof(tb), "t:%.0f/%ds", elapsed, CALIB_SAMPLE_TIME);
                calib_oledLine(2, tb);
                char tk[22]; snprintf(tk, sizeof(tk), "ticks:%ld", ticks);
                calib_oledLine(3, tk);
                display.display();
                motors[m]->spin(PWM_MAX);
                delay(50);
                if (calib_btnPressed()) { stopped = true; break; }
            }
            motors[m]->spin(0); motors[m]->brake();
            cprLog[m][done[m]] = (float)(encoders[m]->read()) / total_rev;
            done[m]++;
            calib_beepShort();
            delay(500);
        }
    }

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
        if (stdv[m] > CALIB_WARN_STDDEV) calib_beepWarn();
    }

    const int CMAX = 20;
    char content[CMAX][22];
    int  nc = 0;
    for (int m = 0; m < 2; m++) {
        if (stdv[m] > CALIB_WARN_STDDEV) {
            char w[22]; snprintf(w, sizeof(w), "!!M%d HI-VAR %.0f", m + 1, stdv[m]);
            strncpy(content[nc++], w, 22);
        }
    }
    int maxR = max(done[0], done[1]);
    for (int r = 0; r < maxR && nc < CMAX - 2; r++) {
        char ll[22];
        float v1 = (r < done[0]) ? cprLog[0][r] : 0;
        float v2 = (r < done[1]) ? cprLog[1][r] : 0;
        snprintf(ll, sizeof(ll), "r%d M1:%.0f M2:%.0f", r + 1, v1, v2);
        strncpy(content[nc++], ll, 22);
    }
    char avgL[22]; snprintf(avgL, sizeof(avgL), "AVG M1:%.0f M2:%.0f", avg[0], avg[1]);
    strncpy(content[nc++], avgL, 22);
    strncpy(content[nc++], "BTN=menu", 22);

    int maxScroll = max(0, nc - CALIB_OLED_ROWS);
    while (true) {
        int scroll = constrain((maxScroll > 0) ? calib_potMap(maxScroll + 1) : 0, 0, maxScroll);
        display.clear();
        calib_oledHeader("CPR x5 RESULT");
        for (int i = 0; i < CALIB_OLED_ROWS; i++) {
            if (scroll + i < nc) calib_oledLine(i, content[scroll + i]);
        }
        display.display();
        if (calib_btnPressed()) break;
        delay(40);
    }
}

// ════════════════════════════════════════════════════════════════
//  MENU 4 — Servo check
// ════════════════════════════════════════════════════════════════
static void calib_runServoCheck() {
#ifdef USE_SERVO
    // attach ก่อนใช้งาน calibration servo check
    if (!servo1.attached()) servo1.attach(SERVO1_PIN);
    if (!servo2.attached()) servo2.attach(SERVO2_PIN);
    if (!servo3.attached()) servo3.attach(SERVO3_PIN);

    while (true) {
        float deg  = calib_potDeg();
        int   ideg = (int)deg;
        servo1.write(ideg);
        servo2.write(ideg);
        servo3.write(ideg);
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
    // ออกจาก servo check → detach กลับ
    servo1.detach(); servo2.detach(); servo3.detach();
#else
    display.clear(); calib_oledHeader("SERVO CHECK");
    calib_oledLine(1, "USE_SERVO not set");
    calib_oledLine(3, "BTN=menu");
    display.display();
    while (!calib_btnPressed()) delay(30);
#endif
}

// ════════════════════════════════════════════════════════════════
//  MENU 5 — Sensor check
// ════════════════════════════════════════════════════════════════
static void calib_runSensorCheck() {
    unsigned long holdStart = 0;
    while (true) {
        int   potRawV = calib_potRaw();
        float potV    = potRawV * (3.3f / 4095.0f);
        bool  btnState = (digitalRead(BUTTON_PIN) == LOW);

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

// ════════════════════════════════════════════════════════════════
//  MENU 6 — LED test
// ════════════════════════════════════════════════════════════════
static void calib_runLEDTest() {
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

// ════════════════════════════════════════════════════════════════
//  MAIN CALIBRATION LOOP — เรียกจาก loop() เมื่อ calib_mode=true
// ════════════════════════════════════════════════════════════════
void runCalibration() {
    int sel = calib_showMenu();
    switch (sel) {
        case 0: calib_runSpin();        break;
        case 1: calib_runSampleCPR();   break;
        case 2: calib_runSampleCPR5();  break;
        case 3: calib_runServoCheck();  break;
        case 4: calib_runSensorCheck(); break;
        case 5: calib_runLEDTest();     break;
        default: break;
    }
}

#endif // CALIB_MENU_H

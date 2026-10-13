// ════════════════════════════════════════════════════════════════
//  calib_motor — เมนู 1 Spin motors · 2 Sample CPR · 3 CPR x5 · 7 Count ticks
//  ⚠️ เมนู 1–3 มอเตอร์หมุนเต็มกำลัง — ยกหุ่นให้ล้อลอยก่อน · เมนู 7 มอเตอร์ไม่หมุน
// ════════════════════════════════════════════════════════════════
#include "calib_common.h"
#include "../base.h"

// ── เมนู 1 — Spin motors: ทดสอบทิศ FWD / LEFT / RIGHT ─────────
static void calib_spinPhase(const char *label, int pwm1, int pwm2, unsigned long ms) {
    unsigned long t0 = millis();
    unsigned long lastDisp = 0;
    while (millis() - t0 < ms) {
        baseMotorSpin(0, pwm1);
        baseMotorSpin(1, pwm2);
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
    baseMotorBrake(0);
    baseMotorBrake(1);
}

void calib_runSpin() {
    if (!calib_confirmScreen("Spin test 4 phase")) return;
    calib_waitRelease();

    calib_spinPhase("FWD (forward)",    PWM_MAX, PWM_MAX, 2000); delay(500);
    calib_spinPhase("LEFT (turn left)",       0, PWM_MAX, 2000); delay(500);
    calib_spinPhase("RIGHT (turn right)", PWM_MAX,     0, 2000); delay(500);

    baseMotorBrake(0);
    baseMotorBrake(1);

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

// รอบล้อที่คาดว่าหมุนได้ใน CALIB_SAMPLE_TIME วินาที (จาก RPM สเปก × แรงดันที่วัดได้)
static float calib_expectedRevs() {
    float measured_v = constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE);
    float scaled_rpm = (measured_v / MOTOR_OPERATING_VOLTAGE) * MOTOR_MAX_RPM;
    return scaled_rpm * (CALIB_SAMPLE_TIME / 60.0f);
}

// ── เมนู 2 — Sample CPR: M1 แล้ว M2 อย่างละ 20 วินาที (ค่าประมาณ) ──
void calib_runSampleCPR() {
    if (!calib_confirmScreen("Sample CPR 20s")) return;
    calib_waitRelease();

    float total_rev = calib_expectedRevs();
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
        baseEncWrite(m, 0);
        unsigned long t0 = millis();
        while (millis() - t0 < CALIB_SAMPLE_TIME * 1000UL) {
            float elapsed = (millis() - t0) / 1000.0f;
            long ticks = baseEncRead(m);
            display.clear();
            calib_oledHeader("SAMPLE CPR");
            char h[22]; snprintf(h, sizeof(h), "M%d spinning", m + 1);
            calib_oledLine(0, h);
            calib_oledBar(1, elapsed / CALIB_SAMPLE_TIME);
            char tb[22]; snprintf(tb, sizeof(tb), "t:%.0f/%ds tks:%ld", elapsed, CALIB_SAMPLE_TIME, ticks);
            calib_oledLine(2, tb);
            calib_oledLine(3, "BTN=stop");
            display.display();
            baseMotorSpin(m, PWM_MAX);
            delay(50);
            if (calib_btnPressed()) break;
        }
        baseMotorSpin(m, 0); baseMotorBrake(m);
        cpr[m] = (long long)(baseEncRead(m) / total_rev);
        calib_beepShort();
    }

    float lin = baseLinearAt(baseMaxRPM());

    display.clear();
    calib_oledHeader("CPR RESULT");
    char b0[22]; snprintf(b0, sizeof(b0), "M1 CPR: %lld", cpr[0]);
    calib_oledLine(0, b0);
    char b1[22]; snprintf(b1, sizeof(b1), "M2 CPR: %lld", cpr[1]);
    calib_oledLine(1, b1);
    char b2[22]; snprintf(b2, sizeof(b2), "Lin:%.2fm/s", lin);
    calib_oledLine(2, b2);
    calib_oledLine(3, "BTN=menu");
    display.display();
    calib_beepDone();
    while (!calib_btnPressed()) delay(30);
}

// ── เมนู 3 — CPR x5: 5 รอบ สลับล้อ หาค่าเฉลี่ย + ค่าแกว่ง ───────
void calib_runSampleCPR5() {
    if (!calib_confirmScreen("CPR x5 auto")) return;
    calib_waitRelease();

    float total_rev = calib_expectedRevs();
    float cprLog[2][CALIB_CPR5_ROUNDS];
    int   done[2] = {0, 0};
    bool  stopped = false;

    for (int r = 0; r < CALIB_CPR5_ROUNDS && !stopped; r++) {
        int order[2] = {r % 2 == 0 ? 0 : 1, r % 2 == 0 ? 1 : 0};
        for (int oi = 0; oi < 2 && !stopped; oi++) {
            int m = order[oi];
            baseEncWrite(m, 0);
            unsigned long t0 = millis();
            while (millis() - t0 < CALIB_SAMPLE_TIME * 1000UL) {
                float elapsed = (millis() - t0) / 1000.0f;
                long ticks = baseEncRead(m);
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
                baseMotorSpin(m, PWM_MAX);
                delay(50);
                if (calib_btnPressed()) { stopped = true; break; }
            }
            baseMotorSpin(m, 0); baseMotorBrake(m);
            cprLog[m][done[m]] = (float)(baseEncRead(m)) / total_rev;
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

// ── เมนู 7 — Count ticks: หมุนล้อด้วยมือ 10 รอบ → CPR จริง (แม่นที่สุด) ──
//   ยกหุ่นให้ล้อลอย · ทำเครื่องหมายที่ล้อ · หมุนด้วยมือ CALIB_HAND_REVS รอบพอดี · มอเตอร์ไม่หมุน
void calib_runCountTicks() {
    if (!calib_confirmScreen("Count ticks by hand")) return;
    calib_waitRelease();

    baseMotorCoast(0);                                  // ปล่อยล้อหมุนฟรี (ไม่งั้นเบรกไฟฟ้าทำให้หมุนมือฝืด)
    baseMotorCoast(1);
    baseEncWrite(0, 0);
    baseEncWrite(1, 0);
    while (true) {
        long t1 = labs(baseEncRead(0));
        long t2 = labs(baseEncRead(1));
        display.clear();
        calib_oledHeader("COUNT TICKS");
        char h[22]; snprintf(h, sizeof(h), "Turn wheel %d rev", CALIB_HAND_REVS);
        calib_oledLine(0, h);
        char a[22]; snprintf(a, sizeof(a), "M1(L):%ld", t1);
        calib_oledLine(1, a);
        char b[22]; snprintf(b, sizeof(b), "M2(R):%ld", t2);
        calib_oledLine(2, b);
        calib_oledLine(3, "BTN=done");
        display.display();
        if (calib_btnPressed()) break;
        delay(60);
    }

    float cpr1 = labs(baseEncRead(0)) / (float)CALIB_HAND_REVS;
    float cpr2 = labs(baseEncRead(1)) / (float)CALIB_HAND_REVS;
    display.clear();
    calib_oledHeader("CPR (hand)");
    char r1[22]; snprintf(r1, sizeof(r1), "M1:%.0f cfg:%d", cpr1, (int)COUNTS_PER_REV1);
    calib_oledLine(0, r1);
    char r2[22]; snprintf(r2, sizeof(r2), "M2:%.0f cfg:%d", cpr2, (int)COUNTS_PER_REV2);
    calib_oledLine(1, r2);
    calib_oledLine(2, "0 = wheel not turned");
    calib_oledLine(3, "BTN=menu");
    display.display();
    calib_beepDone();
    while (!calib_btnPressed()) delay(30);
}

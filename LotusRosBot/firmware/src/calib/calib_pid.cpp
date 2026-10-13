// ════════════════════════════════════════════════════════════════
//  calib_pid — เมนู 10 PID tune: ทดสอบความเร็วแบบขั้นบันได (step) แล้ววาดกราฟบน OLED
//  ⚠️ ล้อหมุน — ยกหุ่นให้ล้อลอยก่อน · กด BTN ระหว่างทดสอบ = หยุดทันที
//
//  ทดสอบ: 0 rpm 0.3 วินาที → เป้า rpm ค้างไว้ hold_ms → 0 rpm 0.7 วินาที (PID คุมทุก 20 ms)
//  ปรับค่า Kp/Ki/Kd ได้จากหน้าเว็บ (tools/lrb_calib_web.py) แล้วกดบันทึกลงบอร์ด
// ════════════════════════════════════════════════════════════════
#include "calib_common.h"
#include "calib_pid.h"
#include "../base.h"

#define STEP_PRE_MS   300
#define STEP_POST_MS  700
#define STEP_DT_MS     20

bool calib_pidStep(char side, float rpm, uint32_t hold_ms, StepSink sink, bool (*abortCheck)())
{
    float maxr = baseMaxRPM();
    if (rpm >  maxr) rpm =  maxr;
    if (rpm < -maxr) rpm = -maxr;
    if (hold_ms > 5000) hold_ms = 5000;
    uint32_t total = STEP_PRE_MS + hold_ms + STEP_POST_MS;

    basePidResetState();
    float r1, r2; int p1, p2;
    baseClosedLoop(0, 0, r1, r2, p1, p2);               // อ่าน encoder ครั้งแรก (ตั้งจุดอ้างอิงเวลา)
    unsigned long t0 = millis(), next = t0;
    bool aborted = false;
    while (true) {
        uint32_t t = millis() - t0;
        if (t > total) break;
        float tgt = (t >= STEP_PRE_MS && t < STEP_PRE_MS + hold_ms) ? rpm : 0.0f;
        float tg1 = (side == 'L' || side == 'B') ? tgt : 0.0f;
        float tg2 = (side == 'R' || side == 'B') ? tgt : 0.0f;
        baseClosedLoop(tg1, tg2, r1, r2, p1, p2);
        if (sink) sink(t, tgt, r1, p1, r2, p2);
        if (calib_btnDown() || (abortCheck && abortCheck())) { aborted = true; break; }
        next += STEP_DT_MS;
        while ((long)(millis() - next) < 0) delay(1);
    }
    baseClosedLoop(0, 0, r1, r2, p1, p2);               // เบรก
    return !aborted;
}

// ── ตัวชี้วัดจากผลทดสอบ ─────────────────────────────────────
void calib_stepMetrics(const float *t_ms, const float *tgt, const float *rpm, int n, float target,
                       float &rise_ms, float &overshoot_pct, float &ss_err_pct)
{
    rise_ms = -1; overshoot_pct = 0; ss_err_pct = 0;
    if (target == 0) return;
    float t_start = -1, t10 = -1, t90 = -1, peak = 0, t_end = 0;
    for (int i = 0; i < n; i++) if (tgt[i] != 0) { if (t_start < 0) t_start = t_ms[i]; t_end = t_ms[i]; }
    if (t_start < 0) return;
    float sum = 0; int cnt = 0;
    for (int i = 0; i < n; i++) {
        if (tgt[i] == 0) continue;
        float v = rpm[i] / target;                       // สัดส่วนของเป้า (ใช้ได้ทั้งเป้าบวกและลบ)
        if (t10 < 0 && v >= 0.1f) t10 = t_ms[i];
        if (t90 < 0 && v >= 0.9f) t90 = t_ms[i];
        if (v > peak) peak = v;
        if (t_ms[i] >= t_end - 400) { sum += v; cnt++; } // 0.4 วินาทีสุดท้ายของช่วงค้าง
    }
    if (t10 >= 0 && t90 >= 0) rise_ms = t90 - t_start;
    overshoot_pct = peak > 1.0f ? (peak - 1.0f) * 100.0f : 0.0f;
    if (cnt) ss_err_pct = (1.0f - sum / cnt) * 100.0f;
}

// ── เมนู 10 บน OLED ─────────────────────────────────────────
#define MAXS 260
static float s_t[MAXS], s_tg[MAXS], s_r1[MAXS], s_r2[MAXS];
static int   s_n = 0;
static void oledSink(uint32_t t, float tgt, float r1, int p1, float r2, int p2) {
    (void)p1; (void)p2;
    if (s_n < MAXS) { s_t[s_n] = t; s_tg[s_n] = tgt; s_r1[s_n] = r1; s_r2[s_n] = r2; s_n++; }
}

void calib_runPidTune() {
    const float RPM = 60.0f; const uint32_t HOLD = 2000;
    if (!calib_confirmScreen("Step 60rpm both")) return;
    calib_waitRelease();

    display.clear();
    calib_oledHeader("PID STEP");
    calib_oledLine(0, "running... BTN=stop");
    display.display();

    s_n = 0;
    bool ok = calib_pidStep('B', RPM, HOLD, oledSink, nullptr);
    if (!ok) calib_beepWarn(); else calib_beepDone();

    float rL, oL, eL, rR, oR, eR;
    calib_stepMetrics(s_t, s_tg, s_r1, s_n, RPM, rL, oL, eL);
    calib_stepMetrics(s_t, s_tg, s_r2, s_n, RPM, rR, oR, eR);

    // กราฟ: เส้นเป้า (ประ) + ล้อซ้าย (เส้น) + ล้อขวา (จุด) · แกน y 0..1.3×เป้า
    display.clear();
    calib_oledHeader("PID STEP");
    const int GX = 0, GY = 13, GW = 128, GH = 26;
    float tmax = s_n ? s_t[s_n - 1] : 1;
    auto yOf = [&](float v) { int y = GY + GH - (int)(v / (RPM * 1.3f) * GH); return constrain(y, GY, GY + GH); };
    for (int x = 0; x < GW; x += 3) {                    // เส้นเป้า
        int i = (int)((float)x / GW * (s_n - 1)); if (i < 0) i = 0;
        display.setPixel(GX + x, yOf(s_tg[i]));
    }
    int px = -1, py = 0;
    for (int i = 0; i < s_n; i++) {
        int x = GX + (int)(s_t[i] / tmax * (GW - 1));
        int y = yOf(s_r1[i]);
        if (px >= 0) display.drawLine(px, py, x, y);
        px = x; py = y;
        if (i % 2 == 0) display.setPixel(x, yOf(s_r2[i]));
    }
    char a[24], b[24];
    snprintf(a, sizeof(a), "L %4.0fms %3.0f%% e%3.0f%%", rL, oL, eL);
    snprintf(b, sizeof(b), "R %4.0fms %3.0f%% e%3.0f%%", rR, oR, eR);
    calib_oledLine(2, a);
    calib_oledLine(3, b);
    display.display();
    while (!calib_btnPressed()) delay(30);
}

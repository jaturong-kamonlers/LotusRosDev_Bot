// ════════════════════════════════════════════════════════════════
//  calib_remote — สั่งโหมดคาลิเบรตจากหน้าเว็บบน Jetson ทางสาย USB (tools/lrb_calib_web.py)
//
//  ใช้ได้เฉพาะโหมดคาลิเบรต (ไม่มี micro-ROS ใช้สายอยู่) · บรรทัดคำสั่งขึ้นต้นด้วย '@' ตอบกลับ 1 บรรทัด
//    @PING                      → OK PING <ชื่อ> <เวอร์ชัน> CALIB
//    @STATUS                    → ST pot=… btn=… irl=… irr=… enc1=… enc2=… rpm1=… rpm2=… led=… net=…
//    @MOTOR <pwm1> <pwm2>       → หมุนมอเตอร์ตรง (-1023..1023) · ต้องส่งซ้ำทุก < 300 ms ไม่งั้นหยุดเอง
//    @STOP                      → หยุดมอเตอร์ทั้งหมด
//    @ENCZERO                   → ล้างตัวนับ encoder
//    @COAST                     → ปล่อยล้อหมุนฟรี (หมุนด้วยมือได้เบา) · @STOP = กลับเป็นเบรก
//    @LED <0|1>  @BEEP <hz> <ms>  @SERVO <a1> <a2> <a3>  @SERVOOFF
//    @PIDGET                    → PID kp1 ki1 kd1 kp2 ki2 kd2 <nvs|cfg>
//    @PIDSET <L|R|B> <kp> <ki> <kd>   @PIDSAVE   @PIDRESET
//    @STEP <L|R|B> <rpm> <hold_ms>    → S t tgt rpm1 pwm1 rpm2 pwm2 (ทุก 20 ms) … END STEP <ok|abort>
//    @NETGET                    → NETINFO host|ssid|ip|user|live
//    @EXIT                      → กลับหน้าเมนูบน OLED      @REBOOT → รีสตาร์ทบอร์ด
//  ความปลอดภัย: กด BTN บนบอร์ด = หยุดทุกอย่างและออกจากโหมดเว็บทันที
// ════════════════════════════════════════════════════════════════
#include "calib_common.h"
#include "calib_pid.h"
#include "calib_remote.h"
#include "../version.h"
#include "../base.h"
#include "../actuators.h"
#include "../netinfo.h"

#define MOTOR_WATCHDOG_MS 300

static String rx;
static bool readLine(String &out) {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') { out = rx; rx = ""; return true; }
        if (rx.length() < 120) rx += c;
    }
    return false;
}

bool calib_bootPoll() {
    String l;
    bool want = false;
    while (readLine(l)) {
        if (l.startsWith("NET\t")) netParse(l);
        else if (l.startsWith("@CALIB")) want = true;
    }
    return want;
}

static bool motor_on = false;
static unsigned long motor_ms = 0;
static String last_cmd = "-";
static bool led_on = false;

static void stopAll() {
    baseMotorBrake(0); baseMotorBrake(1);
    motor_on = false;
}

static void sendPid() {
    float a[3], b[3];
    basePidGet(0, a[0], a[1], a[2]);
    basePidGet(1, b[0], b[1], b[2]);
    Serial.printf("PID %.4f %.4f %.4f %.4f %.4f %.4f %s\n", a[0], a[1], a[2], b[0], b[1], b[2],
                  basePidFromSaved() ? "nvs" : "cfg");
}

// ระหว่าง @STEP: ตรวจว่ามี @STOP เข้ามาไหม
static bool stepAbort() {
    String l;
    while (readLine(l)) if (l.startsWith("@STOP")) return true;
    return false;
}
static void stepSink(uint32_t t, float tgt, float r1, int p1, float r2, int p2) {
    Serial.printf("S %lu %.1f %.1f %d %.1f %d\n", (unsigned long)t, tgt, r1, p1, r2, p2);
}

// คืน false = ออกจากโหมดเว็บ
static bool handle(const String &line) {
    char cmd[16] = {0};
    sscanf(line.c_str(), "@%15s", cmd);
    String c(cmd);
    last_cmd = c;

    if (c == "PING" || c == "CALIB") {
        Serial.printf("OK %s %s %s CALIB\n", c.c_str(), LRB_NAME, LRB_FW_VERSION);
    } else if (c == "STATUS") {
        // rpm จากผลต่าง encoder ระหว่าง @STATUS 2 ครั้ง (ไม่ไปแตะ PID)
        static long pe1 = 0, pe2 = 0; static unsigned long pt = 0;
        long e1 = baseEncRead(0), e2 = baseEncRead(1);
        unsigned long now = millis();
        float rpm1 = 0, rpm2 = 0;
        if (pt != 0 && now > pt) {
            float dt_min = (now - pt) / 60000.0f;
            rpm1 = (e1 - pe1) / (float)COUNTS_PER_REV1 / dt_min;
            rpm2 = (e2 - pe2) / (float)COUNTS_PER_REV2 / dt_min;
        }
        pe1 = e1; pe2 = e2; pt = now;
        int irl = 0, irr = 0;
#ifdef USE_IR_SENSORS
        irl = analogRead(IR_LEFT_PIN); irr = analogRead(IR_RIGHT_PIN);
#endif
        Serial.printf("ST pot=%d btn=%d irl=%d irr=%d enc1=%ld enc2=%ld rpm1=%.1f rpm2=%.1f led=%d net=%d motor=%d\n",
                      calib_potRaw(), calib_btnDown() ? 1 : 0, irl, irr, e1, e2, rpm1, rpm2,
                      led_on ? 1 : 0, netFresh() ? 1 : 0, motor_on ? 1 : 0);
    } else if (c == "MOTOR") {
        int a = 0, b = 0;
        sscanf(line.c_str(), "@MOTOR %d %d", &a, &b);
        a = constrain(a, -1023, 1023); b = constrain(b, -1023, 1023);
        baseMotorSpin(0, a); baseMotorSpin(1, b);
        motor_on = (a != 0 || b != 0); motor_ms = millis();
        if (!motor_on) stopAll();
        Serial.printf("OK MOTOR %d %d\n", a, b);
    } else if (c == "STOP") {
        stopAll();
        Serial.println("OK STOP");
    } else if (c == "COAST") {
        motor_on = false;
        baseMotorCoast(0); baseMotorCoast(1);
        Serial.println("OK COAST");
    } else if (c == "ENCZERO") {
        baseEncWrite(0, 0); baseEncWrite(1, 0);
        Serial.println("OK ENCZERO");
    } else if (c == "LED") {
        int v = 0; sscanf(line.c_str(), "@LED %d", &v);
        led_on = v != 0; digitalWrite(LED_PIN, led_on ? HIGH : LOW);
        Serial.printf("OK LED %d\n", led_on ? 1 : 0);
    } else if (c == "BEEP") {
        int hz = 1000, ms = 100; sscanf(line.c_str(), "@BEEP %d %d", &hz, &ms);
        buzzerBeep(constrain(hz, 20, 20000), constrain(ms, 1, 1000));
        Serial.println("OK BEEP");
    } else if (c == "SERVO") {
        int a = 90, b = 90, d = 90; sscanf(line.c_str(), "@SERVO %d %d %d", &a, &b, &d);
        servoWriteAll(constrain(a, 0, 180), constrain(b, 0, 180), constrain(d, 0, 180));
        Serial.printf("OK SERVO %d %d %d\n", a, b, d);
    } else if (c == "SERVOOFF") {
        servoDetachAll();
        Serial.println("OK SERVOOFF");
    } else if (c == "PIDGET") {
        sendPid();
    } else if (c == "PIDSET") {
        char side = 'B'; float kp, ki, kd;
        if (sscanf(line.c_str(), "@PIDSET %c %f %f %f", &side, &kp, &ki, &kd) == 4
            && kp >= 0 && ki >= 0 && kd >= 0 && kp < 50 && ki < 50 && kd < 50) {
            if (side == 'L' || side == 'B') basePidSet(0, kp, ki, kd);
            if (side == 'R' || side == 'B') basePidSet(1, kp, ki, kd);
            sendPid();
        } else Serial.println("ERR PIDSET <L|R|B> kp ki kd (0..50)");
    } else if (c == "PIDSAVE") {
        basePidSave();
        Serial.println("OK PIDSAVE");
    } else if (c == "PIDRESET") {
        basePidClearSaved();
        sendPid();
    } else if (c == "STEP") {
        char side = 'B'; float rpm = 60; int hold = 2000;
        if (sscanf(line.c_str(), "@STEP %c %f %d", &side, &rpm, &hold) >= 2 && strchr("LRB", side)) {
            stopAll();
            display.clear(); calib_oledHeader("WEB STEP");
            char b[24]; snprintf(b, sizeof(b), "%c %.0f rpm %d ms", side, rpm, hold);
            calib_oledLine(0, b); calib_oledLine(2, "BTN = STOP"); display.display();
            Serial.printf("BEGIN STEP %c %.1f %d\n", side, rpm, hold);
            bool ok = calib_pidStep(side, rpm, (uint32_t)max(hold, 0), stepSink, stepAbort);
            Serial.printf("END STEP %s\n", ok ? "ok" : "abort");
        } else Serial.println("ERR STEP <L|R|B> rpm hold_ms");
    } else if (c == "NETGET") {
        Serial.printf("NETINFO %s|%s|%s|%s|%d\n", net_host.c_str(), net_ssid.c_str(), net_ip.c_str(),
                      net_user.c_str(), netFresh() ? 1 : 0);
    } else if (c == "EXIT") {
        stopAll();
        Serial.println("OK EXIT");
        return false;
    } else if (c == "REBOOT") {
        stopAll();
        Serial.println("OK REBOOT");
        Serial.flush(); delay(50);
        ESP.restart();
    } else {
        Serial.printf("ERR unknown %s\n", cmd);
    }
    return true;
}

static void remoteScreen() {
    display.clear();
    calib_oledHeader("WEB CONTROL");
    calib_oledLine(0, "browser :8091");
    String l = "cmd: " + last_cmd;
    calib_oledLine(1, l.c_str());
    calib_oledLine(2, motor_on ? "MOTOR ON" : "motor off");
    calib_oledLine(3, "BTN = stop/exit");
    display.display();
}

void calib_remoteLoop(const String &first) {
    calib_beepConfirm();
    if (!handle(first)) return;
    unsigned long lastScr = 0;
    while (true) {
        String l;
        while (readLine(l)) {
            if (l.startsWith("NET\t")) { netParse(l); continue; }
            if (l.startsWith("@") && !handle(l)) { stopAll(); return; }
        }
        if (motor_on && millis() - motor_ms > MOTOR_WATCHDOG_MS) {   // เว็บเงียบ → หยุด
            stopAll();
            Serial.println("EVT watchdog stop");
        }
        if (calib_btnDown()) {                                       // ปุ่มบนบอร์ด = หยุดฉุกเฉิน
            stopAll(); servoDetachAll();
            Serial.println("EVT button stop");
            calib_beepWarn();
            while (calib_btnDown()) delay(10);
            calib_btnResetEdge();
            return;
        }
        if (millis() - lastScr > 300) { lastScr = millis(); remoteScreen(); }
        delay(2);
    }
}

bool calib_remotePoll() {
    String l;
    while (readLine(l)) {
        if (l.startsWith("NET\t")) { netParse(l); continue; }
        if (l.startsWith("@")) { calib_remoteLoop(l); return true; }
    }
    return false;
}

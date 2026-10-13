// ════════════════════════════════════════════════════════════════
//  calib_network — เมนู 8 Network info · 9 SSH login
//  ข้อมูลมาจาก service lotus-netinfo บน Jetson (ส่งทุก 3 วินาทีทาง USB)
//  แสดงเฉพาะเมื่อได้รับภายใน 10 วินาที ไม่งั้นขึ้น "No data" (รู้ทันทีว่าสาย/Jetson ใช้ได้ไหม)
// ════════════════════════════════════════════════════════════════
#include "calib_common.h"
#include "../netinfo.h"

static void calib_noData() {
    calib_oledLine(0, "No data");
    calib_oledLine(1, "check USB cable /");
    calib_oledLine(2, "Jetson lotus-netinfo");
}

// ── เมนู 8 — SSID / รหัส / IP ของ Jetson · หมุน POT = เลื่อนข้อความยาว ──
void calib_runNetInfo() {
    calib_waitRelease();
    while (true) {
        bool wasLive = netFresh();
        if (netPollSerial(true) && !wasLive) calib_beepShort();   // บี๊บครั้งเดียวตอนเริ่มได้รับ
        int maxLen = max(net_ssid.length(), net_pass.length());
        int off = (maxLen > 16) ? calib_potMap(maxLen - 15) : 0;
        display.clear();
        calib_oledHeader(net_host.length() ? net_host.c_str() : "NETWORK");
        if (!netFresh()) {
            calib_noData();
        } else {
            String a = "SSID:" + net_ssid.substring(min((int)net_ssid.length(), off));
            String b = "PW:"   + net_pass.substring(min((int)net_pass.length(), off));
            String c = "IP:"   + net_ip;
            calib_oledLine(0, a.substring(0, 21).c_str());
            calib_oledLine(1, b.substring(0, 21).c_str());
            calib_oledLine(2, c.c_str());
        }
        calib_oledLine(3, netFresh() ? "live  BTN=menu" : "No data BTN=menu");
        display.display();
        if (calib_btnPressed()) break;
        delay(80);
    }
}

// ── เมนู 9 — คำสั่ง ssh user@ip + user/รหัสของ Jetson (/etc/lotusbot/ssh_login.conf) ──
void calib_runSshInfo() {
    calib_waitRelease();
    while (true) {
        bool wasLive = netFresh();
        if (netPollSerial(true) && !wasLive) calib_beepShort();
        String cmd = "ssh " + net_user + "@" + net_ip;
        int off = (cmd.length() > 21) ? calib_potMap(cmd.length() - 20) : 0;
        display.clear();
        calib_oledHeader("SSH LOGIN");
        if (!netFresh()) {
            calib_noData();
        } else if (net_user.length() == 0 || net_ip.length() == 0) {
            calib_oledLine(0, "No login from Jetson");
            calib_oledLine(1, "set /etc/lotusbot/");
            calib_oledLine(2, "ssh_login.conf");
        } else {
            String u = "user: " + net_user;
            String p = "pass: " + net_spass;
            calib_oledLine(0, cmd.substring(off, off + 21).c_str());
            calib_oledLine(1, u.substring(0, 21).c_str());
            calib_oledLine(2, p.substring(0, 21).c_str());
        }
        calib_oledLine(3, netFresh() ? "live  BTN=menu" : "No data BTN=menu");
        display.display();
        if (calib_btnPressed()) break;
        delay(80);
    }
}

// ════════════════════════════════════════════════════════════════
//  calib_menu — หน้าเลือกเมนูคาลิเบรต (หมุน POT เลื่อน · กดปุ่มเลือก)
//
//    1 Spin motors    ทดสอบทิศ FWD/LEFT/RIGHT          (calib_motor.cpp)
//    2 Sample CPR     วัด tick/รอบ แบบประมาณ M1→M2       (calib_motor.cpp)
//    3 Sample CPR x5  5 รอบ ค่าเฉลี่ย + ค่าแกว่ง          (calib_motor.cpp)
//    4 Servo check    POT = องศา servo                  (calib_io.cpp)
//    5 Sensor check   ค่าทุกขาแบบ real-time              (calib_io.cpp)
//    6 LED test       กะพริบ POT = ความเร็ว              (calib_io.cpp)
//    7 Count ticks    หมุนล้อมือ 10 รอบ → CPR จริง        (calib_motor.cpp)
//    8 Network info   SSID / รหัส / IP ของ Jetson        (calib_network.cpp)
//    9 SSH login      ssh user@ip + รหัส                 (calib_network.cpp)
// ════════════════════════════════════════════════════════════════
#include "calib_common.h"
#include "../version.h"

static const char *calib_menuItems[CALIB_MENU_ITEMS] = {
    "Spin motors",
    "Sample CPR",
    "Sample CPR x5",
    "Servo check",
    "Sensor check",
    "LED test",
    "Count ticks",
    "Network info",
    "SSH login"
};

static int calib_showMenu() {
    while (true) {
        int sel = calib_potMap(CALIB_MENU_ITEMS);
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 0, LRB_NAME " CALIB");
        display.drawLine(0, 11, 127, 11);
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        int start = constrain(sel - 1, 0, CALIB_MENU_ITEMS - CALIB_OLED_ROWS);
        for (int i = 0; i < CALIB_OLED_ROWS; i++) {
            int idx = start + i;
            if (idx >= CALIB_MENU_ITEMS) break;
            int y = CALIB_OLED_CONTENT_Y + i * CALIB_OLED_ROW_H;
            char buf[22];
            snprintf(buf, sizeof(buf), " %d.%s", idx + 1, calib_menuItems[idx]);
            if (idx == sel) {                         // แถบสีขาว = ตัวที่เลือกอยู่
                display.fillRect(0, y - 1, 128, CALIB_OLED_ROW_H);
                display.setColor(BLACK);
                display.drawString(0, y, String(buf));
                display.setColor(WHITE);
            } else {
                display.drawString(0, y, String(buf));
            }
        }
        display.display();
        if (calib_btnPressed()) return sel;
        delay(40);
    }
}

void runCalibration() {
    switch (calib_showMenu()) {
        case 0: calib_runSpin();        break;
        case 1: calib_runSampleCPR();   break;
        case 2: calib_runSampleCPR5();  break;
        case 3: calib_runServoCheck();  break;
        case 4: calib_runSensorCheck(); break;
        case 5: calib_runLEDTest();     break;
        case 6: calib_runCountTicks();  break;
        case 7: calib_runNetInfo();     break;
        case 8: calib_runSshInfo();     break;
        default: break;
    }
}

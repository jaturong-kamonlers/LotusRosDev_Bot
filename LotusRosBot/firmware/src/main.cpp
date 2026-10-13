// ════════════════════════════════════════════════════════════════
//  LotusRosBot — เฟิร์มแวร์บอร์ด ESP32 ของหุ่น (LotusRosDev Shield, ESP32 DevKit V1 30-pin)
//
//  เปิดบอร์ดแล้วนับถอยหลัง 5 วินาที:
//    กดปุ่มค้าง  → โหมดคาลิเบรต (เมนูบน OLED · src/calib/)
//    ไม่กด       → โหมด ROS2 (micro-ROS ผ่าน USB ไป agent บน Jetson · src/ros_link.cpp)
//
//  โมดูล:
//    ros_link   ต่อ agent, topic, รับคำสั่ง, ส่งข้อมูล      base      มอเตอร์ + PID + odometry ล้อ
//    heading    IMU + มุมหมุนจาก gyro (50 Hz)            sensors   IR, POT, ปุ่ม
//    actuators  LED, buzzer, servo                      display   จอ OLED
//    netinfo    ข้อมูลไวไฟ/IP ของ Jetson                calib/    เมนูคาลิเบรต 9 เมนู
//  ค่าตั้งทั้งหมด: include/lotusrosbot_config.h · เวอร์ชัน: src/version.h
//
//  พัฒนาต่อจาก linorobot2_hardware (Apache-2.0, Juan Miguel Jimeno) — ดู README.md
// ════════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "version.h"
#include "syslog.h"
#include "wifis.h"
#include "ota.h"
#include "ros_link.h"
#include "base.h"
#include "heading.h"
#include "sensors.h"
#include "actuators.h"
#include "display.h"
#include "netinfo.h"
#include "calib/calib_common.h"

#ifdef WDT_TIMEOUT
#include <esp_task_wdt.h>
#endif

static bool calib_mode = false;

void setup()
{
#ifdef BOARD_INIT
    BOARD_INIT;
#endif

    Serial.begin(BAUDRATE);
    netLoad();                       // ข้อมูลเครือข่าย Jetson ที่จำไว้ครั้งก่อน (NVS)
    ledInit();

    Wire.begin(SDA_PIN, SCL_PIN);    // I2C ใช้ร่วม: IMU + OLED

#ifdef WDT_TIMEOUT
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
#endif

    // ── จอ + IMU ─────────────────────────────────────────────
    displayInit();
    displayBootScreen();
    delay(100);

    if (!imuInit()) {
        displayImuFail();
        while (1) { flashLED(3); }   // ไม่พบ MPU6050 → หยุด (ตรวจสาย I2C)
    }
    displayImuOk();
    // build แบบ USB serial ไม่มีโค้ด WiFi — วิทยุ WiFi ไม่ถูกเปิดเลย (v2026.10.12b)

    magInit();
    servoInitDetached();             // servo คลายตัว จนกว่าจะได้ /servo/cmd ครั้งแรก
    buzzerInit();
    sensorsInit();
    rosLinkInitMessages();

    buzzerBeep(1000, 100);           // บี๊บ = บอร์ดเริ่มทำงาน

    // ── นับถอยหลัง 5 วินาที: กดปุ่ม = คาลิเบรต · ไม่กด = ROS2 ──
    calib_mode = false;
    for (int i = 5; i > 0 && !calib_mode; i--) {
        displayCountdown(i);
        unsigned long t0 = millis();
        while (millis() - t0 < 1000) {
            if (buttonIsDown()) { calib_mode = true; break; }
            netPollSerial(true);     // ระหว่างนี้รับข้อมูลไวไฟจาก Jetson ได้
            delay(20);
        }
    }

    if (calib_mode) {
        displayModeBanner("CALIBRATION MODE", "Starting...");
        calib_beepEnterCalib();      // โด เร มี
        delay(500);
        baseBeginEncoders();         // โหมดนี้ไม่มี micro-ROS → เริ่ม encoder ได้เลย
        return;                      // Serial ว่าง (ไม่ init micro-ROS)
    }

    displayModeBanner("ROS2 MODE", "Connecting...");
    buzzerBeep(1800, 100);           // บี๊บสั้น = เข้าโหมด ROS2
    rosLinkBegin();
    updateDisplay();
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
}

void loop()
{
    if (calib_mode) {
        runCalibration();            // วนเมนูคาลิเบรต (ไม่กลับโหมด ROS2 จนกว่าจะรีเซ็ตบอร์ด)
        return;
    }

    rosLinkSpin();

    EXECUTE_EVERY_N_MS(200, readSensors());           // 5 Hz — ถี่กว่านี้สาย 115200 ตัน (2026-07-04)
    EXECUTE_EVERY_N_MS(200, updateDisplaySensors());
#ifdef USE_BUZZER
    EXECUTE_EVERY_N_MS(10, buzzerCheck());
#endif

    runWifis();
    runOta();

#ifdef WDT_TIMEOUT
    esp_task_wdt_reset();
#endif
}

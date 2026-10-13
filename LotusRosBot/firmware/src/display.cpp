// display.cpp — ดูคำอธิบายใน display.h
#include <Arduino.h>
#include "display.h"
#include "config.h"
#include "version.h"
#include "netinfo.h"
#include "sensors.h"
#include "ros_link.h"

#ifdef USE_WIFI_TRANSPORT
#include <WiFi.h>
#endif

SSD1306Wire display(0x3c, SDA_PIN, SCL_PIN);

void displayInit() {
    display.init();
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
}

void displayBootScreen() {
    display.drawString(0, 0, "FW " LRB_FW_VERSION);
    display.drawString(0, 10, "Init IMU...");
    display.drawString(0, 50, LRB_NAME);
    display.display();
}

void displayImuFail() {
    display.drawString(0, 25, "--IMU FAIL--");
    display.drawString(0, 35, "ESP32 Stop process");
    display.display();
}

void displayImuOk() {
    display.clear();
    display.drawString(0, 0,  "IMU......OK");
#ifdef USE_WIFI_TRANSPORT
    display.drawString(0, 10, "Wifi connecting...");
#else
    display.drawString(0, 10, "USB serial mode");
#endif
    display.display();
}

void displayCountdown(int seconds_left) {
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 0,  netSeen() ? ("IP " + net_ip) : String("IP: No data"));
    display.drawLine(0, 11, 127, 11);
    display.drawString(64, 18, "Hold BTN = CALIB");
    display.drawString(64, 32, "Release  = ROS2");
    char cbuf[20];
    snprintf(cbuf, sizeof(cbuf), "Auto ROS2 in %ds", seconds_left);
    display.drawString(64, 48, cbuf);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.display();
}

void displayModeBanner(const char *mode, const char *msg) {
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 16, mode);
    display.drawString(64, 36, msg);
    display.display();
}

void updateDisplay()
{
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
#ifdef USE_WIFI_TRANSPORT
    display.drawString(0, 0, "IP: " + String(WiFi.localIP().toString().c_str()));
#else
    display.drawString(0, 0, "IP:" + (netSeen() ? net_ip : String("No data")));
#endif
    display.drawString(0, 13, rosConnected() ? "ROS2: OK" : "ROS2: WAIT");
    display.display();
}

void updateDisplaySensors()
{
    // ════ OLED 128×64 ═══════════════════════════════════════
    // y= 0  ROS2 status + IP          y=10  เส้นคั่น
    // y=13  L: x.xxV [xxxx]           y=25  แถบ L (100 px)
    // y=37  R: x.xxV [xxxx]           y=49  แถบ R + T: threshold (ขวา)
    display.clear();
    display.setFont(ArialMT_Plain_10);

#ifdef USE_WIFI_TRANSPORT
    String ip_short = WiFi.localIP().toString();
    int last_dot = ip_short.lastIndexOf('.');
#else
    String ip_short = netSeen() ? net_ip : String("No data");   // IP ของ Jetson ที่ได้รับทาง serial ตั้งแต่บูต
    int last_dot = -1;                                            // แสดงเต็ม
#endif
    String status_str = rosConnected() ? "OK" : "--";
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "ROS2:" + status_str);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(127, 0, ip_short.substring(last_dot + 1));
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    display.drawLine(0, 10, 127, 10);

#ifdef USE_IR_SENSORS
    char buf_l[24];
    snprintf(buf_l, sizeof(buf_l), "L:%4.2fV[%4d]", ir_left_v, ir_left_raw);
    display.drawString(0, 13, buf_l);
    int pct_l = (int)constrain(ir_left_v / 3.3f * 100.0f, 0, 100);
    display.drawRect(0, 25, 100, 6);
    display.fillRect(0, 25, pct_l, 6);

    char buf_r[24];
    snprintf(buf_r, sizeof(buf_r), "R:%4.2fV[%4d]", ir_right_v, ir_right_raw);
    display.drawString(0, 37, buf_r);
    int pct_r = (int)constrain(ir_right_v / 3.3f * 100.0f, 0, 100);
    display.drawRect(0, 49, 100, 6);
    display.fillRect(0, 49, pct_r, 6);

    char buf_t[16];
    snprintf(buf_t, sizeof(buf_t), "T:%4.2f", ir_threshold);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(127, 49, buf_t);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
#else
#ifdef USE_POTENTIOMETER
    display.drawString(0, 20, "POT:" + String(pot_v, 2) + "V [" + String(pot_raw) + "]");
#endif
#ifdef USE_BUTTON
    display.drawString(0, 36, "BTN:" + String(button_pressed ? "1" : "0"));
#endif
#endif

    display.display();
}

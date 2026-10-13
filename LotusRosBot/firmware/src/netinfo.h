// ════════════════════════════════════════════════════════════════
//  netinfo — รับข้อมูลเครือข่ายของ Jetson ทาง USB serial แล้วแสดงบน OLED
//  Jetson (service lotus-netinfo) ส่งบรรทัด
//    "NET\t<host>\t<ssid>\t<pass>\t<ip>[\t<user>\t<sshpass>]\n"  ทุก 3 วินาที
//  เฉพาะตอนที่ไม่มี agent/esptool เปิดพอร์ตอยู่ → ESP32 จำลง NVS
//  แสดง "เฉพาะที่ได้รับจริงทาง USB" (ไม่ได้รับใน 10 วินาที = "No data")
// ════════════════════════════════════════════════════════════════
#ifndef LRB_NETINFO_H
#define LRB_NETINFO_H

#include <Arduino.h>

extern String net_host, net_ssid, net_pass, net_ip, net_user, net_spass;

void netLoad();                          // อ่านค่าที่จำไว้ใน NVS ตอนบูต
bool netParse(const String &line);       // แปลงบรรทัด NET → true = ได้ข้อมูล
bool netPollSerial(bool exclusive);      // exclusive=true: ไม่มี micro-ROS ทิ้ง byte อื่นได้
                                         // exclusive=false: รอ agent อยู่ อ่านเฉพาะบรรทัดที่ขึ้นต้นด้วย 'N'
bool netFresh();                         // ได้รับภายใน 10 วินาที (live)
bool netSeen();                          // ได้รับอย่างน้อย 1 ครั้งตั้งแต่บูต

#endif

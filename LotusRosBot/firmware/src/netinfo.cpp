// netinfo.cpp — ดูคำอธิบายใน netinfo.h
#include "netinfo.h"
#include <Preferences.h>

static Preferences netPrefs;
String net_host, net_ssid, net_pass, net_ip, net_user, net_spass;   // user/sshpass = บัญชี SSH ของ Jetson

static unsigned long net_rx_ms = 0;                 // เวลาที่ได้รับบรรทัด NET ล่าสุด · 0 = ยังไม่ได้รับเลย
static const unsigned long NET_FRESH_MS = 10000;    // ได้รับภายใน 10 วินาที = live (Jetson ส่งทุก 3 วินาที)

bool netFresh() { return net_rx_ms != 0 && millis() - net_rx_ms < NET_FRESH_MS; }
bool netSeen()  { return net_rx_ms != 0; }

void netLoad() {
    netPrefs.begin("lotusnet", true);
    net_host = netPrefs.getString("host", "");
    net_ssid = netPrefs.getString("ssid", "");
    net_pass = netPrefs.getString("pass", "");
    net_ip   = netPrefs.getString("ip", "");
    net_user = netPrefs.getString("user", "");
    net_spass= netPrefs.getString("spass", "");
    netPrefs.end();
}

bool netParse(const String &line) {
    if (!line.startsWith("NET\t")) return false;
    String fld[7]; int n = 0, start = 0;
    for (int i = 0; i <= (int)line.length() && n < 7; i++) {
        if (i == (int)line.length() || line[i] == '\t') { fld[n++] = line.substring(start, i); start = i + 1; }
    }
    if (n < 5) return false;
    fld[4].trim();
    if (n < 7) { fld[5] = net_user; fld[6] = net_spass; }   // Jetson รุ่นเก่าส่ง 5 ช่อง — คงค่าเดิม
    fld[6].trim();
    if (fld[4].length() < 7 || fld[4].length() > 15) return false;   // IPv4 เท่านั้น
    net_rx_ms = millis();                              // ได้รับข้อมูลจริงทาง USB
    if (fld[1] == net_host && fld[2] == net_ssid && fld[3] == net_pass && fld[4] == net_ip
        && fld[5] == net_user && fld[6] == net_spass) return true;
    net_host = fld[1]; net_ssid = fld[2]; net_pass = fld[3]; net_ip = fld[4];
    net_user = fld[5]; net_spass = fld[6];
    netPrefs.begin("lotusnet", false);                 // เขียน NVS เฉพาะเมื่อค่าเปลี่ยน (ถนอม flash)
    netPrefs.putString("host", net_host);
    netPrefs.putString("ssid", net_ssid);
    netPrefs.putString("pass", net_pass);
    netPrefs.putString("ip",   net_ip);
    netPrefs.putString("user", net_user);
    netPrefs.putString("spass",net_spass);
    netPrefs.end();
    return true;
}

bool netPollSerial(bool exclusive) {
    bool got = false;
    while (Serial.available() > 0) {
        int c = Serial.peek();
        if (c != 'N') {
            if (!exclusive) return got;                // ไม่แย่ง frame 0x7E ของ micro-ROS
            Serial.read();
            continue;
        }
        Serial.setTimeout(50);
        String line = Serial.readStringUntil('\n');
        if (netParse(line)) got = true;
    }
    return got;
}

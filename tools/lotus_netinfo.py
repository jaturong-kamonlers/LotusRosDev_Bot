#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lotus_netinfo.py — ส่ง hostname / SSID / รหัสไวไฟ / IP ของ Jetson ไปให้ ESP32 แสดงบน OLED (เฟิร์มแวร์ ≥ v2026.10.12c)
รันเป็น systemd service (root เพื่ออ่านรหัสไวไฟจาก NetworkManager):  sudo systemctl status lotus-netinfo

ส่งบรรทัด "NET\t<host>\t<ssid>\t<pass>\t<ip>\t<user>\t<sshpass>\n" ไป /dev/esp32 ตอนเริ่ม และเมื่อข้อมูลเปลี่ยน (v2026.10.13 · เดิมทุก 3 วิ)
user/sshpass อ่านจาก /etc/lotusbot/ssh_login.conf (root อ่านได้คนเดียว) บรรทัด user=… และ pass=…
เฉพาะตอนที่ไม่มีโปรแกรมอื่นเปิดพอร์ตอยู่ (agent, esptool, pio, minicom ...) และไม่มี start/stop/efuse script ทำงาน
→ ไม่ยุ่งกับ micro-ROS ระหว่างใช้งานหุ่น · การเปิดพอร์ตไม่ทำให้ ESP32 รีเซ็ต (ทดสอบแล้ว)
"""
import os, re, socket, subprocess, time
import serial

PORT = '/dev/esp32'
PERIOD = 3.0
LOGIN_CONF = '/etc/lotusbot/ssh_login.conf'
BUSY_SCRIPTS = r'esptool|espefuse|platformio|pio run|start\.sh|start2\.sh|stop\.sh|efuse\.sh|agent\.sh|flash\.sh'


def run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return ''


def port_busy():
    real = os.path.realpath(PORT)
    if not os.path.exists(real):
        return True
    if subprocess.run(['fuser', real], capture_output=True).returncode == 0:   # มีคนเปิดพอร์ตอยู่
        return True
    return subprocess.run(['pgrep', '-f', BUSY_SCRIPTS], capture_output=True).returncode == 0


def clean(x):
    return re.sub(r'[\t\r\n]', ' ', x or '').strip() or '-'


def ssh_login():
    user, pw = '', ''
    try:
        for ln in open(LOGIN_CONF, encoding='utf-8'):
            k, _, v = ln.strip().partition('=')
            if k == 'user': user = v
            elif k == 'pass': pw = v
    except OSError:
        pass
    if os.environ.get('LOTUS_NETINFO_HIDE_PASS') == '1' and pw:
        pw = '(hidden)'
    return clean(user), clean(pw)


def netinfo():
    host = socket.gethostname()
    ssid = psk = '-'
    dev = ''
    for line in run(['nmcli', '-t', '-f', 'NAME,TYPE,DEVICE', 'connection', 'show', '--active']).splitlines():
        name, typ, d = (line.rsplit(':', 2) + ['', ''])[:3]
        if typ == '802-11-wireless':
            ssid = run(['nmcli', '-g', '802-11-wireless.ssid', 'connection', 'show', name]) or name
            psk = run(['nmcli', '-s', '-g', '802-11-wireless-security.psk', 'connection', 'show', name]) or '-'
            if os.environ.get('LOTUS_NETINFO_HIDE_PASS') == '1':   # ซ่อนรหัสบน OLED: ใส่ Environment=LOTUS_NETINFO_HIDE_PASS=1 ใน service
                psk = '(hidden)'
            dev = d
            break
    ip = ''
    for d in ([dev] if dev else []) + ['wlan0', 'eth0']:
        m = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', run(['ip', '-4', '-o', 'addr', 'show', 'dev', d]))
        if m:
            ip = m.group(1)
            if d == 'eth0' and not dev:
                ssid, psk = 'LAN(eth0)', '-'
            break
    return clean(host), clean(ssid), clean(psk), ip


def main():
    # v2026.10.13: ส่งเฉพาะตอนเริ่ม service และเมื่อข้อมูลเปลี่ยน (เดิมส่งทุก 3 วิ)
    #   เหตุผล: การเปิดพอร์ตทำ ESP32 รีเซ็ตได้ ~2% ต่อครั้ง (ตรวจจับ rst:0x1 ได้ 2/108 ครั้ง)
    #   → ส่งทุก 3 วิ = บี๊บทุก 3–5 นาที · ESP32 เก็บค่าลง NVS อยู่แล้ว ไม่ต้องส่งซ้ำ
    #   ยังเช็กเครือข่ายทุก PERIOD วิ (ไม่เปิดพอร์ต) · ส่งไม่สำเร็จ (พอร์ตไม่ว่าง/error) จะลองใหม่รอบถัดไป
    sent = None
    while True:
        time.sleep(PERIOD)
        host, ssid, psk, ip = netinfo()
        if not ip:                      # ไม่มีเครือข่าย → แจ้งจอว่า 0.0.0.0 (ไม่ค้าง IP เก่า)
            ip, ssid, psk = '0.0.0.0', '(no network)', '-'
        user, spw = ssh_login()
        info = (host, ssid, psk, ip, user, spw)
        if info == sent:
            continue                    # ข้อมูลเดิม → ไม่เปิดพอร์ต
        if port_busy():
            continue                    # agent/esptool ใช้พอร์ตอยู่ → รอบหน้าลองใหม่
        line = f'NET\t{host}\t{ssid}\t{psk}\t{ip}\t{user}\t{spw}\n'.encode('utf-8', 'replace')
        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout, s.write_timeout = PORT, 115200, 0.2, 0.5
            s.dtr = False          # ไม่แตะสาย reset ของ ESP32
            s.rts = False
            s.open()
            s.write(line)
            s.flush()
            s.close()
            sent = info
            print(f'sent host={host} ssid={ssid} ip={ip} user={user}', flush=True)
        except Exception as e:
            print('serial error:', e, flush=True)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calib_drive.py — ทดสอบจูนฐานล้อ LotusRosDev_Bot (คู่มือบท "จูนหุ่น")
รันบน Jetson หลังเปิด agent แล้ว (bash ~/agent.sh)  ·  ห้ามรันพร้อม nav.sh / mapping.sh / ภารกิจ

  python3 ~/calib_drive.py watch                          # ไม่ขยับ: ดูระยะ/มุมจาก odom (เข็นหุ่นด้วยมือ)
  python3 ~/calib_drive.py straight --dist 1.0            # เดินตรง 1 m แล้วกรอกระยะจริง → WHEEL_DIAMETER ใหม่
  python3 ~/calib_drive.py rotate --turns 3               # หมุน 3 รอบ แล้วกรอกมุมจริง → LR_WHEELS_DISTANCE ใหม่
  python3 ~/calib_drive.py step --speed 0.15 --time 3     # ความเร็วคงที่ บันทึก CSV → ดูการตอบสนอง PID

⚠️ ทุกโหมดที่ขยับหุ่นต้องมีคนเฝ้า · พื้นที่ว่าง · กด Ctrl+C = หยุดทันที (หุ่นหยุดเองใน 200 ms ด้วย)
ค่าที่ได้ → ใส่ใน ~/LotusRosDev_Bot/LotusRosBot/firmware/include/lotusrosbot_config.h แล้ว build + flash (ไม่มีการแก้ไฟล์อัตโนมัติ)
"""
import argparse, csv, math, os, re, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CONFIG = os.path.expanduser('~/LotusRosDev_Bot/LotusRosBot/firmware/include/lotusrosbot_config.h')
if not os.path.exists(CONFIG):   # เฟิร์มแวร์เก่า (ก่อน v2026.10.14b)
    CONFIG = os.path.expanduser('~/LotusRosDev_Bot/config/custom/myrobot_config.h')
MAX_LIN, MAX_ANG = 0.25, 1.2          # เพดานความปลอดภัย (m/s, rad/s)


def read_config(path=CONFIG):
    """อ่านค่าปัจจุบันจาก lotusrosbot_config.h (ถ้าไม่มีไฟล์ใช้ค่าเริ่มต้นของหุ่น)"""
    v = {'WHEEL_DIAMETER': 0.07, 'LR_WHEELS_DISTANCE': 0.21, 'COUNTS_PER_REV1': 958, 'COUNTS_PER_REV2': 925}
    try:
        txt = open(path, encoding='utf-8').read()
        for k in v:
            m = re.search(r'^\s*#define\s+%s\s+([0-9.]+)' % k, txt, re.M)
            if m:
                v[k] = float(m.group(1))
    except OSError:
        pass
    return v


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Calib(Node):
    def __init__(self, odom_topic):
        super().__init__('calib_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.last = None            # (t, x, y, yaw, vx, wz)
        self.dist = 0.0             # ระยะสะสมตามเส้นทาง (m)
        self.turn = 0.0             # มุมสะสม (rad, มีเครื่องหมาย)
        self.samples = []           # สำหรับ step: (t, vx, wz)
        self.stamp = 0.0

    def on_odom(self, m):
        p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        cur = (time.time(), p.x, p.y, yaw_of(q), tw.linear.x, tw.angular.z)
        if self.last:
            self.dist += math.hypot(cur[1] - self.last[1], cur[2] - self.last[2])
            self.turn += wrap(cur[3] - self.last[3])
        self.last = cur
        self.stamp = cur[0]
        self.samples.append((cur[0], cur[4], cur[5]))

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x = max(-MAX_LIN, min(MAX_LIN, vx))
        t.angular.z = max(-MAX_ANG, min(MAX_ANG, wz))
        self.pub.publish(t)

    def stop(self):
        for _ in range(10):
            self.send(0.0, 0.0)
            self.spin_for(0.05)

    def wait_odom(self, sec=5.0):
        end = time.time() + sec
        while time.time() < end and self.last is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.last is not None

    def reset(self):
        self.dist = self.turn = 0.0
        self.samples.clear()


def preflight(n, args):
    if not n.wait_odom():
        sys.exit('❌ ไม่ได้รับ odom — เปิด agent แล้วหรือยัง? (bash ~/agent.sh แล้วกด RST ที่ ESP32)')
    n.spin_for(0.5)
    others = n.count_publishers('/cmd_vel') - 1
    if others > 0:
        sys.exit(f'❌ มี node อื่นส่ง /cmd_vel อยู่ {others} ตัว (Nav2/teleop/ภารกิจ?) — ปิดก่อน แล้วรันใหม่')
    if args.mode != 'watch' and not args.yes:
        print('⚠️  หุ่นจะเคลื่อนที่ — มีคนเฝ้าหุ่น และพื้นที่ว่างพอแล้วใช่ไหม?')
        if input('พิมพ์ y แล้ว Enter เพื่อเริ่ม: ').strip().lower() != 'y':
            sys.exit('ยกเลิก')


def ask_float(prompt, given):
    if given is not None:
        return given
    s = input(prompt).strip()
    return float(s) if s else None


def drive(n, vx, wz, done, timeout):
    """ส่งคำสั่งต่อเนื่อง 20 Hz (ramp ขึ้น 0.6 s) จน done() เป็นจริง / หมดเวลา / odom หาย"""
    t0 = time.time()
    while True:
        el = time.time() - t0
        k = min(1.0, el / 0.6)
        n.send(vx * k, wz * k)
        n.spin_for(0.05)
        if done():
            return 'ok'
        if el > timeout:
            return 'timeout'
        if time.time() - n.stamp > 1.0:
            return 'odom-lost'
        if el > 3.0 and n.dist < 0.01 and abs(n.turn) < 0.05:
            return 'not-moving'


def mode_watch(n, cfg, args):
    print('ไม่ส่งคำสั่งใด ๆ — เข็น/หมุนหุ่นด้วยมือ แล้วดูค่า (Ctrl+C = จบ)')
    n.reset()
    try:
        while True:
            n.spin_for(0.5)
            print(f'\rระยะ odom {n.dist:6.3f} m   มุม odom {math.degrees(n.turn):8.1f}°   ', end='', flush=True)
    except KeyboardInterrupt:
        print()


def mode_straight(n, cfg, args):
    d = abs(args.dist)
    sp = min(abs(args.speed), MAX_LIN) * (1 if args.dist > 0 else -1)
    n.reset()
    res = drive(n, sp, 0.0, lambda: n.dist >= d - 0.01, d / abs(sp) * 2 + 3)
    n.stop(); n.spin_for(1.0)
    print(f'ผล: {res} · ระยะตาม odom = {n.dist:.3f} m · มุมเบี่ยงตาม odom = {math.degrees(n.turn):.1f}°')
    real = ask_float('วัดระยะจริงด้วยตลับเมตร (m) [Enter = ข้าม]: ', args.measured)
    if real:
        D = cfg['WHEEL_DIAMETER'] * real / n.dist
        print(f'→ WHEEL_DIAMETER ใหม่ = {cfg["WHEEL_DIAMETER"]} × {real:.3f} ÷ {n.dist:.3f} = {D:.4f}')
        print(f'   #define WHEEL_DIAMETER      {D:.4f}')
    lat = ask_float('หุ่นเบี้ยวออกจากเส้นตรงกี่ cm (ซ้าย = +, ขวา = −) [Enter = ข้าม]: ', args.lateral)
    if lat and real:
        L = cfg['LR_WHEELS_DISTANCE']
        th = 2 * (lat / 100.0) / real                    # มุมที่หุ่นเลี้ยวจริงโดยประมาณ
        r = (1 + th * L / (2 * real)) / (1 - th * L / (2 * real))   # dr/dl จริง
        c1, c2 = cfg['COUNTS_PER_REV1'] * math.sqrt(r), cfg['COUNTS_PER_REV2'] / math.sqrt(r)
        print(f'→ ล้อขวาวิ่งจริงมากกว่าซ้าย {100 * (r - 1):+.2f}% · ปรับ CPR (คงค่าเฉลี่ยเดิม):')
        print(f'   #define COUNTS_PER_REV1   {c1:.0f}   // ซ้าย')
        print(f'   #define COUNTS_PER_REV2   {c2:.0f}   // ขวา')


def mode_rotate(n, cfg, args):
    tgt = 2 * math.pi * abs(args.turns)
    w = min(abs(args.speed), MAX_ANG) * (1 if args.turns > 0 else -1)
    n.reset()
    res = drive(n, 0.0, w, lambda: abs(n.turn) >= tgt - 0.02, tgt / abs(w) * 2 + 3)
    n.stop(); n.spin_for(1.0)
    odeg = math.degrees(abs(n.turn))
    print(f'ผล: {res} · มุมตาม odom = {odeg:.1f}° ({odeg / 360:.3f} รอบ)')
    real = ask_float(f'มุมที่หุ่นหมุนจริง (องศา เช่น 3 รอบพอดี = 1080) [Enter = ข้าม]: ', args.measured)
    if real:
        L = cfg['LR_WHEELS_DISTANCE'] * odeg / real
        print(f'→ LR_WHEELS_DISTANCE ใหม่ = {cfg["LR_WHEELS_DISTANCE"]} × {odeg:.1f} ÷ {real:.1f} = {L:.4f}')
        print(f'   #define LR_WHEELS_DISTANCE  {L:.4f}')
        print('   หมายเหตุ: ทิศหลักของหุ่นมาจาก gyro (odom_tf_relay) ค่านี้มีผลกับ /odom/unfiltered และ PID การเลี้ยว')


def mode_step(n, cfg, args):
    sp = max(-MAX_LIN, min(MAX_LIN, args.speed))
    n.reset()
    t0 = time.time()
    while time.time() - t0 < args.time:
        n.send(sp, 0.0)
        n.spin_for(0.05)
    n.stop(); n.spin_for(1.0)
    rows = [(t - t0, vx, wz) for t, vx, wz in n.samples]
    out = os.path.expanduser(args.csv or f'~/calib_step_{time.strftime("%Y%m%d_%H%M%S")}.csv')
    with open(out, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t_s', 'cmd_vx', 'odom_vx', 'odom_wz'])
        for t, vx, wz in rows:
            w.writerow([f'{t:.3f}', sp if t <= args.time else 0.0, f'{vx:.4f}', f'{wz:.4f}'])
    run = [vx for t, vx, _ in rows if t <= args.time]
    if not run:
        sys.exit('❌ ไม่มีข้อมูล odom ระหว่างทดสอบ')
    rise = next((t for t, vx, _ in rows if abs(vx) >= 0.9 * abs(sp)), None)
    tail = [vx for t, vx, _ in rows if args.time * 0.5 <= t <= args.time] or run
    mean = sum(tail) / len(tail)
    std = (sum((v - mean) ** 2 for v in tail) / len(tail)) ** 0.5
    peak = max(run, key=abs)
    print(f'ข้อมูล {len(rows)} จุด → {out}')
    print(f'เวลาขึ้นถึง 90%: {rise:.2f} s' if rise is not None else 'เวลาขึ้นถึง 90%: ไม่ถึง')
    print(f'ความเร็วช่วงหลัง: เฉลี่ย {mean:.3f} m/s (สั่ง {sp:.3f}) · คลาด {100 * (mean - sp) / sp:+.1f}% · แกว่ง ±{std:.3f}')
    print(f'ค่าสูงสุด {peak:.3f} m/s · overshoot {max(0.0, 100 * (abs(peak) - abs(sp)) / abs(sp)):.0f}%')
    print('แนวทาง: แกว่ง/overshoot มาก → ลด K_P · ขึ้นช้า/ไม่ถึงค่าสั่ง → เพิ่ม K_I · สั่นถี่ → ลด K_D')


def main():
    ap = argparse.ArgumentParser(description='ทดสอบจูนฐานล้อ LotusRosDev_Bot')
    ap.add_argument('mode', choices=['watch', 'straight', 'rotate', 'step'])
    ap.add_argument('--dist', type=float, default=1.0, help='straight: ระยะ (m) ติดลบ = ถอย')
    ap.add_argument('--turns', type=float, default=3.0, help='rotate: จำนวนรอบ บวก = ทวนเข็ม')
    ap.add_argument('--speed', type=float, default=None, help='m/s (straight/step) หรือ rad/s (rotate)')
    ap.add_argument('--time', type=float, default=3.0, help='step: เวลาสั่ง (s)')
    ap.add_argument('--measured', type=float, default=None, help='ค่าที่วัดจริง (ข้ามการถาม)')
    ap.add_argument('--lateral', type=float, default=None, help='straight: เบี้ยว cm (ซ้าย +)')
    ap.add_argument('--csv', default=None, help='step: ไฟล์ CSV ปลายทาง')
    ap.add_argument('--odom', default='/odom/unfiltered', help='topic odom จากล้อ')
    ap.add_argument('--config', default=CONFIG, help='ไฟล์ lotusrosbot_config.h')
    ap.add_argument('--yes', action='store_true', help='ข้ามการยืนยัน (ต้องมีคนเฝ้าอยู่แล้ว)')
    args = ap.parse_args()
    if args.speed is None:
        args.speed = {'straight': 0.10, 'rotate': 0.6, 'step': 0.15}.get(args.mode, 0.1)
    cfg = read_config(args.config)
    print('ค่าปัจจุบัน:', ', '.join(f'{k}={v:g}' for k, v in cfg.items()))
    rclpy.init()
    n = Calib(args.odom)
    try:
        preflight(n, args)
        {'watch': mode_watch, 'straight': mode_straight, 'rotate': mode_rotate, 'step': mode_step}[args.mode](n, cfg, args)
    except KeyboardInterrupt:
        print('\n🛑 หยุด (Ctrl+C)')
    finally:
        n.stop()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""T2 มุมไหลตอนจอดนิ่ง — วัด yaw ของ odom ตลอด N วินาที (หุ่นไม่ขยับ)"""
import sys, time
from common import *

sec = int(sys.argv[1]) if len(sys.argv) > 1 else 120
n = node('t2_drift'); w = Watch(n)
if not w.wait_ready(10):
    result(ok=False, msg='ไม่ได้รับ /odom'); raise SystemExit
y0 = w.pose()[2]; t0 = time.time(); worst = 0.0
while time.time() - t0 < sec:
    w.spin(10)
    d = wrap_deg(w.pose()[2] - y0); worst = max(worst, abs(d))
    say(f'{time.time() - t0:5.0f} วิ · ไหล {d:+.2f}°')
d = wrap_deg(w.pose()[2] - y0)
result(ok=worst <= 1.0, drift_deg=round(d, 2), worst_deg=round(worst, 2), sec=sec,
       per_min=round(d / (sec / 60), 3))

"""T4 / T5 สี่เหลี่ยม — ด้าน side m · ความเร็ว v m/s · laps รอบ (เลี้ยวซ้าย 90°)
   วัด: odom กลับถึงจุดเริ่มไหม + ทิศเพี้ยนจริงด้วย LiDAR (เทียบ scan ตอนเริ่มกับตอนจบ)"""
import math, sys, time
from common import *
import lotus_motion as lm

side = float(sys.argv[1]); v = float(sys.argv[2]); laps = int(sys.argv[3]); wz = float(sys.argv[4]) if len(sys.argv) > 4 else 0.6
lm.MAX_LIN = max(lm.MAX_LIN, min(v, 0.35))                # ปลดเพดาน 0.25 เฉพาะการทดสอบนี้ (ไม่แก้ไฟล์)
rclpy.init()
n = lm.Mover('t_square', False); w = Watch(n)
if not w.wait_ready(10):
    result(ok=False, msg='ไม่ได้รับ /odom หรือ /scan'); lm.finish(n); raise SystemExit
w.spin(1.0)
s0, inc = w.scan_avg(8); x0, y0, h0 = w.pose()
laps_out = []
try:
    for lap in range(1, laps + 1):
        t = time.time()
        for k in range(4):
            n.led(True); n.beep(1200, 60)
            n.forward(side, speed=v)
            n.led(False)
            n.turn(90, speed=wz)
        n.wait(0.8)
        x, y, h = w.pose()
        d = math.hypot(x - x0, y - y0); dh = wrap_deg(h - h0)
        laps_out.append({'lap': lap, 'sec': round(time.time() - t, 1), 'odom_dist_m': round(d, 3), 'odom_head_deg': round(dh, 1)})
        say(f'รอบ {lap}: {time.time() - t:.1f} วิ · odom ห่างจุดเริ่ม {d:.3f} m · ทิศ {dh:+.1f}°')
    n.stop(); w.spin(1.0)
    s1, _ = w.scan_avg(8)                                  # ภาพ LiDAR ตอนจบ (ก่อนปิด node)
finally:
    lm.finish(n)
lidar, cost = scan_rotation(s0, s1, inc, search_deg=90.0)
if lidar is not None:
    say(f'LiDAR: ทิศจริงตอนจบเพี้ยน {lidar:+.1f}° จากตอนเริ่ม (odom บอก {laps_out[-1]["odom_head_deg"]:+.1f}°)')
per_lap = None if lidar is None else round(lidar / laps, 1)
result(ok=lidar is not None and abs(lidar) / laps <= 8.0, laps=laps_out, lidar_head_deg=None if lidar is None else round(lidar, 1),
       lidar_head_per_lap=per_lap, side=side, v=v, n=laps)

"""T3 หมุน 360° ซ้าย แล้ว 360° ขวา — วัดมุมคลาดจริงด้วย LiDAR (เทียบ scan ก่อน/หลัง) เทียบกับที่ odom บอก"""
import sys, time
from common import *
import lotus_motion as lm

speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6
rclpy.init()
n = lm.Mover('t3_rotate', False); w = Watch(n)
if not w.wait_ready(10):
    result(ok=False, msg='ไม่ได้รับ /odom หรือ /scan'); lm.finish(n); raise SystemExit
out = {}
try:
    for name, deg in (('ซ้าย', 360.0), ('ขวา', -360.0)):
        w.spin(1.0)
        s0, inc = w.scan_avg(8); y0 = w.pose()[2]
        say(f'หมุน{name} {deg:+.0f}° ที่ {speed} rad/s…')
        n.turn(deg, speed=speed)
        n.wait(1.5)
        s1, _ = w.scan_avg(8); y1 = w.pose()[2]
        lidar, cost = scan_rotation(s0, s1, inc)          # มุมเกินจาก 360 (เพราะ 360 = ภาพเดิม)
        odom_err = wrap_deg(y1 - y0)                      # odom บอกว่าเกิน/ขาดเท่าไร
        out[name] = {'lidar_err_deg': None if lidar is None else round(lidar, 1),
                     'odom_err_deg': round(odom_err, 1), 'match_cost': None if cost is None else round(cost, 3)}
        say(f'  LiDAR: หุ่นหมุนเกิน {lidar:+.1f}° · odom บอก {odom_err:+.1f}°' if lidar is not None else '  LiDAR เทียบภาพไม่ได้')
finally:
    lm.finish(n)
errs = [abs(v['lidar_err_deg']) for v in out.values() if v['lidar_err_deg'] is not None]
result(ok=bool(errs) and max(errs) <= 5.0, **out, max_lidar_err=max(errs) if errs else None)

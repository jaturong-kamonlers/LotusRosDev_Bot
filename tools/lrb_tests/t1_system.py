"""T1 ระบบ — node ของบอร์ด, ความถี่ odom/imu/scan, relay ใช้ yaw จากบอร์ด (หุ่นไม่ขยับ)"""
import subprocess, time
from common import *

n = node('t1_system'); w = Watch(n)
if not w.wait_ready(10):
    result(ok=False, msg='ไม่ได้รับ /odom หรือ /scan — เปิดระบบ ROS2 ก่อน'); raise SystemExit
w.n = {k: 0 for k in w.n}; t0 = time.time(); w.spin(8); dt = time.time() - t0
hz = {k: round(v / dt, 2) for k, v in w.n.items()}
nodes = subprocess.run(['bash', '-c', 'source ~/ros_env.sh >/dev/null 2>&1; ros2 node list'], capture_output=True, text=True).stdout.split()
tag = w.imu is not None and abs(w.imu.orientation_covariance[8] - 0.0004) < 1e-6
checks = {
    'node /lotusrosbot_base': '/lotusrosbot_base' in nodes,
    'odom ≥ 2 Hz': hz['odom'] >= 2.0,
    'imu ≥ 2 Hz': hz['imu'] >= 2.0,
    'scan ≥ 8 Hz': hz['scan'] >= 8.0,
    'yaw จากบอร์ด (ป้าย cov[8])': tag,
}
for k, v in checks.items():
    say(('✅ ' if v else '❌ ') + k)
say(f'ความถี่: {hz}')
result(ok=all(checks.values()), hz=hz, checks=checks, nodes=nodes)

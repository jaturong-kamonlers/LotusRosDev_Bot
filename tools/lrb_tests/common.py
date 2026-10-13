"""
common.py — ตัวช่วยของชุดทดสอบ T (รันบน Jetson ขณะเปิด agent + bringup อยู่)
  · อ่าน /odom (ตำแหน่ง + ทิศ) และ /scan (LiDAR)
  · วัดมุมที่หุ่นหมุนจริงด้วย LiDAR: เทียบภาพ scan ก่อน-หลัง หาเลื่อนมุมที่ตรงกันที่สุด (ไม่พึ่ง odom)
  · ทุกสคริปต์พิมพ์บรรทัดสุดท้ายเป็น  RESULT {json}  ให้หน้าเว็บอ่าน
"""
import json, math, os, sys, time
sys.path.insert(0, os.path.expanduser('~/lotusbot/scripts/example'))
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def say(text):
    print(text, flush=True)


def result(**kw):
    print('RESULT ' + json.dumps(kw, ensure_ascii=False, default=lambda o: o.item() if hasattr(o, 'item') else str(o)), flush=True)


class Watch:
    """สมัครรับ /odom /scan /imu/data แล้วเก็บค่าล่าสุด"""

    def __init__(self, node):
        self.node = node
        self.odom = None; self.scan = None; self.imu = None
        self.n = {'odom': 0, 'scan': 0, 'imu': 0}
        node.create_subscription(Odometry, '/odom', self._o, 10)
        node.create_subscription(LaserScan, '/scan', self._s, qos_profile_sensor_data)
        node.create_subscription(Imu, '/imu/data', self._i, 10)

    def _o(self, m): self.odom = m; self.n['odom'] += 1
    def _s(self, m): self.scan = m; self.n['scan'] += 1
    def _i(self, m): self.imu = m; self.n['imu'] += 1

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_ready(self, sec=8):
        t0 = time.time()
        while time.time() - t0 < sec and (self.odom is None or self.scan is None):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.odom is not None and self.scan is not None

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, math.degrees(yaw_of(p.orientation))

    def scan_avg(self, n=8):
        """เฉลี่ย scan n รอบ (หุ่นต้องนิ่ง) → list ระยะ (None = ไม่มีค่า)"""
        acc = None; cnt = None; got = 0; last = -1
        t0 = time.time()
        while got < n and time.time() - t0 < 5:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.scan is not None and self.n['scan'] != last:
                last = self.n['scan']
                r = self.scan.ranges
                if acc is None:
                    acc = [0.0] * len(r); cnt = [0] * len(r)
                if len(r) != len(acc):
                    continue
                for i, v in enumerate(r):
                    if math.isfinite(v) and 0.15 < v < 8.0:
                        acc[i] += v; cnt[i] += 1
                got += 1
        if acc is None:
            return None, 0
        inc = math.degrees(self.scan.angle_increment)
        return [acc[i] / cnt[i] if cnt[i] >= n // 2 else None for i in range(len(acc))], inc


def scan_rotation(before, after, inc_deg, search_deg=45.0):
    """มุมที่หุ่นหมุนไปจริง (องศา, บวก = ทวนเข็ม) จากการเลื่อน scan ให้ตรงกันที่สุด
       หุ่นหมุน +θ → วัตถุเดิมไปปรากฏที่มุม −θ ใน scan ใหม่"""
    n = len(before)
    K = int(search_deg / inc_deg)
    best = None
    costs = {}
    for k in range(-K, K + 1):
        s = 0.0; c = 0
        for i in range(n):
            a = before[i]; b = after[(i - k) % n]
            if a is not None and b is not None:
                s += min(abs(a - b), 0.5); c += 1
        if c > n * 0.25:
            costs[k] = s / c
            if best is None or costs[k] < costs[best]:
                best = k
    if best is None:
        return None, None
    # ละเอียดกว่า 1 ช่องด้วยพาราโบลา
    frac = 0.0
    if best - 1 in costs and best + 1 in costs:
        y0, y1, y2 = costs[best - 1], costs[best], costs[best + 1]
        d = y0 - 2 * y1 + y2
        if d > 0:
            frac = 0.5 * (y0 - y2) / d
    return (best + frac) * inc_deg, costs[best]


def node(name):
    rclpy.init()
    return rclpy.create_node(name)

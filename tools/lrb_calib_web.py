#!/usr/bin/env python3
"""
lrb_calib_web.py — หน้าเว็บคาลิเบรต + จูน PID ของบอร์ด ESP32 (เฟิร์มแวร์ LotusRosBot v2026.10.14c ขึ้นไป)

รันที่ 🟡 Jetson:   python3 ~/LotusRosDev_Bot/tools/lrb_calib_web.py
เปิดเบราว์เซอร์:     http://nano.local:8091   (หรือ http://<IP ของ Jetson>:8091)

ทำงานอย่างไร:
  · ใช้สาย USB เส้นเดียวกับ micro-ROS → ต้องปิด agent / ภารกิจก่อน (bash ~/stop.sh all)
  · กด "เชื่อมต่อ" → รีเซ็ต ESP32 แล้วส่ง @CALIB ระหว่างนับถอยหลัง → บอร์ดเข้าโหมดคาลิเบรตเอง (ไม่ต้องกดปุ่ม)
  · คำสั่งทั้งหมดเป็นบรรทัดข้อความ (ดูรายการใน LotusRosBot/firmware/src/calib/calib_remote.cpp)
  · "ตัดการเชื่อมต่อ" → บอร์ดรีบูตเข้าโหมด ROS2 ตามปกติ แล้วค่อยเปิด agent

ความปลอดภัย: ต้องติ๊ก "ล้อลอย + มีคนเฝ้า" ก่อนหมุนมอเตอร์ · มอเตอร์หยุดเองถ้าเว็บเงียบเกิน 0.3 วินาที
             · กดปุ่ม BTN บนบอร์ด = หยุดทุกอย่างทันที
"""
import json, os, re, signal, subprocess, sys, threading, time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

try:
    import serial
except ImportError:
    sys.exit('ต้องมี pyserial: sudo apt install python3-serial')

PORT_DEV = os.environ.get('LRB_PORT', '/dev/esp32')
HTTP_PORT = int(os.environ.get('LRB_WEB_PORT', '8091'))
HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, 'lrb_calib_web.html')

# คำสั่งที่หน้าเว็บส่งได้ (กันส่งอย่างอื่นเข้าบอร์ด)
ALLOWED = re.compile(r'^@(PING|STATUS|MOTOR -?\d{1,4} -?\d{1,4}|STOP|ENCZERO|LED [01]|BEEP \d{2,5} \d{1,4}|'
                     r'SERVO \d{1,3} \d{1,3} \d{1,3}|SERVOOFF|PIDGET|PIDSET [LRB] [\d.]+ [\d.]+ [\d.]+|'
                     r'PIDSAVE|PIDRESET|NETGET|EXIT)$')


class Link:
    """สายเชื่อม ESP32: เปิดพอร์ต · ส่งคำสั่ง · อ่านคำตอบ (thread อ่านแยก)"""

    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()          # ส่งคำสั่งทีละคำสั่ง
        self.cv = threading.Condition()
        self.lines = []                       # คำตอบที่ยังไม่มีใครรับ
        self.step = None                      # ข้อมูล @STEP ที่กำลังรับ
        self.state = {'connected': False, 'msg': 'ยังไม่เชื่อมต่อ', 'fw': '', 'status': {}, 'pid': None,
                      'net': None, 'busy': False, 'log': []}
        self.reader = None

    # ── บันทึกเหตุการณ์ (แสดงในหน้าเว็บ) ──
    def log(self, text):
        self.state['log'] = (self.state['log'] + [time.strftime('%H:%M:%S ') + text])[-40:]

    # ── ตรวจว่าพอร์ตว่าง ──
    def port_owner(self):
        real = os.path.realpath(PORT_DEV)
        if not os.path.exists(real):
            return f'ไม่พบ {PORT_DEV} — เสียบสาย USB ของ ESP32'
        if subprocess.run(['pgrep', '-f', 'micro_ros_agent serial'], capture_output=True).returncode == 0:
            return 'micro-ROS agent เปิดอยู่ — ปิดก่อน: bash ~/stop.sh all'
        r = subprocess.run(['fuser', real], capture_output=True, text=True)
        if r.returncode == 0:
            return f'มีโปรแกรมอื่นใช้พอร์ตอยู่ (pid {r.stdout.strip()})'
        return ''

    # ── อ่านบรรทัดจากบอร์ด ──
    def _read_loop(self):
        buf = b''
        while self.ser is not None:
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                self.log(f'อ่านพอร์ตไม่ได้: {e}')
                break
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                line = raw.decode('utf-8', 'replace').strip()
                if line:
                    self._on_line(line)
        self.state['connected'] = False

    def _on_line(self, line):
        if line.startswith('S ') and self.step is not None:
            p = line.split()
            if len(p) == 7:
                try:
                    self.step['t'].append(int(p[1])); self.step['tgt'].append(float(p[2]))
                    self.step['rpm1'].append(float(p[3])); self.step['pwm1'].append(int(p[4]))
                    self.step['rpm2'].append(float(p[5])); self.step['pwm2'].append(int(p[6]))
                except ValueError:
                    pass
            return
        if line.startswith('ST '):
            st = {}
            for kv in line[3:].split():
                k, _, v = kv.partition('=')
                try:
                    st[k] = float(v)
                except ValueError:
                    st[k] = v
            st['t'] = time.time()
            self.state['status'] = st
        elif line.startswith('PID '):
            p = line.split()
            if len(p) >= 8:
                g = [float(x) for x in p[1:7]]
                self.state['pid'] = {'L': g[0:3], 'R': g[3:6], 'src': p[7]}
        elif line.startswith('NETINFO '):
            f = line[8:].split('|')
            if len(f) >= 5:
                self.state['net'] = {'host': f[0], 'ssid': f[1], 'ip': f[2], 'user': f[3], 'live': f[4] == '1'}
        elif line.startswith('EVT '):
            self.log('บอร์ด: ' + line[4:])
        elif line.startswith('OK ') and 'CALIB' in line:
            self.state['fw'] = ' '.join(line.split()[2:4])
        with self.cv:
            self.lines.append(line)
            self.lines = self.lines[-100:]
            self.cv.notify_all()

    def _wait(self, prefixes, timeout):
        end = time.time() + timeout
        with self.cv:
            while True:
                for i, l in enumerate(self.lines):
                    if any(l.startswith(p) for p in prefixes):
                        del self.lines[:i + 1]
                        return l
                left = end - time.time()
                if left <= 0:
                    return None
                self.cv.wait(left)

    def _send(self, text):
        self.ser.write((text + '\n').encode())
        self.ser.flush()

    def command(self, text, expect=('OK', 'ERR', 'ST', 'PID', 'NETINFO'), timeout=1.0):
        if not self.state['connected']:
            return None
        with self.lock:
            with self.cv:
                self.lines.clear()
            self._send(text)
            return self._wait(expect, timeout)

    # ── เชื่อมต่อ: รีเซ็ตบอร์ด → ส่ง @CALIB ระหว่างนับถอยหลัง ──
    def connect(self):
        if self.state['connected']:
            return True, 'เชื่อมต่ออยู่แล้ว'
        why = self.port_owner()
        if why:
            return False, why
        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout, s.write_timeout = PORT_DEV, 115200, 0.1, 1.0
            s.open()
        except Exception as e:
            return False, f'เปิดพอร์ตไม่ได้: {e}'
        self.ser = s
        self.state['connected'] = True
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        if self.command('@PING', ('OK PING',), 0.6):          # อยู่ในโหมดคาลิเบรตแล้ว
            self.log('บอร์ดอยู่ในโหมดคาลิเบรตอยู่แล้ว')
        else:
            self.log('รีเซ็ต ESP32 แล้วขอเข้าโหมดคาลิเบรต…')
            with self.lock:
                s.dtr = False; s.rts = True                   # EN ต่ำ = รีเซ็ต
                time.sleep(0.15)
                s.rts = False                                 # ปล่อย EN → บูตปกติ (IO0 สูง)
                ok = False
                end = time.time() + 9.0
                while time.time() < end and not ok:
                    self._send('@CALIB')
                    ok = self._wait(('OK CALIB',), 0.15) is not None
            if not ok:
                self.disconnect(reboot=False)
                return False, 'บอร์ดไม่ตอบ @CALIB — เฟิร์มแวร์เก่ากว่า v2026.10.14c? (flash ใหม่ด้วย tools/flash.sh upload)'
        self.command('@PIDGET', ('PID',))
        self.command('@NETGET', ('NETINFO',))
        self.state['msg'] = 'เชื่อมต่อแล้ว — โหมดคาลิเบรต (จอ OLED ขึ้น WEB CONTROL)'
        self.log('เชื่อมต่อแล้ว ' + self.state['fw'])
        return True, self.state['msg']

    def disconnect(self, reboot=True):
        if self.ser is not None:
            try:
                if reboot and self.state['connected']:
                    with self.lock:
                        self._send('@STOP'); time.sleep(0.05)
                        self._send('@REBOOT')             # บอร์ดรีบูต → ไม่มีใครส่ง @CALIB → เข้า ROS2 เอง
                        time.sleep(0.3)
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.state.update(connected=False, msg='ตัดการเชื่อมต่อแล้ว — บอร์ดรีบูตเข้าโหมด ROS2 (เปิด agent ต่อได้)',
                          status={})
        self.log('ตัดการเชื่อมต่อ')

    # ── ทดสอบ step ──
    def run_step(self, side, rpm, hold):
        if not self.state['connected']:
            return {'ok': False, 'msg': 'ยังไม่เชื่อมต่อ'}
        with self.lock:
            self.state['busy'] = True
            self.step = {'t': [], 'tgt': [], 'rpm1': [], 'pwm1': [], 'rpm2': [], 'pwm2': []}
            with self.cv:
                self.lines.clear()
            self._send(f'@STEP {side} {rpm:.1f} {int(hold)}')
            end_line = self._wait(('END STEP', 'ERR'), hold / 1000.0 + 4.0)
            data, self.step = self.step, None
            self.state['busy'] = False
        if end_line is None:
            return {'ok': False, 'msg': 'บอร์ดไม่ตอบ (หมดเวลา)', 'data': data}
        if end_line.startswith('ERR'):
            return {'ok': False, 'msg': end_line, 'data': data}
        # encoder ละเอียด ~3 rpm ต่อ tick ในช่วง 20 ms → เฉลี่ย 5 จุด (100 ms) ก่อนคำนวณ/วาด
        data['rpm1s'] = smooth(data['rpm1']); data['rpm2s'] = smooth(data['rpm2'])
        data['metrics'] = {
            'L': metrics(data['t'], data['tgt'], data['rpm1s']) if side in 'LB' else None,
            'R': metrics(data['t'], data['tgt'], data['rpm2s']) if side in 'RB' else None,
        }
        aborted = end_line.endswith('abort')
        self.log(f'STEP {side} {rpm:.0f} rpm {hold} ms → {"หยุดกลางทาง" if aborted else "เสร็จ"} ({len(data["t"])} จุด)')
        return {'ok': not aborted, 'msg': 'ถูกหยุดกลางทาง (กด BTN / หยุด)' if aborted else 'เสร็จ', 'data': data,
                'side': side, 'rpm': rpm, 'hold': hold, 'pid': self.state['pid']}


def smooth(v, n=5):
    """ค่าเฉลี่ยเคลื่อนที่ย้อนหลัง n จุด"""
    out = []
    for i in range(len(v)):
        w = v[max(0, i - n + 1):i + 1]
        out.append(sum(w) / len(w))
    return out


def metrics(t, tgt, rpm):
    """rise 0→90% · overshoot · ความผิดพลาดช่วงท้าย · settling ±5%"""
    idx = [i for i, g in enumerate(tgt) if g != 0]
    if not idx:
        return None
    target = tgt[idx[0]]; t0 = t[idx[0]]; t_end = t[idx[-1]]
    frac = [(t[i], rpm[i] / target) for i in idx]
    t90 = next((ti for ti, v in frac if v >= 0.9), None)
    peak = max(v for _, v in frac)
    tail = [v for ti, v in frac if ti >= t_end - 400]
    ss = (1 - sum(tail) / len(tail)) * 100 if tail else None
    settle = None
    for k in range(len(frac)):
        if all(abs(v - 1) <= 0.05 for _, v in frac[k:]):
            settle = frac[k][0] - t0
            break
    ripple = (max(tail) - min(tail)) * 100 if tail else None
    return {'rise_ms': None if t90 is None else t90 - t0, 'overshoot_pct': max(0.0, (peak - 1) * 100),
            'ss_err_pct': ss, 'settle_ms': settle, 'ripple_pct': ripple}


LINK = Link()


def poller():
    """อ่านสถานะสดทุก 0.3 วินาที (หยุดระหว่างทดสอบ step)"""
    n = 0
    while True:
        time.sleep(0.3)
        if LINK.state['connected'] and not LINK.state['busy']:
            if LINK.lock.acquire(timeout=0.05):
                LINK.lock.release()
                LINK.command('@STATUS', ('ST',), 0.5)
                n += 1
                if n % 10 == 0:
                    LINK.command('@NETGET', ('NETINFO',), 0.5)


class Handler(BaseHTTPRequestHandler):
    def _reply(self, code, obj, ctype='application/json; charset=utf-8'):
        body = obj if isinstance(obj, (bytes, bytearray)) else json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass

    def do_GET(self):
        path = self.path.split('?')[0]
        if path in ('/', '/index.html'):
            try:
                with open(PAGE, 'rb') as f:
                    return self._reply(200, f.read(), 'text/html; charset=utf-8')
            except OSError:
                return self._reply(500, {'msg': f'ไม่พบ {PAGE}'})
        if path == '/api/state':
            st = dict(LINK.state)
            st['port_owner'] = '' if LINK.state['connected'] else LINK.port_owner()
            return self._reply(200, st)
        self._reply(404, {'msg': 'ไม่พบ'})

    def do_POST(self):
        try:
            n = min(int(self.headers.get('Content-Length', 0)), 2048)
            d = json.loads(self.rfile.read(n) or b'{}')
        except (ValueError, TypeError):
            return self._reply(400, {'msg': 'ข้อมูลไม่ถูกต้อง'})
        path = self.path.split('?')[0]
        if path == '/api/connect':
            ok, msg = LINK.connect()
            LINK.state['msg'] = msg
            return self._reply(200, {'ok': ok, 'msg': msg})
        if path == '/api/disconnect':
            LINK.disconnect(reboot=True)
            return self._reply(200, {'ok': True, 'msg': LINK.state['msg']})
        if path == '/api/cmd':
            line = str(d.get('line', '')).strip()
            if not ALLOWED.match(line):
                return self._reply(400, {'ok': False, 'msg': 'คำสั่งไม่อยู่ในรายการที่อนุญาต'})
            if LINK.state['busy']:
                return self._reply(409, {'ok': False, 'msg': 'กำลังทดสอบ step อยู่'})
            r = LINK.command(line)
            return self._reply(200, {'ok': r is not None and not r.startswith('ERR'), 'reply': r,
                                     'pid': LINK.state['pid']})
        if path == '/api/step':
            side = str(d.get('side', 'B'))[:1].upper()
            try:
                rpm = max(-95.0, min(95.0, float(d.get('rpm', 60))))
                hold = max(300, min(5000, int(d.get('hold', 2000))))
            except (TypeError, ValueError):
                return self._reply(400, {'ok': False, 'msg': 'ค่าไม่ถูกต้อง'})
            if side not in 'LRB':
                return self._reply(400, {'ok': False, 'msg': 'side ต้องเป็น L R หรือ B'})
            return self._reply(200, LINK.run_step(side, rpm, hold))
        self._reply(404, {'msg': 'ไม่พบ'})


def _stop(*_):
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTERM, _stop)     # kill / systemctl stop → ตัดการเชื่อมต่อให้บอร์ดกลับโหมด ROS2
    threading.Thread(target=poller, daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    print(f'🔧 LotusRosBot calib web: http://nano.local:{HTTP_PORT}  (หรือ http://<IP ของ Jetson>:{HTTP_PORT})')
    print('   ปิดด้วย Ctrl+C (บอร์ดจะรีบูตเข้าโหมด ROS2 ให้เอง)')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        LINK.disconnect(reboot=True)


if __name__ == '__main__':
    main()

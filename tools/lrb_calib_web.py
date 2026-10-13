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


# ════════════════════════════════════════════════════════════════
#  ค่าหุ่น (lotusrosbot_config.h) — อ่าน/แก้จากหน้าเว็บ
# ════════════════════════════════════════════════════════════════
REPO = os.path.dirname(HERE)                                   # ~/LotusRosDev_Bot
CFG = os.path.join(REPO, 'LotusRosBot', 'firmware', 'include', 'lotusrosbot_config.h')
FLASH = os.path.join(REPO, 'tools', 'flash.sh')
BACKUP_DIR = os.path.expanduser('~/fw_backup')

# ชื่อ: (กลุ่ม, ชนิด, ต่ำสุด, สูงสุด, คำอธิบาย)
SPEC = {
    'K_P_L': ('PID ล้อซ้าย', 'float', 0, 50, 'Kp — ดันตามความผิดพลาดตอนนี้'),
    'K_I_L': ('PID ล้อซ้าย', 'float', 0, 50, 'Ki — สะสมจนถึงเป้า (ตัวหลักที่ทำให้เร็ว)'),
    'K_D_L': ('PID ล้อซ้าย', 'float', 0, 50, 'Kd — เบรกการเปลี่ยนเร็ว'),
    'K_P_R': ('PID ล้อขวา', 'float', 0, 50, 'Kp'),
    'K_I_R': ('PID ล้อขวา', 'float', 0, 50, 'Ki'),
    'K_D_R': ('PID ล้อขวา', 'float', 0, 50, 'Kd'),
    'COUNTS_PER_REV1': ('Encoder', 'int', 10, 20000, 'tick ต่อรอบ ล้อซ้าย (เมนู 7)'),
    'COUNTS_PER_REV2': ('Encoder', 'int', 10, 20000, 'tick ต่อรอบ ล้อขวา (เมนู 7)'),
    'MOTOR1_ENCODER_INV': ('Encoder', 'bool', 0, 1, 'กลับทิศนับ encoder ซ้าย'),
    'MOTOR2_ENCODER_INV': ('Encoder', 'bool', 0, 1, 'กลับทิศนับ encoder ขวา'),
    'WHEEL_DIAMETER': ('ขนาดล้อ', 'float', 0.02, 0.30, 'เส้นผ่านศูนย์กลางล้อ (m) — มีผลกับระยะทาง'),
    'LR_WHEELS_DISTANCE': ('ขนาดล้อ', 'float', 0.05, 1.0, 'ระยะกึ่งกลางล้อซ้าย-ขวา (m) — มีผลกับการหมุน'),
    'MOTOR_MAX_RPM': ('มอเตอร์', 'int', 10, 1000, 'RPM สูงสุดตามสเปก'),
    'MAX_RPM_RATIO_L': ('มอเตอร์', 'float', 0.1, 1.0, 'ใช้ได้ไม่เกินกี่ส่วนของ RPM สูงสุด (ซ้าย)'),
    'MAX_RPM_RATIO_R': ('มอเตอร์', 'float', 0.1, 1.0, '(ขวา)'),
    'MOTOR_OPERATING_VOLTAGE': ('มอเตอร์', 'float', 1, 48, 'แรงดันตามสเปกมอเตอร์ (V)'),
    'MOTOR_POWER_MAX_VOLTAGE': ('มอเตอร์', 'float', 1, 48, 'แรงดันที่จ่ายจริง (V)'),
    'MOTOR_POWER_MEASURED_VOLTAGE': ('มอเตอร์', 'float', 0, 48, 'แรงดันที่วัดได้ (V) — ใช้ในเมนู 2–3'),
    'MOTOR1_INV': ('มอเตอร์', 'bool', 0, 1, 'กลับทิศมอเตอร์ซ้าย'),
    'MOTOR2_INV': ('มอเตอร์', 'bool', 0, 1, 'กลับทิศมอเตอร์ขวา'),
}
TOGGLES = {'USE_BUZZER': 'เสียง buzzer', 'USE_SERVO': 'servo แขนกล', 'USE_IR_SENSORS': 'เซนเซอร์ IR',
           'USE_POTENTIOMETER': 'potentiometer', 'USE_BUTTON': 'ปุ่ม BTN'}
READONLY = ['BAUDRATE', 'NODE_NAME', 'LED_PIN', 'MOTOR1_PWM', 'MOTOR1_IN_A', 'MOTOR1_IN_B', 'MOTOR2_PWM',
            'MOTOR2_IN_A', 'MOTOR2_IN_B', 'MOTOR1_ENCODER_A', 'MOTOR1_ENCODER_B', 'MOTOR2_ENCODER_A',
            'MOTOR2_ENCODER_B', 'BUZZER_PIN', 'SERVO1_PIN', 'SERVO2_PIN', 'SERVO3_PIN', 'IR_LEFT_PIN',
            'IR_RIGHT_PIN', 'POT_PIN', 'BUTTON_PIN', 'SDA_PIN', 'SCL_PIN']
DEF_RE = re.compile(r'^(\s*)(//\s*)?#define\s+(\w+)(\s+)([^/\n]*?)(\s*)(//.*)?$')


def cfg_read():
    with open(CFG, encoding='utf-8') as f:
        lines = f.read().split('\n')
    vals, toggles, ro = {}, {}, {}
    for l in lines:
        m = DEF_RE.match(l)
        if not m:
            continue
        commented, name, val = bool(m.group(2)), m.group(3), m.group(5).strip()
        if name in TOGGLES:
            toggles[name] = not commented
        elif not commented and name in SPEC:
            vals[name] = val
        elif not commented and name in READONLY:
            ro[name] = val
    items = [{'name': k, 'group': g, 'type': t, 'min': lo, 'max': hi, 'desc': d, 'value': vals.get(k)}
             for k, (g, t, lo, hi, d) in SPEC.items()]
    return {'items': items, 'toggles': [{'name': k, 'desc': d, 'on': toggles.get(k, False)} for k, d in TOGGLES.items()],
            'readonly': ro, 'path': CFG}


def cfg_validate(new):
    """คืน (ค่าที่จะเขียน {name: text}, toggles {name: bool}, ข้อผิดพลาด [])"""
    out, tog, err = {}, {}, []
    for k, v in (new.get('values') or {}).items():
        if k not in SPEC:
            err.append(f'{k}: แก้จากหน้าเว็บไม่ได้'); continue
        g, t, lo, hi, d = SPEC[k]
        s = str(v).strip().lower()
        if t == 'bool':
            if s not in ('true', 'false'):
                err.append(f'{k}: ต้องเป็น true/false'); continue
            out[k] = s
        else:
            try:
                x = float(s)
            except ValueError:
                err.append(f'{k}: ไม่ใช่ตัวเลข'); continue
            if not (lo <= x <= hi):
                err.append(f'{k}: ต้องอยู่ระหว่าง {lo}–{hi}'); continue
            if t == 'int':
                out[k] = str(int(round(x)))
            else:
                f4 = ('%.4f' % x).rstrip('0')
                out[k] = f4 + '0' if f4.endswith('.') else f4
    for k, on in (new.get('toggles') or {}).items():
        if k in TOGGLES:
            tog[k] = bool(on)
    return out, tog, err


def cfg_apply(text, out, tog):
    lines = text.split('\n')
    changes = []
    for i, l in enumerate(lines):
        m = DEF_RE.match(l)
        if not m:
            continue
        ind, com, name, sp1, val, sp2, cmt = m.groups()
        if name in out and not com and val.strip() != out[name]:
            newv = out[name]
            pad = max(1, len(val) + len(sp2) - len(newv)) if cmt else 0
            lines[i] = f'{ind}#define {name}{sp1}{newv}' + ((' ' * pad + cmt) if cmt else '')
            changes.append(f'{name} {val.strip()}→{newv}')
        if name in tog:
            on_now = not com
            if tog[name] != on_now:
                lines[i] = (l.replace('// #define', '#define', 1).replace('//#define', '#define', 1) if tog[name]
                            else re.sub(r'^(\s*)#define', r'\1// #define', l, count=1))
                changes.append(f'{name} {"เปิด" if tog[name] else "ปิด"}')
    return '\n'.join(lines), changes


def cfg_diff(old, new):
    import difflib
    return '\n'.join(difflib.unified_diff(old.split('\n'), new.split('\n'), 'เดิม', 'ใหม่', n=0, lineterm=''))


# ════════════════════════════════════════════════════════════════
#  งานที่ใช้เวลานาน (วัด CPR / build+อัปโหลด / ย้อนเวอร์ชัน) — ทีละงาน
# ════════════════════════════════════════════════════════════════
JOB = {'name': '', 'running': False, 'ok': None, 'log': [], 'progress': 0.0, 'result': None, 'cancel': False}
ANSI = re.compile(r'\x1b\[[0-9;]*m')


def job_log(t):
    JOB['log'] = (JOB['log'] + [ANSI.sub('', t)])[-300:]


def job_start(name, fn, *a):
    if JOB['running']:
        return False, f'มีงานกำลังทำอยู่: {JOB["name"]}'
    JOB.update(name=name, running=True, ok=None, log=[], progress=0.0, result=None, cancel=False)

    def run():
        try:
            JOB['ok'] = bool(fn(*a))
        except Exception as e:
            job_log(f'❌ ผิดพลาด: {e}')
            JOB['ok'] = False
        finally:
            JOB['running'] = False
            LINK.state['busy'] = False
    threading.Thread(target=run, daemon=True).start()
    return True, 'เริ่มแล้ว'


def read_status_enc():
    r = LINK.command('@STATUS', ('ST',), 0.6)
    if not r:
        return None
    d = dict(kv.split('=', 1) for kv in r[3:].split() if '=' in kv)
    return int(float(d['enc1'])), int(float(d['enc2']))


def cfg_num(name, default):
    for it in cfg_read()['items']:
        if it['name'] == name and it['value'] is not None:
            try:
                return float(it['value'])
            except ValueError:
                pass
    return default


def job_cpr(rounds, seconds):
    """เมนู 2–3: หมุนมอเตอร์เต็มกำลังทีละล้อ แล้วประมาณ CPR จาก rpm ตามสเปก (ค่าประมาณ — เมนู 7 แม่นกว่า)"""
    vop = cfg_num('MOTOR_OPERATING_VOLTAGE', 12); vme = min(cfg_num('MOTOR_POWER_MEASURED_VOLTAGE', 12), vop)
    rpm_spec = cfg_num('MOTOR_MAX_RPM', 110) * vme / vop
    exp_revs = rpm_spec * seconds / 60.0
    cpr_cfg = [cfg_num('COUNTS_PER_REV1', 958), cfg_num('COUNTS_PER_REV2', 925)]
    job_log(f'rpm ตามสเปก {rpm_spec:.1f} → ใน {seconds} วิ ควรหมุน {exp_revs:.1f} รอบ')
    res = {'rounds': [], 'cpr_cfg': cpr_cfg}
    total = rounds * 2; n = 0
    for r in range(rounds):
        order = (0, 1) if r % 2 == 0 else (1, 0)
        row = {}
        for m in order:
            if JOB['cancel']:
                LINK.command('@STOP'); job_log('หยุดตามคำสั่ง'); return False
            LINK.command('@ENCZERO')
            job_log(f'รอบ {r + 1}/{rounds} · ล้อ{"ซ้าย" if m == 0 else "ขวา"} หมุน PWM เต็ม {seconds} วิ…')
            t0 = time.time()
            while time.time() - t0 < seconds:
                if JOB['cancel'] or not LINK.state['connected']:
                    break
                LINK.command('@MOTOR %d %d' % ((1023, 0) if m == 0 else (0, 1023)), ('OK MOTOR',), 0.3)
                JOB['progress'] = (n + (time.time() - t0) / seconds) / total
                time.sleep(0.1)
            LINK.command('@STOP'); time.sleep(0.6)
            enc = read_status_enc()
            if enc is None:
                job_log('อ่าน encoder ไม่ได้'); return False
            ticks = abs(enc[m]); n += 1
            row['L' if m == 0 else 'R'] = {'ticks': ticks, 'cpr_est': round(ticks / exp_revs),
                                           'rpm_real': round(ticks / cpr_cfg[m] / seconds * 60, 1)}
            job_log(f'  tick {ticks} → CPR ประมาณ {ticks / exp_revs:.0f} · rpm จริง (ใช้ CPR ในโค้ด) {ticks / cpr_cfg[m] / seconds * 60:.1f}')
        res['rounds'].append(row)
    for w in 'LR':
        v = [x[w]['cpr_est'] for x in res['rounds'] if w in x]
        if v:
            avg = sum(v) / len(v); sd = (sum((x - avg) ** 2 for x in v) / (len(v) - 1)) ** 0.5 if len(v) > 1 else 0
            res[w] = {'avg': round(avg), 'sd': round(sd, 1)}
    JOB['result'] = res; JOB['progress'] = 1.0
    job_log('✅ เสร็จ')
    return True


def run_stream(cmd, cwd=None):
    p = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                         text=True, bufsize=1)
    for line in p.stdout:
        line = line.rstrip()
        if line:
            job_log(line)
            for key, pr in (('1) build', 0.1), ('Compiling', None), ('SUCCESS', 0.55), ('2) สำรอง', 0.6),
                            ('4) เขียน', 0.65), ('Wrote', 0.9), ('verify ผ่าน', 0.93)):
                if key in line:
                    JOB['progress'] = min(0.5, JOB['progress'] + 0.02) if pr is None else max(JOB['progress'], pr)
    return p.wait()


def job_flash():
    was = LINK.state['connected']
    if was:
        job_log('ปิดการเชื่อมต่อเว็บชั่วคราว (ให้ flash.sh ใช้พอร์ต)')
        LINK.disconnect(reboot=False)
        time.sleep(0.5)
    why = LINK.port_owner()
    if why:
        job_log('❌ ' + why); return False
    job_log('$ bash tools/flash.sh upload')
    rc = run_stream(['bash', FLASH, 'upload'], cwd=REPO)
    if rc != 0:
        job_log(f'❌ flash.sh จบด้วยรหัส {rc}'); return False
    JOB['progress'] = 0.95
    job_log('บอร์ดรีบูตแล้ว — ต่อกลับเข้าโหมดคาลิเบรต…'); time.sleep(2)
    ok, msg = LINK.connect(); job_log(('✅ ' if ok else '⚠️ ') + msg)
    JOB['progress'] = 1.0
    return True


def job_rollback(path):
    if LINK.state['connected']:
        LINK.disconnect(reboot=False); time.sleep(0.5)
    why = LINK.port_owner()
    if why:
        job_log('❌ ' + why); return False
    py = os.path.expanduser('~/.platformio/penv/bin/python')
    esp = os.path.expanduser('~/.platformio/packages/tool-esptoolpy/esptool.py')
    job_log(f'เขียน {os.path.basename(path)} ลงบอร์ด…')
    for i in range(3):
        rc = run_stream([py, esp, '--chip', 'esp32', '--port', PORT_DEV, '--baud', '115200', 'write_flash', '-z', '0x10000', path])
        if rc == 0:
            rc2 = run_stream([py, esp, '--chip', 'esp32', '--port', PORT_DEV, '--baud', '115200', 'verify_flash', '0x10000', path])
            if rc2 == 0:
                job_log('✅ เขียน + ตรวจผ่าน'); break
        job_log(f'ลองใหม่ครั้งที่ {i + 2}…'); time.sleep(2)
    else:
        return False
    time.sleep(2)
    ok, msg = LINK.connect(); job_log(('✅ ' if ok else '⚠️ ') + msg + ' (เฟิร์มแวร์เก่ากว่า 14c จะต่อหน้าเว็บไม่ได้ — ปกติ)')
    JOB['progress'] = 1.0
    return True


def list_backups():
    out = []
    for fn in sorted(os.listdir(BACKUP_DIR), reverse=True) if os.path.isdir(BACKUP_DIR) else []:
        p = os.path.join(BACKUP_DIR, fn)
        if fn.endswith('.bin') and os.path.isfile(p) and os.path.getsize(p) > 100000:
            out.append({'file': fn, 'size': os.path.getsize(p), 'mtime': time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(p)))})
    out.sort(key=lambda x: x['mtime'], reverse=True)
    return out[:30]



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
        if path == '/lrb_pid_guide.png':
            try:
                with open(os.path.join(HERE, 'lrb_pid_guide.png'), 'rb') as f:
                    return self._reply(200, f.read(), 'image/png')
            except OSError:
                return self._reply(404, {'msg': 'ไม่พบภาพ'})
        if path == '/api/config':
            try:
                return self._reply(200, cfg_read())
            except OSError as e:
                return self._reply(500, {'msg': f'อ่าน config ไม่ได้: {e}'})
        if path == '/api/job':
            return self._reply(200, JOB)
        if path == '/api/backups':
            return self._reply(200, {'files': list_backups()})
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
        if path in ('/api/config/preview', '/api/config/save'):
            out, tog, err = cfg_validate(d)
            if err:
                return self._reply(400, {'ok': False, 'msg': ' · '.join(err)})
            old = open(CFG, encoding='utf-8').read()
            new, changes = cfg_apply(old, out, tog)
            if path.endswith('preview') or not changes:
                return self._reply(200, {'ok': True, 'changes': changes, 'diff': cfg_diff(old, new)})
            bak = CFG + time.strftime('.bak_web_%Y%m%d_%H%M%S')
            with open(bak, 'w', encoding='utf-8') as f: f.write(old)
            with open(CFG, 'w', encoding='utf-8') as f: f.write(new)
            msg = 'config (web): ' + ', '.join(changes)
            subprocess.run(['git', '-C', REPO, 'add', CFG], capture_output=True)
            g = subprocess.run(['git', '-C', REPO, 'commit', '-q', '-m', msg], capture_output=True, text=True)
            LINK.log('บันทึก config: ' + ', '.join(changes))
            return self._reply(200, {'ok': True, 'changes': changes, 'backup': os.path.basename(bak),
                                     'git': 'commit แล้ว' if g.returncode == 0 else g.stderr.strip()[:200]})
        if path == '/api/flash':
            LINK.state['busy'] = True
            ok, msg = job_start('build + อัปโหลดเฟิร์มแวร์', job_flash)
            if not ok: LINK.state['busy'] = False
            return self._reply(200, {'ok': ok, 'msg': msg})
        if path == '/api/rollback':
            fn = os.path.basename(str(d.get('file', '')))
            p = os.path.join(BACKUP_DIR, fn)
            if not (fn.endswith('.bin') and os.path.isfile(p)):
                return self._reply(400, {'ok': False, 'msg': 'ไม่พบไฟล์'})
            LINK.state['busy'] = True
            ok, msg = job_start('ย้อนเฟิร์มแวร์ ' + fn, job_rollback, p)
            if not ok: LINK.state['busy'] = False
            return self._reply(200, {'ok': ok, 'msg': msg})
        if path == '/api/cpr':
            if not LINK.state['connected']:
                return self._reply(400, {'ok': False, 'msg': 'ยังไม่เชื่อมต่อ'})
            try:
                rounds = 5 if int(d.get('rounds', 1)) >= 5 else 1
                secs = max(5, min(30, int(d.get('seconds', 20))))
            except (TypeError, ValueError):
                return self._reply(400, {'ok': False, 'msg': 'ค่าไม่ถูกต้อง'})
            LINK.state['busy'] = True
            ok, msg = job_start(f'วัด CPR ×{rounds}', job_cpr, rounds, secs)
            if not ok: LINK.state['busy'] = False
            return self._reply(200, {'ok': ok, 'msg': msg})
        if path == '/api/job/cancel':
            JOB['cancel'] = True
            LINK.command('@STOP')
            return self._reply(200, {'ok': True})
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

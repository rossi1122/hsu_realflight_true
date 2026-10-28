"""test_plot_report.py — 不用飛機, 先試畫圖流程 (xwing_plot_report.py)
用假 rospy + 簡單飛機模型跑【真的】xwing_virtual_flight.main(), 產生一份完整飛完一趟
(SEARCH → PRE_DIVE → DIVE → PULLOUT → DONE) 的 log, 再做一個假的車子 BIN
(比虛擬目標晚 4 s 出發), 最後自動跑 xwing_plot_report.py --car-bin-dir.
⚠ 飛機模型很粗, 圖裡的數字不代表實飛結果, 只用來確認畫圖流程能跑.

用法: python3 test_plot_report.py            (一趟)
      python3 test_plot_report.py --two-runs (同一份 log 切兩次 GUIDED)
輸出: plot_test/ (log + car_bin/)  →  plot_results/xwing_virtual_<時間>_plottest/run1/"""
import sys, types, math, os, glob, shutil, struct, random, subprocess
REPO = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(REPO, 'plot_test')
TWO = '--two-runs' in sys.argv
VIS = '--vision' in sys.argv
sys.path.insert(0, REPO)
os.chdir(REPO)
import time as _time
CLK = {'t': 1000.0}
_time.time = lambda: CLK['t']
_time.monotonic = lambda: CLK['t']
DT = 0.02
GPS0 = 1793320000.0          # 2026-10-30 附近 (UTC)
rospy = types.ModuleType('rospy')
STOP = {'v': False}
class Rate:
    def __init__(self, hz): pass
    def sleep(self): step()
rospy.Rate = Rate
rospy.init_node = lambda *a, **k: None
rospy.is_shutdown = lambda: STOP['v']
rospy.sleep = lambda s: None
sys.modules['rospy'] = rospy
for mod, names in {'mavros_msgs': [], 'mavros_msgs.msg': ['State','RCOut','AttitudeTarget','NavControllerOutput','VFR_HUD','HomePosition'],
                   'mavros_msgs.srv': ['ParamGet'], 'sensor_msgs': [], 'sensor_msgs.msg': ['NavSatFix','Imu','BatteryState','TimeReference'],
                   'std_msgs': [], 'std_msgs.msg': ['Float64'], 'geometry_msgs': [], 'geometry_msgs.msg': ['TwistStamped']}.items():
    m = types.ModuleType(mod)
    for n in names: setattr(m, n, type(n, (), {}))
    sys.modules[mod] = m
import xwing_virtual_flight as F
cfg = F.XwingVirtualConfig()
S0 = cfg.MOVING_START_GPS
R = 6378137.0
def ne2gps(n, e):
    return (S0[0] + math.degrees(n / R),
            S0[1] + math.degrees(e / (R * math.cos(math.radians(S0[0])))))
random.seed(1)
AC = {'n': 650.0, 'e': 25.0, 'h': 238.0, 'V': 30.0, 'pitch': 0.0, 'chi': 183.0, 'roll': 0.0, 'r': 0.0}
BR = {'out': None}
SCRIPT = [(2.0, True)] + ([(40.0, False), (60.0, True)] if TWO else [])
T0 = CLK['t']
def guided_now():
    g = False
    for tr, gv in SCRIPT:
        if CLK['t'] - T0 >= tr: g = gv
    return g
RUD = {'raw': 0.0, 'sent': 0.0}
def step():
    CLK['t'] += DT
    out = BR['out']; g = guided_now()
    if g and out and out.get('cmd_active'):
        tgt = out['pitch_cmd_deg']
    else:
        tgt = max(-5, min(8, 0.5 * (240 - AC['h'])))
    AC['pitch'] += max(-18 * DT, min(18 * DT, (tgt - AC['pitch']) * 4 * DT))
    mode = out['mode'] if (g and out) else 'X'
    if mode == 'DIVE': AC['V'] = min(62, AC['V'] + 2.6 * DT)
    elif mode == 'PRE_DIVE': AC['V'] += 1.0 * DT
    else: AC['V'] += (30 - AC['V']) * 0.02
    # 航向: SEARCH 跟 heading_cmd (FC 協調轉彎), DIVE 用方向舵
    if g and out and out.get('heading_cmd_deg') is not None and mode in ('SEARCH', 'PRE_DIVE'):
        err = (out['heading_cmd_deg'] - AC['chi'] + 180) % 360 - 180
        AC['r'] = max(-8, min(8, 0.8 * err))
    elif mode == 'DIVE':
        AC['r'] = 0.35 * RUD['sent'] + random.gauss(0, 0.3)
    else:
        AC['r'] *= 0.9
    AC['chi'] = (AC['chi'] + AC['r'] * DT) % 360
    AC['roll'] += (2.0 * AC['r'] - AC['roll']) * 0.05
    v = AC['V']
    AC['h'] += v * math.sin(math.radians(AC['pitch'])) * DT
    AC['n'] += v * math.cos(math.radians(AC['pitch'])) * math.cos(math.radians(AC['chi'])) * DT
    AC['e'] += v * math.cos(math.radians(AC['pitch'])) * math.sin(math.radians(AC['chi'])) * DT
    if CLK['t'] - T0 > (100 if TWO else 45): STOP['v'] = True
    if TWO and abs(CLK['t'] - T0 - 59.0) < DT / 2:
        AC.update({'n': 640.0, 'e': -20.0, 'h': 241.0, 'V': 30.0, 'pitch': 0.0, 'chi': 178.0})

class FakeIO:
    def __init__(self): pass
    def snapshot(self):
        lat, lon = ne2gps(AC['n'], AC['e'])
        g = guided_now()
        c = math.cos(math.radians(AC['pitch']))
        vn = AC['V'] * c * math.cos(math.radians(AC['chi']))
        ve = AC['V'] * c * math.sin(math.radians(AC['chi']))
        vd = -AC['V'] * math.sin(math.radians(AC['pitch']))
        return {'guided': g, 'lat': lat, 'lon': lon, 'rel_alt': AC['h'],
                'compass_hdg': AC['chi'], 'pitch_deg': AC['pitch'], 'roll_deg': AC['roll'],
                'airspeed': AC['V'], 'vn': vn, 've': ve, 'vd': vd,
                'yaw_rate_dps': AC['r'], 'gps_unix': GPS0 + CLK['t'] - T0, 'p_dps': 0, 'q_dps': 0,
                'r_dps': AC['r'], 'nz_g': 1, 'fc_pitch_tgt_deg': None,
                'fc_nav_pitch_deg': None, 'fc_nav_roll_deg': None,
                'fc_nav_bearing_deg': None, 'fc_mode': 'GUIDED' if g else 'AUTO',
                'batt_v': 16.4 - 0.004 * (CLK['t'] - T0), 'rcout': [1500] * 8}
class FakeBridge:
    def __init__(self, **k): pass
    def start(self): pass
    def stop(self): pass
    def submit(self, out, *a, **k):
        BR['out'] = out
        if out.get('mode') == 'DIVE' and out.get('pid_dpsi_deg') not in (None, ''):
            raw = cfg.K_LAT_TO_RUDDER * float(out['pid_dpsi_deg']) * 4
            lim = cfg.RUDDER_LIMIT_DEG
            RUD['raw'] = raw
            cmd = max(-lim, min(lim, raw))
            RUD['sent'] += max(-cfg.RUDDER_SLEW_DPS * DT, min(cfg.RUDDER_SLEW_DPS * DT, cmd - RUD['sent']))
        else:
            RUD['raw'] = 0.0; RUD['sent'] = 0.0
    def get_debug(self):
        o = BR['out'] or {}
        if o.get('mode') == 'DIVE':
            return {'brg_mask': 128, 'rudder_raw_deg': round(RUD['raw'], 2),
                    'rudder_cmd_deg': round(max(-30, min(30, RUD['raw'])), 2),
                    'rudder_sent_deg': round(RUD['sent'], 2), 'rudder_limit_deg': 30.0,
                    'lim_by_airspeed': 0, 'lim_by_roll': int(abs(AC['roll']) > 25),
                    'brg_thr': 0.05}
        return {'brg_mask': 132, 'brg_thr': 0.55}
    def send_tecs_targets_async(self, alt_m=None, speed_mps=None, tag='', on_done=None):
        if on_done: on_done(True, True)
F.MavrosIO = FakeIO
F.ArduPilotAttitudeBridge = FakeBridge
F.check_airspeed_params = lambda cfg: None
F.keyboard_thread = lambda flags: None
case = 'plottest2' if TWO else 'plottest'
sys.argv = ['x', '--moving', '--no-vision', '--case', case]
F.main()
f = sorted(glob.glob(os.path.join(REPO, 'flight_logs', f'*{case}_events.txt')))[-1]
c = f.replace('_events.txt', '.csv')
os.makedirs(OUT, exist_ok=True)
for p in (f, c):
    shutil.move(p, os.path.join(OUT, os.path.basename(p)))
print('log ->', os.path.join(OUT, os.path.basename(c)))

# ── 假車子 BIN: 從起點北方 10 m 出發往南 ~10 m/s, GPS 5 Hz ──
def fmt_msg(typ, name, fmt, cols):
    length = 3 + struct.calcsize('<' + ''.join({'Q':'Q','B':'B','I':'I','H':'H','L':'i','f':'f'}[x] for x in fmt))
    return b'\xa3\x95\x80' + struct.pack('<BB4s16s64s', typ, length, name.encode(), fmt.encode(), cols.encode())
buf = fmt_msg(130, 'GPS', 'QBBIHLLf', 'TimeUS,I,Status,GMS,GWk,Lat,Lng,Spd')
lead = -4.0   # 車子比虛擬目標晚出發 4 s
for k in range(int(130 * 5)):
    tt = k * 0.2
    unix = GPS0 - 20 + tt
    gps_s = unix - 315964800 + 18
    wk = int(gps_s // 604800); ms = int(round((gps_s - wk * 604800) * 1000))
    tm = tt - 20 - 2.0 + lead       # 相對 GUIDED 時刻
    spd = 0 if tm < 0 else min(10.0, 10.0 * tm / 7.0)
    dist = 0 if tm < 0 else (5 / 7.0 * tm * tm if tm < 7 else 35 + 10 * (tm - 7))
    n = 10 - dist + random.gauss(0, 0.6); e = 1.5 + random.gauss(0, 0.6)
    la, lo = ne2gps(n, e)
    buf += b'\xa3\x95' + struct.pack('<BQBBIHiif', 130, int(tt * 1e6), 0, 3, ms, wk,
                                       int(round(la * 1e7)), int(round(lo * 1e7)), spd + random.gauss(0, 0.15))
os.makedirs(os.path.join(OUT, 'car_bin'), exist_ok=True)
open(os.path.join(OUT, 'car_bin', '00000031.BIN'), 'wb').write(buf)
print('bin ->', os.path.join(OUT, 'car_bin', '00000031.BIN'))

print('\n[test] 開始畫圖 ...\n')
r = subprocess.call([sys.executable, os.path.join(REPO, 'xwing_plot_report.py'),
                     os.path.join(OUT, os.path.basename(c)),
                     '--car-bin-dir', os.path.join(OUT, 'car_bin')])
print('\n[test] ' + ('完成 — 看 plot_results/ 裡的 report.pdf' if r == 0 else '畫圖失敗'))
sys.exit(r)

"""test_guided_rearm.py — 重複撥 GUIDED 的流程測試 (不需要 ROS / 飛控)
假 rospy + 假時鐘 + 簡單飛機模型, 直接跑 xwing_virtual_flight.main(). 情境:
  t=0.2 還沒撥 GUIDED 就誤按 x              → ABORT, 1 s 後重置 (不算一趟)
  t=2   撥 GUIDED (WP3, 離 WP4 約 600 m)   → 第 1 趟開始
  t=15  DIVE 中切 FBWA                       → 第 1 趟結束 (DONE), 1 s 後重置
  t=21  在目標附近 (高度也不夠) 又撥 GUIDED  → 拒絕啟動
  t=26  切出; t=41 飛機回到 WP3; t=42 撥 GUIDED → 第 2 趟從頭完整飛完
用法: python3 test_guided_rearm.py   (最後印 PASS / FAIL)"""
import sys, types, math, os, glob, csv
REPO = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, REPO)

# ── 假時鐘 ──
import time as _time
CLK = {'t': 1000.0}
_time.time = lambda: CLK['t']
_time.monotonic = lambda: CLK['t']
DT = 0.02

# ── 假 rospy ──
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
from virtual_target_geom import gps_to_ned
cfg = F.XwingVirtualConfig()
S0 = cfg.MOVING_START_GPS
R = 6378137.0
def ne2gps(n, e):
    return (S0[0] + math.degrees(n / R),
            S0[1] + math.degrees(e / (R * math.cos(math.radians(S0[0])))))

# ── 飛機: 往南 30 m/s, 高度跟 pitch 命令 (GUIDED 時), 非 GUIDED 時 FC 爬回 240 ──
AC = {'n': 613.0, 'e': 0.0, 'h': 240.0, 'V': 30.0, 'pitch': 0.0}
BR = {'out': None, 'guided': False}
SCRIPT = []   # (t_rel, guided)
T0 = CLK['t']
def guided_now():
    g = False
    for tr, gv in SCRIPT:
        if CLK['t'] - T0 >= tr: g = gv
    return g
def step():
    CLK['t'] += DT
    out = BR['out']; g = guided_now()
    if g and out and out.get('cmd_active'):
        tgt = out['pitch_cmd_deg']
    else:
        tgt = max(-5, min(8, 0.5 * (240 - AC['h'])))
    AC['pitch'] += max(-15 * DT, min(15 * DT, tgt - AC['pitch']))
    if g and out and out['mode'] == 'DIVE': AC['V'] = min(60, AC['V'] + 3 * DT)
    elif not (g and out and out['mode'] in ('PRE_DIVE',)): AC['V'] += (30 - AC['V']) * 0.02
    v = AC['V']
    AC['h'] += v * math.sin(math.radians(AC['pitch'])) * DT
    AC['n'] -= v * math.cos(math.radians(AC['pitch'])) * DT
    if CLK['t'] - T0 > 80: STOP['v'] = True

class FakeIO:
    def __init__(self): pass
    def snapshot(self):
        lat, lon = ne2gps(AC['n'], AC['e'])
        g = guided_now()
        vn = -AC['V'] * math.cos(math.radians(AC['pitch']))
        vd = -AC['V'] * math.sin(math.radians(AC['pitch']))
        return {'guided': g, 'lat': lat, 'lon': lon, 'rel_alt': AC['h'],
                'compass_hdg': 180.0, 'pitch_deg': AC['pitch'], 'roll_deg': 0.0,
                'airspeed': AC['V'], 'vn': vn, 've': 0.0, 'vd': vd,
                'yaw_rate_dps': 0.0, 'gps_unix': None, 'p_dps': 0, 'q_dps': 0,
                'r_dps': 0, 'nz_g': 1, 'fc_pitch_tgt_deg': None,
                'fc_nav_pitch_deg': None, 'fc_nav_roll_deg': None,
                'fc_nav_bearing_deg': None, 'fc_mode': 'GUIDED' if g else 'FBWA',
                'batt_v': 16.0, 'rcout': [1500] * 8}
class FakeBridge:
    def __init__(self, **k): pass
    def start(self): pass
    def stop(self): pass
    def submit(self, out, *a, **k): BR['out'] = out
    def get_debug(self): return {}
    def send_tecs_targets_async(self, alt_m=None, speed_mps=None, tag='', on_done=None):
        if on_done: on_done(True, True)
F.MavrosIO = FakeIO
F.ArduPilotAttitudeBridge = FakeBridge
F.check_airspeed_params = lambda cfg: None
FLAGS = {}
F.keyboard_thread = lambda flags: FLAGS.update(ref=flags)

# 腳本: t=1 GUIDED; DIVE 中 (t≈14) 切出 2 s; 然後把飛機搬回 WP3 北方 613 m @240 m (模擬繞一圈) 再切 GUIDED
SCRIPT += [(2.0, True), (15.0, False), (21.0, True), (26.0, False), (42.0, True)]
_orig_step = step
def step2():
    _orig_step()
    if abs(CLK['t'] - T0 - 0.2) < DT / 2 and 'ref' in FLAGS:
        FLAGS['ref']['abort'] = True          # 還沒撥 GUIDED 就誤按 x
    if abs(CLK['t'] - T0 - 41.0) < DT / 2:
        AC.update({'n': 613.0, 'h': 240.0, 'V': 30.0, 'pitch': 0.0})
        # 目標從 WP4 重新出發, 飛機在 WP3 → 重現第一趟幾何
        AC['n'] = 613.0
Rate.sleep = lambda self: step2()

sys.argv = ['x', '--moving', '--dive-hold-heading', '--no-vision', '--case', 'rearm_test']
F.main()

f = sorted(glob.glob(os.path.join(REPO, 'flight_logs', '*rearm_test_events.txt')))[-1]
print(open(f).read())
c = f.replace('_events.txt', '.csv')
rows = list(csv.DictReader(open(c)))
seq = []
for r in rows:
    k = (r['run'], r['mode'], r['guided'])
    if not seq or seq[-1][0] != k: seq.append((k, r['t']))
for k, t in seq: print(t, k)
ev = open(f).read()
ok = all(x in ev for x in ('任務重置 — 下次撥 GUIDED = 第 1 趟', '第 1 趟任務開始',
                           '第 1 趟 mode=DIVE (推頭後離開', '任務重置 — 下次撥 GUIDED = 第 2 趟',
                           '不啟動任務', '第 2 趟任務開始', 'MODE: PULLOUT → DONE'))
ok = ok and [k for k, _ in seq][-1] == ('2', 'DONE', '1')
for p in (f, c):
    os.remove(p)
print('PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)

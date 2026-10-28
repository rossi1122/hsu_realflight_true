#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xwing_plot_report.py — xwing_virtual 實飛 log → 報告圖 + 自動檢查 (飛完直接給老師看)
═══════════════════════════════════════════════════════════════════
參考學長 plot_flight_report / plot_moving_report 的圖表配置, 改成讀 xwing_virtual_*.csv
(沒有 YOLO / _vision.csv / real_* 欄位, 改畫虛擬目標 PID、方向舵、雲台 UART 紀錄).
只需要 numpy + matplotlib.

用法:
  python3 xwing_plot_report.py --latest                       # flight_logs/ 裡最新的一份
  python3 xwing_plot_report.py flight_logs/xwing_virtual_20261030_xxxxxx_flight2_A.csv
  python3 xwing_plot_report.py <log.csv> --car-bin-dir car_bin   # 加畫 虛擬目標 vs 真車 GPS
  python3 xwing_plot_report.py <log.csv> --run 2              # 只畫第 2 趟
  python3 xwing_plot_report.py <log.csv> --full               # 不裁切, 畫整份 (地面測試用)

輸出: plot_results/<log 檔名>/run<N>/
  fig1_overview.png          模式 / AGL / 空速 / pitch / 油門 時間歷程
  fig2_dive_profile.png      距離–高度剖面 (對照設計 30° 線) + 航跡角 γ
  fig3_attitude.png          pitch / heading 命令 vs 實際, roll (含 roll 保護線)
  fig4_image_error.png       虛擬畫面誤差 (橫向 / 縱向) + 畫面上的目標軌跡
  fig5_pid_terms.png         DIVE 段 PID 各項 (P / I / D / 輸出)
  fig6_rudder.png            方向舵 raw / cmd / sent, 限幅, 偏航角速度
  fig7_trajectory.png        俯視圖: 飛機 vs 虛擬目標
  fig8_miss.png              預測落點誤差 (沿航跡 / 側向)
  fig9_gimbal.png            雲台 UART (有接雲台才有): 連線、模式、pitch、偵測
  fig10_system.png           電壓、控制迴路週期
  fig11_target_gps.png       (有給 BIN) 虛擬目標 vs 真車 GPS
  report.pdf                 上面全部合成一份
  summary.txt                時間線 + 各段數字 + 自動檢查 (⚠ = 建議看的參數)
"""
import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
plt.rcParams['axes.unicode_minus'] = False

try:
    from xwing_virtual_config import XwingVirtualConfig
    CFG = XwingVirtualConfig()
except Exception:  # noqa: BLE001  (單獨拷到筆電也能跑, 用下面的預設值)
    CFG = None

DEFAULTS = dict(CRUISE_AGL_M=240.0, DIVE_ANGLE_DEG=30.0, PULLOUT_AGL_M=120.0,
                PULLOUT_VMAX_MPS=60.0, ENTRY_MIN_AGL_M=200.0, ABORT_MIN_AGL_M=100.0,
                V_CRUISE_MPS=30.0, PREDIVE_HANDOFF_PITCH_TOL_DEG=3.0,
                ROLL_PROT_FULL_DEG=25.0, ROLL_PROT_ZERO_DEG=35.0, RUDDER_LIMIT_DEG=30.0,
                IMG_W=1920, IMG_H=1080, FX_1X=1066.7, FY_1X=1059.5,
                CX_1X=959.4932, CY_1X=545.0292, GIMBAL_LOCK_PITCH_DEG=-30.0,
                GIMBAL_LOCK_TOL_DEG=2.0, PID_LAT_KP=2.0, PID_LAT_KI=0.2, PID_LAT_KD=0.0,
                PID_LON_KP=0.8, PID_LON_KI=0.1, PID_LON_KD=0.2, K_LAT_TO_RUDDER=0.5)


def C(name):
    if CFG is not None and hasattr(CFG, name):
        return getattr(CFG, name)
    return DEFAULTS[name]


MODE_COLOR = {'SEARCH': '#4A90E2', 'PRE_DIVE': '#F5A623', 'DIVE': '#2CA02C',
              'PULLOUT': '#9467BD', 'DONE': '#9B9B9B', 'ABORT': '#D0021B'}
C_CMD, C_ACT, C_REF = '#D0021B', '#1F4E79', '#555555'
C_TGT = '#D0021B'


# ─────────────────────────── log 讀取 ───────────────────────────
class Log:
    """CSV → 欄位字串陣列; num() 轉 float (空白/文字 = NaN)."""

    def __init__(self, cols):
        self.cols = cols
        self._num = {}
        self.n = len(next(iter(cols.values()))) if cols else 0

    @classmethod
    def load(cls, path):
        with open(path, newline='', encoding='utf-8-sig') as f:
            rd = csv.reader(f)
            head = next(rd)
            rows = [r for r in rd if r]
        cols = {}
        for j, k in enumerate(head):
            cols[k.strip()] = np.array([r[j].strip() if j < len(r) else '' for r in rows], dtype=object)
        return cls(cols)

    def has(self, k):
        return k in self.cols and np.any(np.isfinite(self.num(k)))

    def num(self, k):
        if k not in self._num:
            out = np.full(self.n, np.nan)
            if k in self.cols:
                for i, v in enumerate(self.cols[k]):
                    if v in ('', 'None', 'nan'):
                        continue
                    if v in ('True', 'true'):
                        out[i] = 1.0
                    elif v in ('False', 'false'):
                        out[i] = 0.0
                    else:
                        try:
                            out[i] = float(v)
                        except ValueError:
                            pass
            self._num[k] = out
        return self._num[k]

    def s(self, k):
        if k not in self.cols:
            return np.array([''] * self.n, dtype=object)
        return self.cols[k]

    def sub(self, mask):
        L = Log({k: v[mask] for k, v in self.cols.items()})
        L._num = {k: v[mask] for k, v in self._num.items()}
        return L


def read_events(csv_path):
    p = csv_path[:-4] + '_events.txt' if csv_path.endswith('.csv') else None
    if not p or not os.path.isfile(p):
        return []
    out = []
    with open(p, encoding='utf-8', errors='ignore') as f:
        for ln in f:
            m = re.match(r'\s*([\d.]+)s\s+(.*)', ln.rstrip('\n'))
            if m:
                out.append((float(m.group(1)), m.group(2)))
    return out


# ─────────────────────────── 小工具 ───────────────────────────
def wrap180(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def rmse(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float('nan')


def nanmean(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(x.mean()) if len(x) else float('nan')


def nanmax(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(x.max()) if len(x) else float('nan')


def nanmin(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(x.min()) if len(x) else float('nan')


def f1(x, nd=1, unit=''):
    return '—' if x is None or not np.isfinite(x) else f'{x:.{nd}f}{unit}'


def mode_segments(t, mode):
    segs = []
    if len(t) == 0:
        return segs
    s = 0
    for i in range(1, len(t) + 1):
        if i == len(t) or mode[i] != mode[s]:
            e = t[i] if i < len(t) else t[-1]
            segs.append((mode[s], t[s], e))
            s = i
    return segs


def shade_modes(ax, segs):
    for m, a, b in segs:
        c = MODE_COLOR.get(m)
        if c:
            ax.axvspan(a, b, color=c, alpha=.10, lw=0)


def mode_legend(fig, segs, y=.995):
    seen = []
    for m, _, _ in segs:
        if m in MODE_COLOR and m not in seen:
            seen.append(m)
    if seen:
        fig.legend(handles=[Patch(facecolor=MODE_COLOR[m], alpha=.25, label=m) for m in seen],
                   loc='upper right', ncol=len(seen), fontsize=8, frameon=False,
                   bbox_to_anchor=(.99, y))


def stat_box(ax, text, loc='upper left'):
    xy = {'upper left': (.01, .97, 'left', 'top'), 'upper right': (.99, .97, 'right', 'top'),
          'lower left': (.01, .03, 'left', 'bottom'), 'lower right': (.99, .03, 'right', 'bottom')}[loc]
    ax.text(xy[0], xy[1], text, transform=ax.transAxes, ha=xy[2], va=xy[3], fontsize=8,
            family='monospace', bbox=dict(boxstyle='round', fc='white', ec='#BBBBBB', alpha=.85))


# ─────────────────────────── 單趟資料整理 ───────────────────────────
class Run:
    def __init__(self, L, run, full=False, pre=3.0, post=3.0):
        t = L.num('t')
        g = L.num('guided') > 0
        r = L.num('run') if L.has('run') else np.where(g, 1.0, 0.0)
        self.run = run
        if full:
            sel = np.ones(L.n, bool)
            self.t_on = float(t[g][0]) if g.any() else float(t[0])
        else:
            mine = (r == run)
            gm = mine & g
            if not gm.any():
                raise ValueError(f'第 {run} 趟沒有 GUIDED 資料')
            self.t_on = float(t[gm][0])
            t_off = float(t[gm][-1])
            t_last = float(t[mine][-1])
            sel = (t >= self.t_on - pre) & (t <= min(t_off + post, t_last))
        self.L = L.sub(sel)
        self.t = self.L.num('t') - self.t_on          # 0 = 撥 GUIDED
        self.mode = np.array([m if gg > 0 else '' for m, gg in
                              zip(self.L.s('mode'), self.L.num('guided'))], dtype=object)
        self.mode_raw = self.L.s('mode')
        self.segs = [s for s in mode_segments(self.t, self.mode) if s[0]]
        # 各模式第一次出現的時間 (GUIDED 中)
        self.first = {}
        for m, a, _ in self.segs:
            self.first.setdefault(m, a)

    def m(self, *modes):
        return np.isin(self.mode, modes)

    def window(self, pad0=2.0, pad1=3.0):
        """PRE_DIVE 前 pad0 秒 ~ PULLOUT/ABORT 後 pad1 秒; 沒推頭就整段."""
        a = self.first.get('PRE_DIVE', self.first.get('DIVE'))
        if a is None:
            return self.t[0], self.t[-1]
        b = None
        for k in ('DONE', 'ABORT'):
            if k in self.first:
                b = self.first[k] if b is None else min(b, self.first[k])
        if b is None:
            b = self.t[-1]
        return a - pad0, min(b + pad1, self.t[-1])


# ─────────────────────────── 圖 ───────────────────────────
def _save(fig, path, pdf):
    fig.savefig(path, dpi=140)
    if pdf is not None:
        pdf.savefig(fig)
    plt.close(fig)


def fig1_overview(R, path, title, pdf):
    L, t = R.L, R.t
    fig, ax = plt.subplots(5, 1, figsize=(12, 11), sharex=True, layout='constrained')
    agl = L.num('agl')
    ax[0].plot(t, agl, color=C_ACT, label='AGL')
    for y, lab, st in ((C('CRUISE_AGL_M'), 'cruise', ':'), (C('ENTRY_MIN_AGL_M'), 'entry min', '--'),
                       (C('PULLOUT_AGL_M'), 'pullout', '--'), (C('ABORT_MIN_AGL_M'), 'abort', '-.')):
        ax[0].axhline(y, color=C_REF, ls=st, lw=.8)
        ax[0].text(t[0], y, f' {lab} {y:g} m', fontsize=7, va='bottom', color=C_REF)
    ax[0].set_ylabel('AGL (m)')
    V = L.num('V')
    ax[1].plot(t, V, color=C_ACT, label='Airspeed')
    ax[1].axhline(C('PULLOUT_VMAX_MPS'), color=C_CMD, ls='--', lw=.8, label=f'Vmax {C("PULLOUT_VMAX_MPS"):g}')
    ax[1].axhline(C('V_CRUISE_MPS'), color=C_REF, ls=':', lw=.8, label=f'cruise {C("V_CRUISE_MPS"):g}')
    ax[1].set_ylabel('Airspeed (m/s)')
    ax[1].legend(fontsize=8, loc='upper left')
    ax[2].plot(t, L.num('pitch_cmd') * np.where(R.mode != '', 1.0, np.nan), color=C_CMD, lw=1.2,
               label='pitch cmd')
    ax[2].plot(t, L.num('theta_deg'), color=C_ACT, lw=1.2, label='pitch')
    ax[2].plot(t, L.num('gamma_deg'), color='#2CA02C', lw=.9, ls='--', label='γ (flight path)')
    ax[2].axhline(-C('DIVE_ANGLE_DEG'), color=C_REF, ls=':', lw=.8)
    ax[2].set_ylabel('deg')
    ax[2].legend(fontsize=8, loc='lower left', ncol=3)
    ax[3].plot(t, L.num('ground_m'), color=C_ACT, label='ground dist to target')
    ax[3].plot(t, L.num('slant_m'), color='#7F7F7F', lw=.9, label='slant range')
    ax[3].axhline(nanmean(L.num('predive_dist_m')), color=C_REF, ls='--', lw=.8, label='push-over dist')
    ax[3].set_ylabel('Distance (m)')
    ax[3].legend(fontsize=8, loc='upper right')
    ax[4].plot(t, L.num('throttle'), color='#8C564B', label='throttle')
    ax[4].set_ylabel('Throttle')
    ax[4].set_xlabel('Time from GUIDED (s)')
    for a in ax:
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
    fig.suptitle(f'Fig 1  Mission overview — {title}', fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


def fig2_profile(R, path, title, pdf):
    L, t = R.L, R.t
    a0, a1 = R.window(4.0, 4.0)
    w = (t >= a0) & (t <= a1)
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.2), layout='constrained',
                           gridspec_kw=dict(width_ratios=[1.4, 1]))
    gd, agl = L.num('ground_m'), L.num('agl')
    for m, c in MODE_COLOR.items():
        k = w & (R.mode == m)
        if k.any():
            ax[0].plot(gd[k], agl[k], '.', ms=2.5, color=c, label=m)
    th = math.tan(math.radians(C('DIVE_ANGLE_DEG')))
    xs = np.linspace(0, C('CRUISE_AGL_M') / th, 50)
    ax[0].plot(xs, xs * th, color=C_REF, ls='--', lw=1, label=f'design {C("DIVE_ANGLE_DEG"):g}° line')
    ax[0].axhline(C('PULLOUT_AGL_M'), color=C_CMD, ls=':', lw=.8, label='pullout AGL')
    ax[0].axhline(C('ABORT_MIN_AGL_M'), color=C_CMD, ls='-.', lw=.8, label='abort AGL')
    for j, m in enumerate(('PRE_DIVE', 'DIVE', 'PULLOUT', 'ABORT')):
        if m in R.first:
            i = np.argmin(np.abs(t - R.first[m]))
            if np.isfinite(gd[i]):
                ax[0].plot(gd[i], agl[i], 'o', mfc='none', mec='k')
                ax[0].annotate(f'{m}\n{gd[i]:.0f} m / {agl[i]:.0f} m', (gd[i], agl[i]),
                               textcoords='offset points', fontsize=7,
                               xytext=(-70, -26) if j % 2 == 0 else (8, 6))
    ax[0].set(xlabel='Ground distance to (moving) target (m)', ylabel='AGL (m)',
              title='(a) Distance–altitude profile')
    ax[0].invert_xaxis()
    ax[0].set_ylim(bottom=0)
    ax[0].legend(fontsize=7, loc='lower right')
    gam, the = L.num('gamma_deg'), L.num('theta_deg')
    ax[1].plot(t[w], gam[w], color='#2CA02C', label='γ flight path angle')
    ax[1].plot(t[w], the[w], color=C_ACT, lw=.9, label='θ pitch')
    ax[1].plot(t[w], L.num('pitch_cmd')[w], color=C_CMD, lw=.9, ls='--', label='pitch cmd')
    ax[1].axhline(-C('DIVE_ANGLE_DEG'), color=C_REF, ls=':', lw=1)
    shade_modes(ax[1], R.segs)
    ax[1].set_xlim(a0, a1)
    ax[1].set(xlabel='Time from GUIDED (s)', ylabel='deg', title='(b) γ / θ during dive')
    d = R.m('DIVE')
    if d.any():
        stat_box(ax[1], f'DIVE mean γ  {nanmean(gam[d]):6.1f}°\n'
                        f'design      {-C("DIVE_ANGLE_DEG"):6.1f}°\n'
                        f'γ err RMSE  {rmse(gam[d] + C("DIVE_ANGLE_DEG")):6.1f}°', 'lower left')
    ax[1].legend(fontsize=7, loc='upper right')
    for a in ax:
        a.grid(alpha=.25)
    fig.suptitle(f'Fig 2  Dive profile — {title}', fontsize=11, x=.02, ha='left')
    _save(fig, path, pdf)


def fig3_attitude(R, path, title, pdf, st):
    L, t = R.L, R.t
    gm = np.where(R.mode != '', 1.0, np.nan)      # 命令只在 GUIDED 中有意義
    fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True, layout='constrained')
    pc, th = L.num('pitch_cmd') * gm, L.num('theta_deg')
    ax[0].plot(t, pc, color=C_CMD, label='pitch cmd (companion)')
    if L.has('fc_pitch_tgt_deg'):
        ax[0].plot(t, L.num('fc_pitch_tgt_deg'), color='#FF9896', lw=.8, label='FC pitch target')
    ax[0].plot(t, th, color=C_ACT, label='pitch actual')
    ax[0].set_ylabel('Pitch (deg)')
    dd = R.m('PRE_DIVE', 'DIVE')
    if dd.any():
        st['pitch_rmse_dive'] = rmse((pc - th)[R.m('DIVE')])
        st['pitch_maxerr_dive'] = nanmax(np.abs(pc - th)[R.m('DIVE')])
        stat_box(ax[0], f'PRE_DIVE+DIVE RMSE(cmd−act) {rmse((pc - th)[dd]):5.2f}°\n'
                        f'DIVE RMSE {st["pitch_rmse_dive"]:5.2f}°  max {st["pitch_maxerr_dive"]:5.2f}°',
                 'lower left')
    hc = L.num('heading_cmd') * gm
    hd = L.num('yaw_deg') if L.has('yaw_deg') else L.num('chi_deg')
    base = hd[np.isfinite(hd)][0] if np.isfinite(hd).any() else 0
    ax[1].plot(t, base + wrap180(hc - base), color=C_CMD, label='heading cmd')
    ax[1].plot(t, base + wrap180(hd - base), color=C_ACT, label='heading')
    if L.has('brg_hdg_want'):
        ax[1].plot(t, base + wrap180(L.num('brg_hdg_want') - base), color='#FF9896', lw=.8,
                   label='bridge hdg want')
    ax[1].set_ylabel('Heading (deg)')
    s = R.m('SEARCH')
    if s.any():
        he = wrap180(hc - hd)
        st['hdg_err_search_end'] = nanmean(he[s & (t >= R.first.get('PRE_DIVE', t[-1]) - 2)
                                              & (t < R.first.get('PRE_DIVE', t[-1]))])
        stat_box(ax[1], f'SEARCH hdg err RMSE {rmse(he[s]):5.1f}°\n'
                        f'last 2 s before PRE_DIVE {f1(st["hdg_err_search_end"])}°', 'lower left')
    roll = L.num('roll_deg')
    ax[2].plot(t, roll, color=C_ACT, label='roll')
    for y, st_ in ((C('ROLL_PROT_FULL_DEG'), '--'), (C('ROLL_PROT_ZERO_DEG'), ':')):
        ax[2].axhline(y, color=C_CMD, ls=st_, lw=.8)
        ax[2].axhline(-y, color=C_CMD, ls=st_, lw=.8)
    ax[2].set_ylabel('Roll (deg)')
    ax[2].set_xlabel('Time from GUIDED (s)')
    d = R.m('DIVE')
    if d.any():
        st['roll_max_dive'] = nanmax(np.abs(roll[d]))
        stat_box(ax[2], f'DIVE max |roll| {st["roll_max_dive"]:5.1f}°\n'
                        f'roll prot {C("ROLL_PROT_FULL_DEG"):g}/{C("ROLL_PROT_ZERO_DEG"):g}°', 'lower left')
    for a in ax:
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=8, loc='upper right')
    fig.suptitle(f'Fig 3  Attitude tracking — {title}', fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


def fig4_image(R, path, title, pdf, st):
    L, t = R.L, R.t
    a0, a1 = R.window(3.0, 1.0)
    w = (t >= a0) & (t <= a1)
    d = R.m('DIVE')
    dn = np.where(d, 1.0, np.nan)        # 虛擬畫面只在 DIVE 投影, 其他段固定在畫面中心
    u, v = L.num('virt_bbox_u') * dn, L.num('virt_bbox_v') * dn
    eu = np.degrees(np.arctan((u - C('CX_1X')) / C('FX_1X')))
    ev = np.degrees(np.arctan((v - C('CY_1X')) / C('FY_1X')))
    pu, pv = L.num('pid_u_err_deg') * dn, L.num('pid_v_err_deg') * dn
    fig = plt.figure(figsize=(13, 7.5), layout='constrained')
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1])
    a = fig.add_subplot(gs[0, 0])
    b = fig.add_subplot(gs[1, 0], sharex=a)
    c = fig.add_subplot(gs[:, 1])
    for ax_, e_bb, e_pid, lab in ((a, eu, pu, 'Lateral (u, +right)'), (b, ev, pv, 'Vertical (v, +down)')):
        ax_.plot(t[w], e_bb[w], color='#7F7F7F', lw=.9, label='target offset from image center')
        ax_.plot(t[w], e_pid[w], color=C_CMD, lw=1.4, label='PID error (vs setpoint)')
        ax_.axhline(0, color='k', lw=.6)
        ax_.set_ylabel(f'{lab}\n(deg)')
        shade_modes(ax_, R.segs)
        ax_.grid(alpha=.25)
        ax_.legend(fontsize=8, loc='upper right')
        ax_.set_xlim(a0, a1)
    if d.any():
        half = d & (t >= (R.first['DIVE'] + nanmax(t[d])) / 2)
        st['u_rmse'] = rmse(pu[d])
        st['v_rmse'] = rmse(pv[d])
        st['u_mean_2nd'] = nanmean(pu[half])
        st['v_mean_2nd'] = nanmean(pv[half])
        st['u_final'] = float(pu[d][np.isfinite(pu[d])][-1]) if np.isfinite(pu[d]).any() else float('nan')
        st['v_final'] = float(pv[d][np.isfinite(pv[d])][-1]) if np.isfinite(pv[d]).any() else float('nan')
        st['u_osc_hz'] = _osc_rate(t[d], pu[d])
        st['v_osc_hz'] = _osc_rate(t[d], pv[d])
        stat_box(a, f'DIVE RMSE {st["u_rmse"]:5.2f}°  2nd-half mean {st["u_mean_2nd"]:+5.2f}°\n'
                    f'final {st["u_final"]:+5.2f}°  zero-cross {st["u_osc_hz"]:.2f} Hz', 'lower left')
        stat_box(b, f'DIVE RMSE {st["v_rmse"]:5.2f}°  2nd-half mean {st["v_mean_2nd"]:+5.2f}°\n'
                    f'final {st["v_final"]:+5.2f}°  zero-cross {st["v_osc_hz"]:.2f} Hz', 'lower left')
    b.set_xlabel('Time from GUIDED (s)')
    W, H = C('IMG_W'), C('IMG_H')
    c.add_patch(plt.Rectangle((0, 0), W, H, fill=False, ec='k'))
    c.plot([W / 2], [H / 2], '+', color='k', ms=14)
    k = w & np.isfinite(u)
    if k.any():
        sc = c.scatter(u[k], v[k], c=t[k], s=6, cmap='viridis')
        fig.colorbar(sc, ax=c, label='Time from GUIDED (s)', shrink=.7)
        if L.has('virt_u_sp'):
            c.plot(L.num('virt_u_sp')[k & d], L.num('virt_v_sp')[k & d], '.', ms=2,
                   color=C_CMD, label='PID setpoint')
            c.legend(fontsize=8, loc='lower right')
    c.set_xlim(-50, W + 50)
    c.set_ylim(H + 50, -50)
    c.set_aspect('equal')
    c.set(xlabel='u (px)', ylabel='v (px)', title='Virtual target in image (control frame 1x)')
    a.set_title('Virtual image error (DIVE only — projection is active only in DIVE)')
    fig.suptitle(f'Fig 4  Image-plane error — {title}', fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


def _osc_rate(t, e, dead=0.3):
    """|e| > dead 的過零次數 / 秒 (大略代表擺盪頻率 ×2)."""
    ok = np.isfinite(e) & (np.abs(e) > dead)
    if ok.sum() < 5 or (t[-1] - t[0]) <= 0:
        return 0.0
    sg = np.sign(e[ok])
    return float(np.sum(sg[1:] != sg[:-1]) / (t[-1] - t[0]) / 2.0)


def fig5_pid(R, path, title, pdf):
    L, t = R.L, R.t
    a0, a1 = R.window(1.0, 1.0)
    w = (t >= a0) & (t <= a1)
    dn = np.where(R.m('DIVE'), 1.0, np.nan)
    fig, ax = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True, layout='constrained')
    for a, pre, out, lab, gains in (
            (ax[0], 'pid_lat', 'pid_dpsi_deg', 'Lateral → heading / rudder',
             (C('PID_LAT_KP'), C('PID_LAT_KI'), C('PID_LAT_KD'))),
            (ax[1], 'pid_lon', 'pid_dtheta_deg', 'Longitudinal → pitch',
             (C('PID_LON_KP'), C('PID_LON_KI'), C('PID_LON_KD')))):
        for k, col in (('p', '#1F77B4'), ('i', '#FF7F0E'), ('d', '#2CA02C')):
            a.plot(t[w], (L.num(f'{pre}_{k}') * dn)[w], color=col, lw=1, label=k.upper())
        a.plot(t[w], (L.num(out) * dn)[w], color='k', lw=1.5, label=f'output {out}')
        a.axhline(0, color='k', lw=.5)
        a.set_ylabel(lab + '\n(deg)')
        stat_box(a, f'KP {gains[0]:g}  KI {gains[1]:g}  KD {gains[2]:g}', 'upper left')
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=8, loc='upper right', ncol=4)
        a.set_xlim(a0, a1)
    ax[1].set_xlabel('Time from GUIDED (s)')
    fig.suptitle(f'Fig 5  DIVE PID terms — {title}', fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


def fig6_rudder(R, path, title, pdf, st):
    L, t = R.L, R.t
    a0, a1 = R.window(1.0, 2.0)
    w = (t >= a0) & (t <= a1)
    fig, ax = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True, layout='constrained')
    raw, cmd, sent = L.num('rudder_raw_deg'), L.num('rudder_cmd_deg'), L.num('rudder_sent_deg')
    lim = L.num('rudder_limit_deg')
    ax[0].plot(t[w], raw[w], color='#BBBBBB', label='raw (before limit)')
    ax[0].plot(t[w], cmd[w], color=C_CMD, lw=1, label='cmd')
    ax[0].plot(t[w], sent[w], color=C_ACT, lw=1.4, label='sent to FC')
    if np.isfinite(lim[w]).any():
        ax[0].plot(t[w], lim[w], color='k', ls='--', lw=.8, label='± limit')
        ax[0].plot(t[w], -lim[w], color='k', ls='--', lw=.8)
    ax[0].set_ylabel('Rudder (deg)')
    for k, col, lab in (('lim_by_airspeed', '#FF7F0E', 'limited by airspeed'),
                        ('lim_by_roll', '#D62728', 'limited by roll')):
        f = L.num(k)
        on = w & (f > 0)
        if on.any():
            ax[0].fill_between(t, -1e3, 1e3, where=on, color=col, alpha=.12, label=lab)
    yl = nanmax(np.abs(np.r_[raw[w], sent[w], lim[w]]))
    if np.isfinite(yl):
        ax[0].set_ylim(-yl * 1.15 - 1, yl * 1.15 + 1)
    r = L.num('r_dps')
    ax[1].plot(t[w], r[w], color=C_ACT, label='yaw rate r')
    ax[1].set_ylabel('Yaw rate (deg/s)')
    ax[2].plot(t[w], L.num('roll_deg')[w], color=C_ACT, label='roll')
    ax[2].plot(t[w], (L.num('pid_u_err_deg') * np.where(R.m('DIVE'), 1.0, np.nan))[w], color=C_CMD, lw=1, label='lateral image err (deg)')
    ax[2].set_ylabel('deg')
    ax[2].set_xlabel('Time from GUIDED (s)')
    d = R.m('DIVE')
    if d.any() and np.isfinite(sent[d]).any():
        L_ = np.where(np.isfinite(lim), lim, C('RUDDER_LIMIT_DEG'))
        sat = np.abs(sent[d]) >= L_[d] - 0.5
        st['rud_sat_pct'] = 100.0 * sat.mean()
        st['rud_max'] = nanmax(np.abs(sent[d]))
        st['rud_lim_roll_pct'] = 100.0 * np.nanmean(L.num('lim_by_roll')[d] > 0)
        stat_box(ax[0], f'DIVE max |sent| {st["rud_max"]:5.1f}°\n'
                        f'at limit       {st["rud_sat_pct"]:5.1f} %\n'
                        f'roll-limited   {st["rud_lim_roll_pct"]:5.1f} %', 'lower left')
    for a in ax:
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=8, loc='upper right', ncol=3)
        a.set_xlim(a0, a1)
    fig.suptitle(f'Fig 6  Rudder loop (DIVE) — {title}', fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


def fig7_traj(R, path, title, pdf):
    L, t = R.L, R.t
    fig, axs = plt.subplots(1, 2, figsize=(14, 8.5), layout='constrained',
                            gridspec_kw=dict(width_ratios=[1, 1.15]))
    an, ae = L.num('ac_n'), L.num('ac_e')
    tn, te = L.num('tgt_n'), L.num('tgt_e')
    g = R.mode != ''
    a0, a1 = R.window(2.0, 1.0)
    for ax, zoom in ((axs[0], False), (axs[1], True)):
        w = (t >= a0) & (t <= a1) if zoom else np.ones(len(t), bool)
        if not zoom:
            ax.plot(ae[~g], an[~g], color='#CCCCCC', lw=1, label='aircraft (not GUIDED)')
        for m, c in MODE_COLOR.items():
            k = w & (R.mode == m)
            if k.any():
                ax.plot(ae[k], an[k], '.', ms=2.5 if not zoom else 4, color=c, label=f'aircraft {m}')
        ax.plot(te[g & w], tn[g & w], color=C_TGT, lw=2, ls='--', label='virtual target')
        for m in ('PRE_DIVE', 'DIVE', 'PULLOUT', 'ABORT'):
            if m in R.first:
                i = np.argmin(np.abs(t - R.first[m]))
                ax.plot(ae[i], an[i], 'o', mfc='none', mec='k', ms=8)
                ax.plot(te[i], tn[i], 's', mfc='none', mec=C_TGT, ms=7)
                ax.plot([ae[i], te[i]], [an[i], tn[i]], color='#999999', lw=.6, ls=':')
                ax.annotate(f'{m} t={t[i]:.1f}s', (ae[i], an[i]), textcoords='offset points',
                            xytext=(8, 4), fontsize=8)
        ax.plot([0], [0], '*', color='k', ms=14, label='virtual start (origin)')
        ax.set(xlabel='East (m)', ylabel='North (m)')
        ax.grid(alpha=.25)
        if zoom:
            xs = np.r_[ae[w], te[g & w]]
            xs = xs[np.isfinite(xs)]
            if len(xs):
                c0, span = (xs.max() + xs.min()) / 2, max(xs.max() - xs.min(), 20) * 0.6
                ax.set_xlim(c0 - span, c0 + span)
            ax.set_title('(b) PRE_DIVE → PULLOUT zoom  (East axis exaggerated, not to scale)')
        else:
            ax.set_aspect('equal', adjustable='datalim')
            ax.set_title('(a) Top view (○ aircraft, □ target at the same instant)')
            ax.legend(fontsize=8, loc='best')
    fig.suptitle(f'Fig 7  Trajectory — {title}', fontsize=11, x=.02, ha='left')
    _save(fig, path, pdf)


def fig8_miss(R, path, title, pdf, st):
    L, t = R.L, R.t
    a0, a1 = R.window(1.0, 1.0)
    w = (t >= a0) & (t <= a1)
    # 只畫 DIVE: 平飛 / 推頭時直線外推的落點在幾公里外, 沒有意義
    dn = np.where(R.m('DIVE'), 1.0, np.nan)
    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True, layout='constrained')
    mt, ml, mc = (L.num('miss_pred_m') * dn, L.num('miss_pred_lon_m') * dn,
                  L.num('miss_pred_lat_m') * dn)
    ax[0].plot(t[w], mt[w], color='k', label='|miss|')
    ax[0].plot(t[w], ml[w], color='#1F77B4', label='along-track (+ = long / overshoot)')
    ax[0].plot(t[w], mc[w], color='#FF7F0E', label='cross-track (+ = right)')
    ax[0].axhline(0, color='k', lw=.5)
    ax[0].set_ylabel('Predicted miss (m)')
    d = R.m('DIVE') & np.isfinite(mt)
    if d.any():
        i = np.where(d)[0][-1]
        st['miss_end'] = (float(mt[i]), float(ml[i]), float(mc[i]))
        stat_box(ax[0], f'end of DIVE: |miss| {mt[i]:6.1f} m\n'
                        f'  along {ml[i]:+6.1f} m  cross {mc[i]:+6.1f} m', 'upper left')
    ax[1].plot(t[w], (L.num('impact_t_fall_s') * dn)[w], color=C_ACT, label='time to impact (straight-line)')
    ax[1].set_ylabel('s')
    ax[1].set_xlabel('Time from GUIDED (s)')
    for a in ax:
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=8, loc='upper right')
        a.set_xlim(a0, a1)
    fig.suptitle(f'Fig 8  Predicted miss in DIVE (straight-line extrapolation) — {title}',
                 fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


def fig9_gimbal(R, path, title, pdf, st):
    L, t = R.L, R.t
    if not (L.has('vis_link') or L.has('vis_frames')):
        return False
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True, layout='constrained')
    link = L.num('vis_link')
    ax[0].plot(t, link, color=C_ACT, drawstyle='steps-post', label='UART link')
    ax[0].plot(t, L.num('vis_valid') * .9, color='#2CA02C', drawstyle='steps-post',
               label='target detected (×0.9)')
    ax[0].set_ylim(-.1, 1.2)
    ax[0].set_ylabel('0 / 1')
    gm = L.num('vis_gimbal_mode')
    req = L.num('gimbal_mode_req')
    ax[1].plot(t, gm, color=C_ACT, drawstyle='steps-post', label='gimbal mode (reported)')
    ax[1].plot(t, req, color=C_CMD, ls='--', drawstyle='steps-post', label='mode requested')
    ax[1].set_ylabel('Mode key')
    gp = L.num('vis_gimbal_pitch')
    ax[2].plot(t, gp, color=C_ACT, label='gimbal pitch')
    ax[2].axhline(C('GIMBAL_LOCK_PITCH_DEG'), color=C_REF, ls='--', lw=.8, label='lock −30°')
    ax[2].set_ylabel('Gimbal pitch (deg)')
    ax[3].plot(t, L.num('vis_err_u_deg'), '.-', ms=3, lw=.8, color='#FF7F0E', label='camera err u (deg)')
    ax[3].plot(t, L.num('vis_err_v_deg'), '.-', ms=3, lw=.8, color='#9467BD', label='camera err v (deg)')
    dn = np.where(R.m('DIVE'), 1.0, np.nan)
    ax[3].plot(t, L.num('pid_u_err_deg') * dn, color='#FF7F0E', ls=':', label='virtual PID err u')
    ax[3].plot(t, L.num('pid_v_err_deg') * dn, color='#9467BD', ls=':', label='virtual PID err v')
    ax[3].set_ylabel('deg')
    ax[3].set_xlabel('Time from GUIDED (s)')
    g = R.mode != ''
    st['vis_link_pct'] = 100.0 * np.nanmean(link[g] > 0) if g.any() else float('nan')
    for m in ('SEARCH', 'PRE_DIVE', 'DIVE'):
        k = R.m(m) & (link > 0)
        if k.any():
            st[f'vis_det_{m}'] = 100.0 * np.nanmean(L.num('vis_valid')[k] > 0)
    lk = L.num('gimbal_locked')
    if 'PRE_DIVE' in R.first and np.isfinite(lk).any():
        k = (t >= R.first['PRE_DIVE']) & (lk > 0)
        st['gimbal_lock_s'] = float(t[k][0] - R.first['PRE_DIVE']) if k.any() else float('nan')
    txt = f'link {f1(st["vis_link_pct"])} % of GUIDED\n' + '\n'.join(
        f'det {m:<8s} {f1(st.get("vis_det_" + m))} %' for m in ('SEARCH', 'PRE_DIVE', 'DIVE'))
    stat_box(ax[0], txt, 'lower left')
    for a in ax:
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=8, loc='upper right', ncol=2)
    fig.suptitle(f'Fig 9  Gimbal UART (log only, not used for control) — {title}',
                 fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)
    return True


def fig10_system(R, path, title, pdf, st):
    L, t = R.L, R.t
    fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True, layout='constrained')
    bv = L.num('batt_v')
    bv_ok = np.where(bv > 1.0, bv, np.nan)
    ax[0].plot(t, bv_ok, color=C_ACT, label='battery V')
    ax[0].set_ylabel('Battery (V)')
    dt = L.num('dt') * 1000
    ax[1].plot(t, dt, color=C_ACT, lw=.8, label='loop period')
    ax[1].axhline(20, color=C_REF, ls='--', lw=.8, label='50 Hz = 20 ms')
    ax[1].plot(t, L.num('ctrl_ms'), color=C_CMD, lw=.8, label='control compute')
    ax[1].set_ylabel('ms')
    ax[2].plot(t, L.num('nz_g'), color=C_ACT, label='nz (g)')
    ax[2].set_ylabel('nz (g)')
    ax[2].set_xlabel('Time from GUIDED (s)')
    st['batt_min'] = nanmin(bv_ok)
    st['batt_bad_pct'] = 100.0 * np.mean(np.isfinite(bv) & (bv <= 1.0)) if L.has('batt_v') else 0.0
    st['dt_med'] = float(np.nanmedian(dt)) if np.isfinite(dt).any() else float('nan')
    st['dt_slow_pct'] = 100.0 * np.nanmean(dt > 40) if np.isfinite(dt).any() else float('nan')
    st['dt_max'] = nanmax(dt)
    stat_box(ax[1], f'median {f1(st["dt_med"])} ms  max {f1(st["dt_max"])} ms\n'
                    f'> 40 ms: {f1(st["dt_slow_pct"], 2)} %', 'upper left')
    for a in ax:
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=8, loc='upper right')
    fig.suptitle(f'Fig 10  System health — {title}', fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


# ─────────────────────────── 自動檢查 ───────────────────────────
def analyse(R, st, events, gps_lines):
    """回傳 (時間線, 數字, 檢查) 三段文字. 檢查用 ✓ / ⚠ / ✗ 開頭."""
    L, t = R.L, R.t
    agl, V = L.num('agl'), L.num('V')
    tl = []
    for m, a, b in R.segs:
        i0 = np.argmin(np.abs(t - a))
        tl.append(f'  {a:7.2f} ~ {b:7.2f} s  {m:<8s} ({b - a:5.2f} s)  進入時 AGL {f1(agl[i0])} m'
                  f'  V {f1(V[i0])} m/s  離目標 {f1(L.num("ground_m")[i0], 0)} m')
    ev_lines = [f'  [{tt - (R.t_on + 0):7.2f}] {s}' for tt, s in events]
    nums, chk = [], []

    def ok(s):
        chk.append('✓ ' + s)

    def warn(s):
        chk.append('⚠ ' + s)

    def bad(s):
        chk.append('✗ ' + s)

    # 1. 撥 GUIDED 時
    g0 = R.mode != ''
    if g0.any():
        i = np.where(g0)[0][0]
        nums.append(f'撥 GUIDED: AGL {f1(agl[i])} m, 空速 {f1(V[i])} m/s, 離虛擬起點 '
                    f'{f1(np.hypot(L.num("ac_n")[i], L.num("ac_e")[i]), 0)} m')
        if np.isfinite(agl[i]) and agl[i] < 20:
            chk.append(f'· 撥 GUIDED 時 AGL {agl[i]:.0f} m = 地面測試, 不檢查飛行項目')
        elif abs(agl[i] - C('CRUISE_AGL_M')) > 20:
            warn(f'撥 GUIDED 時 AGL {agl[i]:.0f} m, 跟巡航 {C("CRUISE_AGL_M"):g} m 差超過 20 m '
                 f'→ 推頭點的幾何會跟設計不同 (看 Mission Planner 航點高度)')
        else:
            ok(f'撥 GUIDED 高度 {agl[i]:.0f} m (設計 {C("CRUISE_AGL_M"):g} m)')
    if 'ABORT' in R.first:
        why = [s for _, s in events if 'ABORT' in s]
        bad('這趟有 ABORT: ' + (why[0] if why else '看 events.txt'))
    if 'PRE_DIVE' not in R.first:
        refuse = [x for _, x in events if '不啟動' in x]
        if refuse:
            chk.append('· ' + refuse[0])
        warn('這趟沒有進 PRE_DIVE (沒推頭) → 看 events.txt 是「拒絕啟動」還是提早切出 GUIDED')
        return tl, ev_lines, nums, chk

    # 2. SEARCH 對準
    he = st.get('hdg_err_search_end', float('nan'))
    if np.isfinite(he):
        nums.append(f'推頭前 2 s 航向誤差 {he:+.1f}°')
        (ok if abs(he) < 5 else warn)(
            f'推頭前航向誤差 {he:+.1f}°' + ('' if abs(he) < 5 else
                                       ' → 還沒對準就推頭: 撥 GUIDED 再早一點 / 遠一點, 或 GUIDED_P 轉向太慢'))

    # 3. PRE_DIVE 推頭時間
    if 'DIVE' in R.first:
        tp = R.first['DIVE'] - R.first['PRE_DIVE']
        nums.append(f'PRE_DIVE 推頭用時 {tp:.2f} s')
        if tp > 4.0:
            warn(f'推頭到 −{C("DIVE_ANGLE_DEG"):g}° 花 {tp:.1f} s (>4 s) → 飛控 pitch 迴路偏慢: '
                 f'看飛控 pitch 角速度上限 / pitch 增益 (PTCH_RATE_MAX、PTCH2SRV_*) 或 '
                 f'PREDIVE_HANDOFF_PITCH_TOL_DEG={C("PREDIVE_HANDOFF_PITCH_TOL_DEG"):g}')
        else:
            ok(f'推頭用時 {tp:.1f} s')

    d = R.m('DIVE')
    if d.any():
        gam = L.num('gamma_deg')
        late = d & (t >= R.first['DIVE'] + 1.0)
        gm = nanmean(gam[late]) if late.any() else nanmean(gam[d])
        nums.append(f'DIVE 時長 {nanmax(t[d]) - R.first["DIVE"]:.2f} s, 平均航跡角 γ {gm:.1f}° '
                    f'(設計 −{C("DIVE_ANGLE_DEG"):g}°)')
        if np.isfinite(gm) and abs(gm + C('DIVE_ANGLE_DEG')) > 3:
            warn(f'實際俯衝角 γ {gm:.1f}° 跟設計 −{C("DIVE_ANGLE_DEG"):g}° 差 '
                 f'{abs(gm + C("DIVE_ANGLE_DEG")):.1f}° → 看 pitch 追蹤 (Fig 3) / ALPHA_DIVE_DEG')
        elif np.isfinite(gm):
            ok(f'俯衝角 γ {gm:.1f}°')
        pr = st.get('pitch_rmse_dive', float('nan'))
        if np.isfinite(pr):
            nums.append(f'DIVE pitch 追蹤 RMSE {pr:.2f}°, 最大差 {f1(st.get("pitch_maxerr_dive"), 1)}°')
            if pr > 3:
                warn(f'pitch 追蹤 RMSE {pr:.1f}° 偏大 → 飛控 pitch 迴路跟不上命令 '
                     f'(先調飛控, 不要先加 PID_LON 增益)')
            else:
                ok(f'pitch 追蹤 RMSE {pr:.1f}°')
        vmax = nanmax(V[d])
        nums.append(f'DIVE 最大空速 {vmax:.1f} m/s (上限 {C("PULLOUT_VMAX_MPS"):g})')
        pull_by_v = any('PULLOUT' in s and ('超速' in s or 'Vmax' in s or 'VMAX' in s) for _, s in events)
        if vmax >= C('PULLOUT_VMAX_MPS') - 0.5 or pull_by_v:
            warn(f'DIVE 空速到 {vmax:.1f} m/s, 因超速提前拉平 (沒到 {C("PULLOUT_AGL_M"):g} m) '
                 f'→ 俯衝角太陡 / 怠速油門太大 (THR_MIN) / 撥 GUIDED 高度太高')
        elif vmax >= C('PULLOUT_VMAX_MPS') - 5:
            warn(f'DIVE 空速 {vmax:.1f} m/s 接近上限 {C("PULLOUT_VMAX_MPS"):g}')
        else:
            ok(f'DIVE 最大空速 {vmax:.1f} m/s')

        # 影像誤差
        for ax_, nm, kp, ki in (('u', '橫向', 'PID_LAT_KP', 'PID_LAT_KI'),
                                ('v', '縱向', 'PID_LON_KP', 'PID_LON_KI')):
            r_, mn, osc, fin = (st.get(f'{ax_}_rmse'), st.get(f'{ax_}_mean_2nd'),
                                st.get(f'{ax_}_osc_hz'), st.get(f'{ax_}_final'))
            if r_ is None or not np.isfinite(r_):
                continue
            nums.append(f'{nm}影像誤差 DIVE RMSE {r_:.2f}°, 後半段平均 {mn:+.2f}°, 結束 {fin:+.2f}°, '
                        f'擺盪 {osc:.2f} Hz')
            msg = []
            if osc > 0.8 and r_ > 1.0:
                msg.append(f'誤差來回擺盪 ({osc:.1f} Hz) → {kp}={C(kp):g} 可能太大')
            if abs(mn) > 1.5:
                msg.append(f'後半段一直偏 {mn:+.1f}° 沒收斂 → {ki}={C(ki):g} 可能不夠 '
                           + ('或方向舵不夠力 (Fig 6)' if ax_ == 'u' else '或 pitch 追不上 (Fig 3)'))
            if r_ > 5:
                msg.append(f'RMSE {r_:.1f}° 很大 → 先看推頭時是否已經對準 (Fig 4 起點)')
            (warn if msg else ok)(f'{nm}影像誤差 RMSE {r_:.1f}°' + ('：' + '；'.join(msg) if msg else ''))

        # 方向舵
        sp = st.get('rud_sat_pct')
        if sp is not None and np.isfinite(sp):
            nums.append(f'方向舵 DIVE 最大 {st["rud_max"]:.1f}°, 打到限幅 {sp:.0f} % 時間, '
                        f'被 roll 保護限制 {st["rud_lim_roll_pct"]:.0f} %')
            if sp > 20:
                warn(f'方向舵 {sp:.0f} % 時間打滿 → 側偏太大 (推頭前沒對準) 或 K_LAT_TO_RUDDER×PID_LAT 太大; '
                     f'若橫向誤差也收斂不了 = 方向舵權限不夠')
            else:
                ok(f'方向舵打滿 {sp:.0f} % 時間')
            if st['rud_lim_roll_pct'] > 10:
                warn(f'{st["rud_lim_roll_pct"]:.0f} % 時間被 roll 保護砍方向舵 → 方向舵造成大 roll '
                     f'(ROLL_PROT_FULL/ZERO={C("ROLL_PROT_FULL_DEG"):g}/{C("ROLL_PROT_ZERO_DEG"):g}°)')
        rm = st.get('roll_max_dive')
        if rm is not None and np.isfinite(rm):
            nums.append(f'DIVE 最大 |roll| {rm:.1f}°')
            if rm > C('ROLL_PROT_FULL_DEG'):
                warn(f'DIVE roll 到 {rm:.0f}° (> {C("ROLL_PROT_FULL_DEG"):g}°) → 方向舵-滾轉耦合大, '
                     f'看 Fig 6 是否方向舵一打就滾')

        me = st.get('miss_end')
        if me:
            nums.append(f'DIVE 結束時預測落點誤差 {me[0]:.1f} m (沿航跡 {me[1]:+.1f}, 側向 {me[2]:+.1f})')
    score = [s for _, s in events if '成績' in s]
    if score:
        nums.append('程式成績: ' + score[-1].replace('★', '').strip())

    # 拉平
    for m in ('PULLOUT', 'ABORT'):
        if m in R.first:
            k = t >= R.first[m]
            amin = nanmin(agl[k])
            i0 = np.argmin(np.abs(t - R.first[m]))
            nums.append(f'{m} 開始 AGL {f1(agl[i0])} m / V {f1(V[i0])} m/s, 之後最低 AGL {f1(amin)} m')
            if m == 'PULLOUT':
                under = C('PULLOUT_AGL_M') - amin
                if amin < C('ABORT_MIN_AGL_M'):
                    bad(f'拉平後最低 {amin:.0f} m, 低於 ABORT 高度 {C("ABORT_MIN_AGL_M"):g} m '
                        f'→ 拉平太晚: PULLOUT_AGL_M 提高 或 降低俯衝速度')
                elif under > 15:
                    warn(f'拉平掉了 {under:.0f} m 才停 (最低 {amin:.0f} m) → 考慮 PULLOUT_AGL_M 提高 '
                         f'{under:.0f} m 或確認 TECS / pitch 上仰速度')
                else:
                    ok(f'拉平最低 {amin:.0f} m (設定 {C("PULLOUT_AGL_M"):g} m)')

    # 系統
    if np.isfinite(st.get('dt_med', float('nan'))):
        nums.append(f'控制迴路週期 中位數 {st["dt_med"]:.1f} ms, 最大 {st["dt_max"]:.0f} ms, '
                    f'>40 ms 佔 {st["dt_slow_pct"]:.2f} %')
        if st['dt_slow_pct'] > 2 or st['dt_max'] > 200:
            warn('控制迴路有卡頓 (>40 ms 或單次 >200 ms) → Orin 負載 / 其他程式')
        else:
            ok('控制迴路 50 Hz 正常')
    if np.isfinite(st.get('batt_min', float('nan'))):
        nums.append(f'電壓最低 {st["batt_min"]:.2f} V')
    if st.get('batt_bad_pct', 0) > 5:
        warn(f'batt_v 有 {st["batt_bad_pct"]:.0f} % 讀到 ≈0 V → 第二顆電池監測沒接 / BATT 參數')
    if 'vis_link_pct' in st:
        nums.append(f'雲台 UART 連線 {f1(st["vis_link_pct"])} % (GUIDED 期間); 偵測率 '
                    + ', '.join(f'{m} {f1(st.get("vis_det_" + m))} %' for m in ('SEARCH', 'PRE_DIVE', 'DIVE')))
        if st['vis_link_pct'] < 95:
            warn(f'雲台 UART 連線只有 {st["vis_link_pct"]:.0f} % → 線材 / 對方程式')
        gl = st.get('gimbal_lock_s')
        if gl is not None:
            nums.append(f'雲台鎖定 −30° 到位: PRE_DIVE 後 {f1(gl, 2)} s')
            if not np.isfinite(gl):
                warn('PRE_DIVE 之後雲台一直沒有回報鎖定到位')
    for ln in gps_lines or []:
        nums.append('真車 GPS: ' + ln)
    return tl, ev_lines, nums, chk


# ─────────────────────────── 主程式 ───────────────────────────
def pick_latest(log_dir):
    cs = [p for p in glob.glob(os.path.join(log_dir, 'xwing_virtual_*.csv'))
          if not p.endswith('_events.csv')]
    if not cs:
        sys.exit(f'[plot] {log_dir} 裡沒有 xwing_virtual_*.csv')
    return max(cs, key=os.path.getmtime)


def runs_of(L):
    g = L.num('guided') > 0
    if not L.has('run'):
        return [1] if g.any() else []
    r = L.num('run')
    return sorted(int(x) for x in np.unique(r[g & (r > 0)]))


def make_report(log_path, run, outdir, args, bins):
    L = Log.load(log_path)
    title = os.path.splitext(os.path.basename(log_path))[0]
    R = Run(L, run, full=args.full)
    events = [(tt, s) for tt, s in read_events(log_path)
              if R.L.n and R.L.num('t')[0] - 0.5 <= tt <= R.L.num('t')[-1] + 0.5]
    os.makedirs(outdir, exist_ok=True)
    p = lambda n: os.path.join(outdir, n)
    st = {}
    sub = f'{title}  run {run}' if not args.full else f'{title}  (full log)'
    with PdfPages(p('report.pdf')) as pdf:
        fig1_overview(R, p('fig1_overview.png'), sub, pdf)
        fig2_profile(R, p('fig2_dive_profile.png'), sub, pdf)
        fig3_attitude(R, p('fig3_attitude.png'), sub, pdf, st)
        fig4_image(R, p('fig4_image_error.png'), sub, pdf, st)
        fig5_pid(R, p('fig5_pid_terms.png'), sub, pdf)
        fig6_rudder(R, p('fig6_rudder.png'), sub, pdf, st)
        fig7_traj(R, p('fig7_trajectory.png'), sub, pdf)
        fig8_miss(R, p('fig8_miss.png'), sub, pdf, st)
        if not fig9_gimbal(R, p('fig9_gimbal.png'), sub, pdf, st):
            if os.path.isfile(p('fig9_gimbal.png')):
                os.remove(p('fig9_gimbal.png'))
        fig10_system(R, p('fig10_system.png'), sub, pdf, st)
        gps_lines = []
        if bins:
            import xwing_plot_target_gps as TG
            try:
                gps_lines, _ = TG.compare(TG.load_log_basic(log_path), run, bins,
                                          p('fig11_target_gps'), title=title, pdf=pdf)
            except ValueError as exc:
                gps_lines = [f'沒畫 ({exc})']
    tl, evl, nums, chk = analyse(R, st, events, gps_lines)
    head = [f'log : {log_path}', f'趟次: run {run}' + ('  (--full 整份)' if args.full else ''),
            f'時間 0 = 撥 GUIDED (log t = {R.t_on:.2f} s)',
            f'參數: 巡航 {C("CRUISE_AGL_M"):g} m / 俯衝 {C("DIVE_ANGLE_DEG"):g}° / 拉平 '
            f'{C("PULLOUT_AGL_M"):g} m / Vmax {C("PULLOUT_VMAX_MPS"):g} m/s; '
            f'PID_LAT {C("PID_LAT_KP"):g}/{C("PID_LAT_KI"):g}/{C("PID_LAT_KD"):g}, '
            f'PID_LON {C("PID_LON_KP"):g}/{C("PID_LON_KI"):g}/{C("PID_LON_KD"):g}, '
            f'K_LAT_TO_RUDDER {C("K_LAT_TO_RUDDER"):g}'
            + ('' if CFG is not None else '  (找不到 xwing_virtual_config.py, 用內建預設值)')]
    text = ('\n'.join(head) + '\n\n【模式時間線】\n' + '\n'.join(tl)
            + '\n\n【數字】\n' + '\n'.join('  ' + x for x in nums)
            + '\n\n【自動檢查】 (✓ 正常  ⚠ 建議看  ✗ 有問題  · 說明; 只是根據 log 的提示, 最後仍以圖為準)\n'
            + '\n'.join('  ' + x for x in chk)
            + '\n\n【events.txt (這趟)】\n' + '\n'.join(evl) + '\n')
    with open(p('summary.txt'), 'w', encoding='utf-8') as f:
        f.write(text)
    return text


def main():
    ap = argparse.ArgumentParser(description='xwing_virtual 實飛 log → 報告圖 + 自動檢查')
    ap.add_argument('log', nargs='?', help='xwing_virtual_*.csv (不給就要用 --latest)')
    ap.add_argument('--latest', action='store_true', help='用 flight_logs/ 裡最新的 log')
    ap.add_argument('--log-dir', default=os.path.join(HERE, 'flight_logs'))
    ap.add_argument('--run', type=int, default=None, help='只畫第幾趟 (預設每一趟都畫)')
    ap.add_argument('--full', action='store_true', help='不分趟, 畫整份 log (地面測試)')
    ap.add_argument('-o', '--outdir', default=os.path.join(HERE, 'plot_results'),
                    help='輸出根目錄 (預設 plot_results/)')
    ap.add_argument('--car-bin-dir', default=None, help='車子 BIN 資料夾 → 加畫 Fig 11')
    ap.add_argument('--car-bin', action='append', default=[], help='直接指定車子 BIN (可多次)')
    args = ap.parse_args()

    if args.latest or not args.log:
        if not args.latest:
            ap.error('請給 log 檔, 或加 --latest')
        args.log = pick_latest(args.log_dir)
    if not os.path.isfile(args.log):
        cand = os.path.join(args.log_dir, args.log)
        if os.path.isfile(cand):
            args.log = cand
        else:
            sys.exit(f'[plot] 找不到 {args.log}')
    print(f'[plot] log: {args.log}')

    bins = []
    if args.car_bin_dir or args.car_bin:
        import xwing_plot_target_gps as TG
        bins = TG.load_bins(TG.find_bins(args.car_bin_dir, args.car_bin))
        if not bins:
            print('[plot] ⚠ 沒有可用的車子 BIN, 不畫 Fig 11')

    L = Log.load(args.log)
    stem = os.path.splitext(os.path.basename(args.log))[0]
    if args.full:
        runs = [0]
    else:
        runs = [args.run] if args.run else runs_of(L)
        if not runs:
            sys.exit('[plot] 這份 log 沒有任何 GUIDED 段 (地面測試請加 --full)')
    for r in runs:
        od = os.path.join(args.outdir, stem, 'full' if args.full else f'run{r}')
        try:
            text = make_report(args.log, r, od, args, bins)
        except ValueError as exc:
            print(f'[plot] 第 {r} 趟跳過: {exc}')
            continue
        print('\n' + '=' * 78)
        print(text)
        print(f'[plot] 圖 → {od}')


if __name__ == '__main__':
    main()

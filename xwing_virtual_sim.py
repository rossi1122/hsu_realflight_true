#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xwing_virtual_sim.py — xwing_virtual 任務剖面模擬 + 各段距離計算
═══════════════════════════════════════════════════════════════════
不用任何硬體 (無 ROS / 飛機). 用【實飛同一套】狀態機 (dive_guidance)
+ 虛擬目標 (virtual_source) + 參數 (xwing_virtual_config / _common),
接上一個點質量飛機模型, 算出:

  · SEARCH / PRE_DIVE / DIVE / PULLOUT 各段的時間、水平距離、高度、速度
  · 俯衝末速 (會不會撞到 60 m/s 上限)、拉平最低高度
  · 整條軌跡 CSV (含 GPS), 給 xwing_virtual_route.py 畫路線圖

飛機模型 (比 test_logic_sim 細, 但仍是估算, 不是氣動模擬):
  · pitch 追命令: 一階 τ=0.25 s + 俯仰角速度上限 Q_MAX (預設 20°/s)
  · 航跡角 γ 追 (pitch − 攻角): τ=0.5 s, 且受過載 n_max 限制
  · 速度: dV/dt = T/m − k·V² − g·sinγ,  k = g / (L/D · V_cruise²)
        L/D 預設 8 (巡航阻力 = 重量/8); --ld 0 = 無阻力 (速度上界)
  · SEARCH/DONE: 飛控 TECS 定高定速 + 43002 航向 (坡度上限 22°)
  · DIVE: 怠速; 方向舵依 PID dpsi 修航向

用法:
  python3 xwing_virtual_sim.py                     # 移動目標, 撥 GUIDED 時距目標 613 m (WP3→WP4)
  python3 xwing_virtual_sim.py --static
  python3 xwing_virtual_sim.py --guided-dist 900   # 改撥 GUIDED 的距離
  python3 xwing_virtual_sim.py --ld 0              # 無阻力 → 最壞末速
  python3 xwing_virtual_sim.py --sweep             # 掃推頭距離, 看俯衝角
  python3 xwing_virtual_sim.py --csv out.csv       # 輸出軌跡
"""

import argparse
import csv
import math

from xwing_virtual_config import XwingVirtualConfig
from xwing_virtual_common import (build_gcfg, build_target, target_heading_deg,
                                  road_length_m)
from dive_guidance import MissionStateMachine
from virtual_source import VirtualSource
from virtual_target_geom import extrapolate_impact, miss_distance

G = 9.81
_R_EARTH = 6378137.0


def ned_to_gps(n, e, lat0, lon0):
    lat = lat0 + math.degrees(n / _R_EARTH)
    lon = lon0 + math.degrees(e / (_R_EARTH * math.cos(math.radians(lat0))))
    return lat, lon


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def simulate(cfg=None, moving=True, guided_dist=613.0, predive_dist=None,
             ld=8.0, q_max=20.0, n_max=2.5, dt=0.02, t_after_done=8.0,
             verbose=False):
    """跑一趟任務. 回傳 dict: traj (list of dict), phases (dict), info."""
    cfg = cfg or XwingVirtualConfig()
    start = cfg.MOVING_START_GPS if moving else cfg.TARGET_GPS
    heading = target_heading_deg(cfg, cfg.MOVING_START_GPS)
    speed = cfg.TARGET_SPEED_MPS if moving else 0.0
    if predive_dist is None:
        predive_dist = cfg.PREDIVE_DIST_M if moving else cfg.STATIC_PREDIVE_DIST_M

    gcfg = build_gcfg(cfg, moving, predive_dist)
    sm = MissionStateMachine(gcfg, verbose=verbose)
    target = build_target(cfg, moving, start, start, heading, speed, t0=0.0)
    virt = VirtualSource(cfg, gcfg, start[0], start[1], target) if verbose else \
        _quiet_virtual_source(cfg, gcfg, start, target)

    # ── 初始: 撥 GUIDED 瞬間, 飛機在目標起點「後方」guided_dist, 沿目標方位飛 ──
    #   (追擊: 飛機與目標同方向. 新場地目標往南 → 飛機從北邊往南進場)
    hr = math.radians(heading)
    n, e = -guided_dist * math.cos(hr), -guided_dist * math.sin(hr)
    h = cfg.CRUISE_AGL_M
    V = cfg.V_CRUISE_MPS
    chi = heading
    gam = 0.0
    theta = cfg.ALPHA_NOM_DEG
    vc = cfg.V_CRUISE_MPS
    k_drag = (G / (ld * vc * vc)) if ld and ld > 0 else 0.0
    thr_cruise = k_drag * vc * vc          # 巡航所需推力/質量 (m/s²)
    tmax = 0.6 * G                          # 推力上限 (ABORT 大油門)
    h_ref = h                               # TECS 高度目標
    v_ref = cfg.SEARCH_SPEED_MPS

    sm.arm()
    traj = []
    phases = {}
    mode_prev = None
    t = 0.0
    t_done = None
    miss = None
    max_V = V

    while t < 200.0:
        dive_mode = (sm.mode == sm.DIVE)
        vf = virt.get_fields({'n': n, 'e': e, 'agl': h, 'theta_deg': theta,
                              'chi_deg': chi, 'gimbal_locked': dive_mode, 't': t})
        cg, sg = math.cos(math.radians(gam)), math.sin(math.radians(gam))
        vn = V * cg * math.cos(math.radians(chi))
        ve = V * cg * math.sin(math.radians(chi))
        vd = -V * sg
        inp = {'t': t, 'guided': True, 'agl_m': h, 'airspeed_mps': V,
               'gs_mps': V * cg, 'vn': vn, 've': ve, 'vd': vd,
               'yaw_true_deg': chi, 'pitch_deg': theta, 'roll_deg': 0.0,
               'ac_n': n, 'ac_e': e, **vf}
        out = sm.update(inp)
        mode = out['mode']

        if mode != mode_prev:
            if mode_prev is not None:
                phases[mode_prev]['end'] = traj[-1]
            phases[mode] = {'start': None, 'end': None}
            mode_prev = mode
            if mode in ('PRE_DIVE', 'PULLOUT'):
                h_ref = cfg.TECS_HOLD_ALT_M          # tecs_resend
                v_ref = cfg.TECS_SPEED_MPS
        if sm.pullout_snapshot is not None and miss is None:
            sp = sm.pullout_snapshot
            i_n, i_e, tf = extrapolate_impact(sp['ac_n'], sp['ac_e'], sp['agl'],
                                              sp['vn'], sp['ve'], sp['vd'])
            if i_n is not None:
                miss = miss_distance(i_n, i_e, tf, sp['tgt_n'], sp['tgt_e'],
                                     sp['tgt_vn'] or 0.0, sp['tgt_ve'] or 0.0,
                                     sp['vn'], sp['ve'])

        row = {'t': t, 'mode': mode, 'n': n, 'e': e, 'agl': h, 'V': V,
               'gamma': gam, 'theta': theta, 'chi': chi,
               'pitch_cmd': out['pitch_cmd_deg'], 'hdg_cmd': out['heading_cmd_deg'],
               'tgt_n': vf['tgt_n'], 'tgt_e': vf['tgt_e'],
               'ground_m': vf['ground_m']}
        traj.append(row)
        if phases[mode]['start'] is None:
            phases[mode]['start'] = row

        # ══ 飛機模型 ══════════════════════════════════════════
        # 縱向: 哪一段由誰控制
        if mode in ('SEARCH', 'DONE'):
            # 飛控 TECS: 定高 + 定速
            gam_cmd = max(-15.0, min(10.0, 0.3 * (h_ref - h)))
            theta_cmd = gam_cmd + cfg.ALPHA_NOM_DEG
            thrust = max(0.0, min(tmax, thr_cruise + 0.4 * (v_ref - V)
                                  + G * math.sin(math.radians(gam))))
        else:
            theta_cmd = out['pitch_cmd_deg']
            if mode == 'DIVE' or mode == 'PULLOUT':
                thrust = 0.0                              # 怠速 / TECS 見超速收油門
            elif mode == 'ABORT':
                thrust = tmax
            else:                                        # PRE_DIVE: 空速保持油門
                thrust = max(0.0, thr_cruise * (1.0 + (cfg.V_CRUISE_MPS - V) * 0.1))
        # pitch: 一階 + 角速度限制
        dth = (theta_cmd - theta) / 0.25 * dt
        theta += max(-q_max * dt, min(q_max * dt, dth))
        # 航跡角: 追 (pitch − 攻角), 受過載限制
        alpha = cfg.ALPHA_DIVE_DEG if mode in ('PRE_DIVE', 'DIVE') else cfg.ALPHA_NOM_DEG
        gdot = (theta - alpha - gam) / 0.5
        gdot_max = math.degrees((n_max - 1.0) * G / max(V, 5.0))
        gam += max(-gdot_max, min(gdot_max, gdot)) * dt
        # 速度
        V += (thrust - k_drag * V * V - G * math.sin(math.radians(gam))) * dt
        V = max(cfg.V_STALL_MPS, V)
        max_V = max(max_V, V)

        # 橫向
        if mode in ('SEARCH', 'DONE', 'ABORT'):
            # 43002 / TECS: 坡度上限 22° 的協調轉彎
            rate_max = math.degrees(G * math.tan(math.radians(22.0)) / max(V, 5.0))
            err = _wrap(out['heading_cmd_deg'] - chi)
            if mode == 'DONE':
                err = 0.0                                # DONE 只鎖 roll=0
            chi += max(-rate_max * dt, min(rate_max * dt, err / 2.0 * dt))
        elif mode == 'DIVE':
            # 方向舵: 航向命令 = 當下航向 + dpsi (增量), 時間常數 2 s
            if not gcfg.DIVE_HOLD_HEADING:
                chi += _wrap(out['heading_cmd_deg'] - chi) / 2.0 * dt
        # PRE_DIVE / PULLOUT: roll=0 + 方向舵歸中 → 航向不變
        chi %= 360.0

        cg, sg = math.cos(math.radians(gam)), math.sin(math.radians(gam))
        n += V * cg * math.cos(math.radians(chi)) * dt
        e += V * cg * math.sin(math.radians(chi)) * dt
        h += V * sg * dt

        if mode == 'DONE' and t_done is None:
            t_done = t
        if t_done is not None and t - t_done >= t_after_done:
            break
        if h <= 0.0:
            break
        t += dt

    if mode_prev is not None and traj:
        phases[mode_prev]['end'] = traj[-1]

    return {'traj': traj, 'phases': phases, 'miss': miss, 'max_V': max_V,
            'min_agl_after_pullout': min((r['agl'] for r in traj
                                          if r['mode'] in ('PULLOUT', 'DONE')),
                                         default=None),
            'heading': heading, 'origin': start, 'predive_dist': predive_dist,
            'guided_dist': guided_dist, 'moving': moving, 'cfg': cfg,
            'ld': ld, 'q_max': q_max, 'n_max': n_max}


def _quiet_virtual_source(cfg, gcfg, start, target):
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        return VirtualSource(cfg, gcfg, start[0], start[1], target)


def path_length(traj, i0, i1):
    s = 0.0
    for a, b in zip(traj[i0:i1], traj[i0 + 1:i1 + 1]):
        s += math.hypot(b['n'] - a['n'], b['e'] - a['e'])
    return s


def phase_table(res):
    traj = res['traj']
    idx = {id(r): i for i, r in enumerate(traj)}
    rows = []
    for name in ('SEARCH', 'PRE_DIVE', 'DIVE', 'PULLOUT', 'DONE', 'ABORT'):
        p = res['phases'].get(name)
        if not p or p['start'] is None:
            continue
        a, b = p['start'], p['end']
        i0, i1 = idx[id(a)], idx[id(b)]
        horiz = path_length(traj, i0, i1)
        slant = 0.0
        for x, y in zip(traj[i0:i1], traj[i0 + 1:i1 + 1]):
            slant += math.sqrt((y['n'] - x['n']) ** 2 + (y['e'] - x['e']) ** 2
                               + (y['agl'] - x['agl']) ** 2)
        rows.append({'phase': name, 't0': a['t'], 'dur': b['t'] - a['t'],
                     'horiz_m': horiz, 'slant_m': slant,
                     'agl0': a['agl'], 'agl1': b['agl'],
                     'V0': a['V'], 'V1': b['V'],
                     'tgt_dist0': a['ground_m'], 'tgt_dist1': b['ground_m'],
                     'tgt_moved': math.hypot(b['tgt_n'] - traj[0]['tgt_n'],
                                             b['tgt_e'] - traj[0]['tgt_e'])})
    return rows


def recovery_stats(res):
    """拉平改出: PULLOUT 起點 → 航跡角回到 ≥ −2° 的水平距 / 時間 / 最低高度."""
    traj = res['traj']
    i0 = next((i for i, r in enumerate(traj) if r['mode'] == 'PULLOUT'), None)
    if i0 is None:
        return None
    i1 = next((i for i in range(i0, len(traj)) if traj[i]['gamma'] >= -2.0),
              len(traj) - 1)
    return {'horiz_m': path_length(traj, i0, i1),
            'dur': traj[i1]['t'] - traj[i0]['t'],
            'agl_end': traj[i1]['agl'],
            'agl_min': min(r['agl'] for r in traj[i0:]),
            'end': traj[i1]}


def dive_gamma_stats(res):
    g = [r['gamma'] for r in res['traj'] if r['mode'] == 'DIVE']
    if not g:
        return None
    return {'mean': sum(g) / len(g), 'min': min(g), 'max': max(g)}


def print_report(res):
    cfg = res['cfg']
    print("=" * 78)
    print(f" xwing_virtual 任務剖面  ({'移動' if res['moving'] else '靜止'}目標, "
          f"撥 GUIDED 時距目標 {res['guided_dist']:.0f} m, 推頭門檻 "
          f"{res['predive_dist']:.0f} m)")
    print(f" 模型: L/D={res['ld'] or '∞(無阻力)'}  pitch 速率 {res['q_max']:.0f}°/s  "
          f"過載 {res['n_max']:.1f} g")
    print("=" * 78)
    print(f"{'段':<9}{'起始t':>7}{'時長s':>7}{'水平距m':>9}{'航跡長m':>9}"
          f"{'AGL起→終':>14}{'空速起→終':>14}{'距目標起→終m':>16}")
    for r in phase_table(res):
        print(f"{r['phase']:<9}{r['t0']:>7.1f}{r['dur']:>7.1f}{r['horiz_m']:>9.0f}"
              f"{r['slant_m']:>9.0f}{r['agl0']:>7.0f}→{r['agl1']:<6.0f}"
              f"{r['V0']:>7.1f}→{r['V1']:<6.1f}"
              f"{r['tgt_dist0']:>8.0f}→{r['tgt_dist1']:<7.0f}")
    gs = dive_gamma_stats(res)
    print("-" * 78)
    if gs:
        print(f" DIVE 航跡角: 平均 {gs['mean']:.1f}°  範圍 {gs['min']:.1f}° ~ {gs['max']:.1f}°"
              f"  (目標 −{cfg.DIVE_ANGLE_DEG:.0f}°)")
    print(f" 最高空速 {res['max_V']:.1f} m/s (上限 {cfg.PULLOUT_VMAX_MPS:.0f})   "
          f"拉平後最低高度 {res['min_agl_after_pullout']:.0f} m "
          f"(拉平門檻 {cfg.PULLOUT_AGL_M:.0f})")
    rc = recovery_stats(res)
    if rc:
        print(f" 拉平改出 (PULLOUT 起 → 航跡角 ≥ −2°): 水平 {rc['horiz_m']:.0f} m, "
              f"{rc['dur']:.1f} s, 改平高度 {rc['agl_end']:.0f} m")
    if res['miss']:
        m = res['miss']
        print(f" 拉平瞬間沿速度外插落點 miss = {m['miss_total']:.1f} m "
              f"(沿航跡 {m['miss_lon']:+.1f}, 側向 {m['miss_lat']:+.1f})")
    print("=" * 78)


def write_csv(res, path):
    lat0, lon0 = res['origin']
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'mode', 'lat', 'lon', 'agl', 'V', 'gamma_deg', 'theta_deg',
                    'chi_deg', 'pitch_cmd', 'hdg_cmd', 'tgt_lat', 'tgt_lon',
                    'ground_m', 'n', 'e', 'tgt_n', 'tgt_e'])
        for r in res['traj']:
            la, lo = ned_to_gps(r['n'], r['e'], lat0, lon0)
            tla, tlo = ned_to_gps(r['tgt_n'], r['tgt_e'], lat0, lon0)
            w.writerow([f"{r['t']:.2f}", r['mode'], f"{la:.7f}", f"{lo:.7f}",
                        f"{r['agl']:.1f}", f"{r['V']:.1f}", f"{r['gamma']:.2f}",
                        f"{r['theta']:.2f}", f"{r['chi']:.1f}",
                        r['pitch_cmd'], r['hdg_cmd'], f"{tla:.7f}", f"{tlo:.7f}",
                        f"{r['ground_m']:.1f}", f"{r['n']:.1f}", f"{r['e']:.1f}",
                        f"{r['tgt_n']:.1f}", f"{r['tgt_e']:.1f}"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--static', action='store_true', help='靜止目標 (預設移動)')
    ap.add_argument('--guided-dist', type=float, default=613.0,
                    help='撥 GUIDED 時飛機在目標起點後方幾公尺 (預設 613 = 任務規劃 WP3→WP4)')
    ap.add_argument('--predive-dist', type=float, default=None,
                    help='推頭門檻 m (預設用 config)')
    ap.add_argument('--ld', type=float, default=8.0,
                    help='升阻比 (阻力模型), 0 = 無阻力 (預設 8)')
    ap.add_argument('--q-max', type=float, default=20.0,
                    help='俯仰角速度上限 °/s (預設 20)')
    ap.add_argument('--n-max', type=float, default=2.5,
                    help='拉起過載上限 g (預設 2.5)')
    ap.add_argument('--sweep', action='store_true',
                    help='掃推頭距離 380~480 m, 印 DIVE 平均航跡角')
    ap.add_argument('--csv', type=str, default=None, help='輸出軌跡 CSV')
    ap.add_argument('--verbose', action='store_true', help='印狀態機訊息')
    args = ap.parse_args()

    cfg = XwingVirtualConfig()
    moving = not args.static
    print(f"[sim] 目標起點 {cfg.MOVING_START_GPS}  方位 "
          f"{target_heading_deg(cfg, cfg.MOVING_START_GPS):.1f}°  "
          f"起點→航向點 {road_length_m(cfg, cfg.MOVING_START_GPS):.0f} m  "
          f"速度 {cfg.TARGET_SPEED_MPS if moving else 0:.0f} m/s")
    print(f"[sim] 純幾何: 推頭距離 240/tan30° = {cfg.predive_ground_range_m:.1f} m, "
          f"DIVE 水平距 120/tan30° = {cfg.dive_ground_range_m:.1f} m, "
          f"DIVE 斜距 120/sin30° = "
          f"{(cfg.CRUISE_AGL_M - cfg.PULLOUT_AGL_M) / math.sin(math.radians(cfg.DIVE_ANGLE_DEG)):.1f} m")

    if args.sweep:
        print(f"\n{'推頭門檻m':>9}{'DIVE平均γ':>11}{'γ範圍':>16}{'末速':>8}{'miss m':>9}")
        for d in range(380, 490, 10):
            r = simulate(cfg, moving, args.guided_dist, d, args.ld,
                         args.q_max, args.n_max)
            gs = dive_gamma_stats(r)
            if gs is None:
                print(f"{d:>9}  (沒進 DIVE)")
                continue
            mt = r['miss']['miss_total'] if r['miss'] else float('nan')
            print(f"{d:>9}{gs['mean']:>11.1f}{gs['min']:>8.1f}~{gs['max']:<7.1f}"
                  f"{r['max_V']:>8.1f}{mt:>9.1f}")
        return

    res = simulate(cfg, moving, args.guided_dist, args.predive_dist, args.ld,
                   args.q_max, args.n_max, verbose=args.verbose)
    print_report(res)
    if args.csv:
        write_csv(res, args.csv)
        print(f"[sim] 軌跡 → {args.csv}")


if __name__ == '__main__':
    main()

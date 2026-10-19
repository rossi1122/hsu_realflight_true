#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xwing_virtual_route.py — xwing_virtual 飛行路線規劃 (路線圖 + 航點檔)
═══════════════════════════════════════════════════════════════════
用 xwing_virtual_sim.py 跑一趟任務, 產生:
  1. xwing_virtual_route.png        路線圖 (俯視 + 高度剖面), 標出各段距離
  2. xwing_virtual_mission.waypoints Mission Planner AUTO 跑道航線 (240 m)
  3. xwing_virtual_route.kml        Google Earth 檢視 (航線 + 模擬軌跡 + 關鍵點)
  4. xwing_virtual_route_traj.csv   模擬軌跡 (含 GPS)

AUTO 跑道 (racetrack) 設計: 進場邊與目標行進方向【同向】(追擊),
  沿 起點→航向點 那條線的延長線進場:
     WP1  進場邊起點   目標起點後方 approach_m      (直線穩定)
     WP2  撥 GUIDED 點 目標起點後方 guided_m        (飛到這裡撥 GUIDED)
     WP3  進場邊終點   目標起點前方 overrun_m       (沒撥 GUIDED 就飛過去)
     WP4/WP5           側移 side_m 的回程邊
     DO_JUMP → WP1 (無限繞)
  全部 240 m (相對 home). ★ 起降航點請自己加在前後 (X-wing 不是 VTOL).

  ★ 2026/10/19 預設值對應使用者的 Mission Planner 任務規劃:
     本檔 WP1 (轉彎後進場起點) ≈ 任務規劃 WP2 轉彎點 (目標起點北方 1013 m)
     本檔 WP2 (撥 GUIDED)      = 任務規劃 WP3 (轉彎點後 400 m, 目標起點北方 613 m)
     本檔 WP3 (進場邊終點)      = 任務規劃 WP4 = 目標起點
     本檔 WP4 / WP5            ≈ 任務規劃 WP5 / WP6、WP1 (西側回程邊)
     實際飛行以 Mission Planner 裡的任務為準.

用法:
  python3 xwing_virtual_route.py                 # 移動目標
  python3 xwing_virtual_route.py --static
  python3 xwing_virtual_route.py --side west     # 回程邊放西側 (預設東側)
  python3 xwing_virtual_route.py --guided-dist 900 --approach 1500
"""

import argparse
import math
import os

from xwing_virtual_config import XwingVirtualConfig
from xwing_virtual_common import road_length_m
from xwing_virtual_sim import (simulate, phase_table, recovery_stats,
                               dive_gamma_stats, ned_to_gps, write_csv,
                               print_report)
from virtual_target_geom import gps_to_ned

HERE = os.path.dirname(os.path.abspath(__file__))

PHASE_COLOR = {
    'SEARCH': '#1f77b4', 'PRE_DIVE': '#ff7f0e', 'DIVE': '#d62728',
    'PULLOUT': '#9467bd', 'DONE': '#2ca02c', 'ABORT': '#e377c2',
}


def axis_point(s, lat_off, heading_deg):
    """沿目標方位 s 公尺 (+ = 目標前進方向), 右側偏 lat_off 公尺 → NED."""
    h = math.radians(heading_deg)
    un, ue = math.cos(h), math.sin(h)          # 前進方向
    rn, re = -ue, un                           # 右手方向
    return s * un + lat_off * rn, s * ue + lat_off * re


def build_racetrack(cfg, heading, approach_m, guided_m, overrun_m, side_m, side):
    sgn = 1.0 if side == 'right' else -1.0
    pts = [
        ('WP1 進場邊起點', axis_point(-approach_m, 0.0, heading)),
        ('WP2 撥 GUIDED 點', axis_point(-guided_m, 0.0, heading)),
        ('WP3 進場邊終點', axis_point(overrun_m, 0.0, heading)),
        ('WP4 回程轉彎', axis_point(overrun_m, sgn * side_m, heading)),
        ('WP5 回程邊終點', axis_point(-approach_m, sgn * side_m, heading)),
    ]
    return pts


def write_waypoints(path, cfg, origin, wps, alt):
    lat0, lon0 = origin
    lines = ["QGC WPL 110",
             # seq cur frame cmd p1 p2 p3 p4 lat lon alt autocontinue
             f"0\t1\t0\t16\t0\t0\t0\t0\t{lat0:.7f}\t{lon0:.7f}\t0.000000\t1"]
    for i, (_, (n, e)) in enumerate(wps, start=1):
        la, lo = ned_to_gps(n, e, lat0, lon0)
        lines.append(f"{i}\t0\t3\t16\t0\t0\t0\t0\t{la:.7f}\t{lo:.7f}\t{alt:.6f}\t1")
    # DO_JUMP 回 WP1, 重複 -1 = 無限
    lines.append(f"{len(wps) + 1}\t0\t0\t177\t1\t-1\t0\t0\t0\t0\t0\t1")
    with open(path, 'w') as f:
        f.write("\n".join(lines) + "\n")


def write_kml(path, res, wps, key_pts, alt):
    lat0, lon0 = res['origin']

    def coord(n, e, h):
        la, lo = ned_to_gps(n, e, lat0, lon0)
        return f"{lo:.7f},{la:.7f},{h:.1f}"

    traj = res['traj']
    parts = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
             '<name>xwing_virtual 路線</name>']
    colors = {'SEARCH': 'ffb4771f', 'PRE_DIVE': 'ff0e7fff', 'DIVE': 'ff2827d6',
              'PULLOUT': 'ffbd6794', 'DONE': 'ff2ca02c', 'AUTO': 'ff999999',
              'TGT': 'ff00ffff'}
    for k, c in colors.items():
        parts.append(f'<Style id="{k}"><LineStyle><color>{c}</color>'
                     f'<width>4</width></LineStyle></Style>')
    # AUTO 跑道
    ring = [wp[1] for wp in wps] + [wps[0][1]]
    parts.append('<Placemark><name>AUTO 跑道 240 m</name><styleUrl>#AUTO</styleUrl>'
                 '<LineString><altitudeMode>relativeToGround</altitudeMode><coordinates>'
                 + " ".join(coord(n, e, alt) for n, e in ring)
                 + '</coordinates></LineString></Placemark>')
    # 模擬軌跡分段
    for ph in ('SEARCH', 'PRE_DIVE', 'DIVE', 'PULLOUT', 'DONE'):
        seg = [r for r in traj if r['mode'] == ph]
        if len(seg) < 2:
            continue
        parts.append(f'<Placemark><name>{ph}</name><styleUrl>#{ph}</styleUrl>'
                     '<LineString><altitudeMode>relativeToGround</altitudeMode>'
                     '<coordinates>'
                     + " ".join(coord(r['n'], r['e'], r['agl']) for r in seg[::5])
                     + '</coordinates></LineString></Placemark>')
    # 目標軌跡
    parts.append('<Placemark><name>虛擬目標軌跡</name><styleUrl>#TGT</styleUrl>'
                 '<LineString><altitudeMode>clampToGround</altitudeMode><coordinates>'
                 + " ".join(coord(r['tgt_n'], r['tgt_e'], 0) for r in traj[::10])
                 + '</coordinates></LineString></Placemark>')
    for name, (n, e, h) in key_pts:
        parts.append(f'<Placemark><name>{name}</name><Point>'
                     f'<altitudeMode>relativeToGround</altitudeMode>'
                     f'<coordinates>{coord(n, e, h)}</coordinates></Point></Placemark>')
    parts.append('</Document></kml>')
    with open(path, 'w', encoding='utf-8') as f:
        f.write("\n".join(parts))


def plot(path, res, wps, key_pts, road_end_ne, alt):
    import logging
    import matplotlib
    matplotlib.use('Agg')
    logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for fp in ('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
               '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
               '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'):
        if os.path.exists(fp):
            font_manager.fontManager.addfont(fp)
            plt.rcParams['font.family'] = font_manager.FontProperties(fname=fp).get_name()
            break
    plt.rcParams['axes.unicode_minus'] = False

    cfg = res['cfg']
    traj = res['traj']
    rows = {r['phase']: r for r in phase_table(res)}
    rc = recovery_stats(res)

    # 各事件當下的虛擬目標位置 (看出追擊: 拉平時車已跑到前方)
    car_pts = []
    for name, mode in (('車@撥GUIDED', 'SEARCH'), ('車@推頭', 'PRE_DIVE'),
                       ('車@拉平', 'PULLOUT')):
        r = next((x for x in traj if x['mode'] == mode), None)
        if r:
            car_pts.append((name, (r['tgt_n'], r['tgt_e'])))

    fig = plt.figure(figsize=(14, 12))
    gsp = fig.add_gridspec(2, 2, height_ratios=[2.0, 1], width_ratios=[1, 1])
    ax = fig.add_subplot(gsp[0, 0])
    axz = fig.add_subplot(gsp[0, 1])
    axp = fig.add_subplot(gsp[1, :])

    def draw_top(a, zoom):
        # AUTO 跑道
        ring = [wp[1] for wp in wps] + [wps[0][1]]
        a.plot([p[1] for p in ring], [p[0] for p in ring], '--', color='#999999',
               lw=1.5, label='AUTO 跑道 (240 m)')
        if not zoom:
            for name, (n, e) in wps:
                a.plot(e, n, 's', color='#666666', ms=6)
                a.annotate(name, (e, n), textcoords='offset points', xytext=(6, 4),
                           fontsize=8, color='#444444')
        # 模擬軌跡
        for ph in ('SEARCH', 'PRE_DIVE', 'DIVE', 'PULLOUT', 'DONE'):
            seg = [r for r in traj if r['mode'] == ph]
            if seg:
                a.plot([r['e'] for r in seg], [r['n'] for r in seg], '-',
                       color=PHASE_COLOR[ph], lw=3, label=ph)
        # 目標
        a.plot([r['tgt_e'] for r in traj], [r['tgt_n'] for r in traj], '-',
               color='#e6b800', lw=2, label='虛擬目標軌跡')
        a.plot(0, 0, '*', color='#e6b800', ms=16, mec='k')
        a.annotate('目標起點', (0, 0), textcoords='offset points', xytext=(8, -12),
                   fontsize=9)
        a.plot(road_end_ne[1], road_end_ne[0], 'D', color='#e6b800', mec='k', ms=7)
        a.annotate('航向點', road_end_ne[::-1], textcoords='offset points',
                   xytext=(8, -4), fontsize=9)
        for name, (n, e, h) in key_pts:
            a.plot(e, n, 'o', color='k', ms=5)
            if zoom or 'GUIDED' in name:
                a.annotate(name.split(' ')[0], (e, n), textcoords='offset points',
                           xytext=(-75, -3), fontsize=8)
        if zoom:
            for name, (tn, te) in car_pts:
                a.plot(te, tn, 'o', mfc='none', mec='#b38f00', mew=2, ms=9)
                a.annotate(name, (te, tn), textcoords='offset points',
                           xytext=(10, -3), fontsize=8, color='#806600')
        a.set_aspect('equal')
        a.grid(alpha=0.3)
        a.set_xlabel('東 (m)  ← 以目標起點為原點')
        a.set_ylabel('北 (m)')

    draw_top(ax, zoom=False)
    ax.set_title('俯視圖: AUTO 跑道 + 任務軌跡', fontsize=12)

    draw_top(axz, zoom=True)
    axz.legend(loc='upper right', fontsize=8)
    ns = [r['n'] for r in traj if r['mode'] != 'SEARCH'] + \
         [r['tgt_n'] for r in traj]
    es = [r['e'] for r in traj if r['mode'] != 'SEARCH'] + [0.0]
    axz.set_ylim(min(ns) - 60, max(ns) + 80)
    axz.set_xlim(min(es) - 260, max(es) + 260)
    axz.set_title('放大: PRE_DIVE → DIVE → 拉平', fontsize=12)

    # 高度剖面 (沿進場軸距離, 0 = 目標起點)
    h = math.radians(res['heading'])
    un, ue = math.cos(h), math.sin(h)

    def along(n, e):
        return n * un + e * ue

    for ph in ('SEARCH', 'PRE_DIVE', 'DIVE', 'PULLOUT', 'DONE'):
        seg = [r for r in traj if r['mode'] == ph]
        if seg:
            axp.plot([along(r['n'], r['e']) for r in seg], [r['agl'] for r in seg],
                     '-', color=PHASE_COLOR[ph], lw=3, label=ph)
    # 30° 參考線 (從 DIVE 起點)
    d0 = rows.get('DIVE')
    if d0:
        s0 = next(r for r in traj if r['mode'] == 'DIVE')
        x0, z0 = along(s0['n'], s0['e']), s0['agl']
        x1 = x0 + (z0 - cfg.PULLOUT_AGL_M) / math.tan(math.radians(cfg.DIVE_ANGLE_DEG))
        axp.plot([x0, x1], [z0, cfg.PULLOUT_AGL_M], ':', color='k', lw=1,
                 label=f'理想 {cfg.DIVE_ANGLE_DEG:.0f}° 線')
    axp.axhline(cfg.CRUISE_AGL_M, color='#bbbbbb', lw=0.8)
    axp.axhline(cfg.PULLOUT_AGL_M, color='#bbbbbb', lw=0.8)
    axp.axhline(cfg.ABORT_MIN_AGL_M, color='#ffaaaa', lw=0.8, ls='--')
    axp.text(along(traj[0]['n'], traj[0]['e']), cfg.ABORT_MIN_AGL_M - 12, f'ABORT {cfg.ABORT_MIN_AGL_M:.0f} m',
             color='#cc4444', fontsize=8)
    for name, (tn, te) in car_pts:
        axp.plot(along(tn, te), 0, 'o', mfc='none', mec='#b38f00', mew=2, ms=9)
        axp.annotate(name, (along(tn, te), 0), textcoords='offset points',
                     xytext=(-20, 10), fontsize=8, color='#806600')
    tg = [along(r['tgt_n'], r['tgt_e']) for r in traj]
    axp.plot(tg, [0] * len(tg), '-', color='#e6b800', lw=4, label='虛擬目標 (地面)')
    # 各段距離標註
    y_lab = cfg.CRUISE_AGL_M + 18
    for ph in ('SEARCH', 'PRE_DIVE', 'DIVE'):
        r = rows.get(ph)
        if not r:
            continue
        seg = [x for x in traj if x['mode'] == ph]
        xa, xb = along(seg[0]['n'], seg[0]['e']), along(seg[-1]['n'], seg[-1]['e'])
        axp.annotate('', xy=(xb, y_lab), xytext=(xa, y_lab),
                     arrowprops=dict(arrowstyle='<->', color=PHASE_COLOR[ph]))
        axp.text((xa + xb) / 2, y_lab + 6,
                 f"{ph}\n{r['horiz_m']:.0f} m / {r['dur']:.1f} s",
                 ha='center', fontsize=8, color=PHASE_COLOR[ph])
        y_lab += 0 if ph == 'SEARCH' else 22
    axp.set_xlabel('沿進場方向距離 (m), 0 = 目標起點, + = 目標前進方向 (往南)')
    axp.set_ylabel('高度 AGL (m)')
    axp.set_ylim(-10, cfg.CRUISE_AGL_M + 80)
    axp.grid(alpha=0.3)
    axp.legend(loc='lower left', fontsize=8, ncol=4)
    axp.set_title('高度剖面', fontsize=12)

    gs = dive_gamma_stats(res)
    txt = (f"巡航 {cfg.CRUISE_AGL_M:.0f} m / {cfg.V_CRUISE_MPS:.0f} m/s, "
           f"俯衝 {cfg.DIVE_ANGLE_DEG:.0f}° ({cfg.CRUISE_AGL_M:.0f}→{cfg.PULLOUT_AGL_M:.0f} m), "
           f"速度上限 {cfg.PULLOUT_VMAX_MPS:.0f} m/s, "
           f"{'移動' if res['moving'] else '靜止'}目標 "
           f"{cfg.TARGET_SPEED_MPS if res['moving'] else 0:.0f} m/s 方位 {res['heading']:.1f}°\n"
           f"撥 GUIDED: 距目標起點 {res['guided_dist']:.0f} m | 推頭門檻 {res['predive_dist']:.0f} m | "
           f"DIVE 平均 {gs['mean']:.1f}° | 最高空速 {res['max_V']:.1f} m/s | "
           f"拉平最低 {res['min_agl_after_pullout']:.0f} m"
           + (f" | 改出水平 {rc['horiz_m']:.0f} m" if rc else ""))
    fig.suptitle('xwing_virtual 飛行路線規劃\n' + txt, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--static', action='store_true', help='靜止目標')
    # 預設值 = 2026/10/19 任務規劃 (Mission Planner):
    #   WP2 轉彎點 → 400 m → WP3 切 GUIDED → 613 m → WP4 目標起點, 回程邊在西側 ~520 m
    ap.add_argument('--guided-dist', type=float, default=613.0,
                    help='撥 GUIDED 點: 目標起點後方 m (預設 613 = WP3→WP4)')
    ap.add_argument('--approach', type=float, default=1013.0,
                    help='進場邊起點 (轉彎點): 目標起點後方 m (預設 1013 = WP2, 轉彎後直線 400 m)')
    ap.add_argument('--overrun', type=float, default=0.0,
                    help='進場邊終點: 目標起點前方 m (預設 0 = 在 WP4 轉彎)')
    ap.add_argument('--side-dist', type=float, default=520.0,
                    help='回程邊側移 m (預設 520, ≥ 2×轉彎半徑)')
    ap.add_argument('--side', choices=('east', 'west'), default='west',
                    help='回程邊在進場邊的哪一側 (預設 west)')
    ap.add_argument('--ld', type=float, default=8.0)
    ap.add_argument('--out-prefix', type=str, default=os.path.join(HERE, 'xwing_virtual'))
    args = ap.parse_args()

    cfg = XwingVirtualConfig()
    moving = not args.static
    res = simulate(cfg, moving, args.guided_dist, None, args.ld)
    print_report(res)
    heading = res['heading']
    origin = res['origin']

    # 右手方向的東分量 = cos(heading). 往南飛 → 右手朝西.
    h = math.radians(heading)
    side = 'right' if ((math.cos(h) > 0) == (args.side == 'east')) else 'left'

    wps = build_racetrack(cfg, heading, args.approach, args.guided_dist,
                          args.overrun, args.side_dist, side)
    traj = res['traj']

    def first(mode):
        return next((r for r in traj if r['mode'] == mode), None)

    key_pts = []
    for name, mode in (('撥GUIDED 任務開始', 'SEARCH'), ('PRE_DIVE 推頭', 'PRE_DIVE'),
                       ('DIVE 開始', 'DIVE'), ('PULLOUT 拉平', 'PULLOUT'),
                       ('DONE TECS', 'DONE')):
        r = first(mode)
        if r:
            key_pts.append((name, (r['n'], r['e'], r['agl'])))
    rn, re_ = gps_to_ned(cfg.MOVING_HEADING_GPS[0], cfg.MOVING_HEADING_GPS[1],
                         origin[0], origin[1])

    pre = args.out_prefix
    write_waypoints(pre + '_mission.waypoints', cfg, origin, wps, cfg.CRUISE_AGL_M)
    write_kml(pre + '_route.kml', res, wps, key_pts, cfg.CRUISE_AGL_M)
    write_csv(res, pre + '_route_traj.csv')
    try:
        plot(pre + '_route.png', res, wps, key_pts, (rn, re_), cfg.CRUISE_AGL_M)
        print(f"[route] 路線圖 → {pre}_route.png")
    except ImportError:
        print("[route] 沒有 matplotlib, 略過畫圖 (pip3 install matplotlib)")

    print(f"[route] Mission Planner 航點 → {pre}_mission.waypoints")
    print(f"[route] Google Earth → {pre}_route.kml")
    print(f"[route] 軌跡 CSV → {pre}_route_traj.csv")
    print("\n AUTO 航點 (相對高度 240 m):")
    for name, (n, e) in wps:
        la, lo = ned_to_gps(n, e, origin[0], origin[1])
        print(f"   {name:<14} {la:.7f}, {lo:.7f}   (北 {n:+7.0f} m, 東 {e:+7.0f} m)")
    print("\n 任務關鍵點:")
    for name, (n, e, hh) in key_pts:
        la, lo = ned_to_gps(n, e, origin[0], origin[1])
        print(f"   {name:<16} {la:.7f}, {lo:.7f}   AGL {hh:5.0f} m")
    print(f"\n 目標: 起點 {origin}, 起點→航向點 {road_length_m(cfg, origin):.0f} m")


if __name__ == '__main__':
    main()

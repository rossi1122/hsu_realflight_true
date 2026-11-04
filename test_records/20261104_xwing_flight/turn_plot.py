import pickle, sys, math, bisect, statistics as st, gzip, json
def LOAD(f):
    return json.load(gzip.open(f,'rt')) if f.endswith('.gz') else pickle.load(open(f,'rb'))
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
fm.fontManager.addfont('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc')
plt.rcParams['font.family'] = 'WenQuanYi Zen Hei'; plt.rcParams['axes.unicode_minus'] = False
R = 6378137.0
def ne(p, o): return (math.radians(p[0]-o[0])*R, math.radians(p[1]-o[1])*R*math.cos(math.radians(o[0])))
def idx(ts, t): return min(bisect.bisect_left(ts, t), len(ts)-1)
COL6 = ['#2a78d6','#eb6834','#1baf7a','#eda100','#e87ba4','#008300','#8e44ad','#7f6a00']
SURF, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
import re
PASSES = []
for f in sys.argv[2:]:
    o = LOAD(f)
    for d in o['MSG']:
        m = re.search(r'(Reached|Passed) waypoint #(\d+)', d['Message'])
        if m and int(m.group(2)) == 2: PASSES.append((f, d['TimeUS']/1e6, int(m.group(2))))
cache = {}; res = []
for f, t0, wk in PASSES:
    o = cache.setdefault(f, LOAD(f))
    cmd = {d['CNum']: (d['Lat'], d['Lng'], d['Alt']) for d in o['CMD'] if d['TimeUS']/1e6 <= t0}   # 當時有效的航點 (中途重傳過任務)
    TW, NW = cmd[wk], cmd[wk+1]
    nn, ee = ne(NW, TW); LEG = math.degrees(math.atan2(ee, nn)) % 360; ux, uy = nn/math.hypot(nn, ee), ee/math.hypot(nn, ee)
    P, A, G, N = o['POS'], o['ATT'], o['GPS'], o['NTUN']
    C = o['CTUN']; ct=[r['TimeUS']/1e6 for r in C]
    pt=[r['TimeUS']/1e6 for r in P]; at=[r['TimeUS']/1e6 for r in A]; gt=[r['TimeUS']/1e6 for r in G]; nt=[r['TimeUS']/1e6 for r in N]
    modes = [(d['TimeUS']/1e6, d['Mode']) for d in o['MODE'] if d['TimeUS']/1e6 > t0 and d['Mode'] != 10]
    t_end = min(modes[0][0] if modes else 1e9, t0 + 30.0)
    # AUTO 下一個轉彎 (離開這條往南航線) 也算結束
    for d in o['MSG']:
        tm = d['TimeUS']/1e6; m = re.search(r'(Reached|Passed) waypoint #(\d+)', d['Message'])
        if m and t0 + 0.5 < tm < t_end:
            j = int(m.group(2))
            if j + 1 in cmd and cmd[j+1][0] != 0:
                a2, b2 = ne(cmd[j+1], cmd[j]); brg2 = math.degrees(math.atan2(b2, a2)) % 360
                if abs((brg2 - LEG + 180) % 360 - 180) > 20: t_end = tm
    i0 = idx(pt, t0); path = 0; rows = []
    for i in range(i0, len(pt)):
        t = pt[i]
        if t > t_end: break
        if i > i0:
            a = ne((P[i-1]['Lat'], P[i-1]['Lng']), TW); b = ne((P[i]['Lat'], P[i]['Lng']), TW); path += math.hypot(b[0]-a[0], b[1]-a[1])
        n, e = ne((P[i]['Lat'], P[i]['Lng']), TW)
        along = n*ux + e*uy; cross = -n*uy + e*ux     # cross > 0 = 航線右側 (往南飛時 = 西)
        rows.append(dict(t=t-t0, path=path, south=along, east=-cross, roll=A[idx(at,t)]['Roll'],
                         terr=(G[idx(gt,t)]['GCrs']-LEG+180)%360-180, xt=-cross, alt=P[i]['RelHomeAlt'],
                         asp=C[idx(ct,t)]['As'], gs=G[idx(gt,t)]['Spd']))
    def first(cond, hold):
        for k, r in enumerate(rows):
            if r['t'] < 3: continue
            win = [q for q in rows[k:] if q['t'] <= r['t']+hold]
            if win and win[-1]['t'] >= r['t']+hold-0.05 and all(cond(q) for q in win): return r
    first_ok = lambda rw: first(lambda q: abs(q['roll']) <= 10 and abs(q['terr']) <= 5 and abs(q['xt']) <= 10, 3.0)
    lvl = first(lambda q: abs(q['roll']) <= 10, 2.0)
    ali = first(lambda q: abs(q['roll']) <= 10 and abs(q['terr']) <= 5 and abs(q['xt']) <= 10, 3.0)
    fs = [d['TimeUS']/1e6 for d in o['MSG'] if 'RC Short Failsafe On' in d['Message'] or 'RC Short Failsafe: switched' in d['Message']]
    fs_note = ''.join(f' / {x-t0:.1f}s RC 失控' for x in fs if t0 <= x <= t0 + 30)
    a_off = [d['TimeUS']/1e6 for d in o['MSG'] if 'Airspeed sensor 1 failure' in d['Message']]
    a_on = [d['TimeUS']/1e6 for d in o['MSG'] if 'Airspeed sensor 1 now OK' in d['Message']]
    if a_off and a_off[0] <= t0 and not (a_on and a_on[0] <= t0): fs_note += ' / 空速計停用'
    if modes and abs(modes[0][0]-t_end) < 0.05: mode_note = f"{t_end - t0:.1f}s 切 {({5: 'FBWA', 15: 'GUIDED', 0: 'MANUAL'}).get(modes[0][1], modes[0][1])}"
    else: mode_note = f"{t_end - t0:.1f} s 後下一個轉彎" if not first_ok(rows) else ''
    mode_note = (mode_note + fs_note).lstrip(' /')
    _k = len([x for x in res if x['file']==f and x['wk']==wk])+1
    res.append(dict(wk=wk, alt=TW[2], short=f"第 {_k} 次 (t={t0:.0f}s)", name=f"第 {_k} 次", file=f, t0=t0, rows=rows, lvl=lvl, ali=ali, note=mode_note,
                    start_dist=math.hypot(rows[0]['south'], rows[0]['east'])))
for r in res:
    l, a = r['lvl'], r['ali']
    print(r['name'], round(r['t0'],1), '轉彎起點離WP2 %.0f m' % r['start_dist'],
          '拉平', (f"{l['t']:.1f}s 路徑{l['path']:.0f} 南{l['south']:.0f}" if l else '-'),
          '對正+拉平', (f"{a['t']:.1f}s 路徑{a['path']:.0f} 南{a['south']:.0f}" if a else '-'), r['note'])
done = [r for r in res if r['ali']]
print('平均 對正+拉平: %.1f s, 路徑 %.0f m, WP2 南方 %.0f m (n=%d)' % (st.mean(r['ali']['t'] for r in done), st.mean(r['ali']['path'] for r in done), st.mean(r['ali']['south'] for r in done), len(done)))
lv = [r for r in res if r['lvl']]
print('平均 拉平: %.1f s, 路徑 %.0f m, WP2 南方 %.0f m (n=%d)' % (st.mean(r['lvl']['t'] for r in lv), st.mean(r['lvl']['path'] for r in lv), st.mean(r['lvl']['south'] for r in lv), len(lv)))

fig = plt.figure(figsize=(14, 12.5), facecolor=SURF)
gs = fig.add_gridspec(3, 2, width_ratios=[0.9, 1.25], hspace=0.45, wspace=0.2)
axm = fig.add_subplot(gs[0:3, 0]); axr = fig.add_subplot(gs[0, 1]); axx = fig.add_subplot(gs[1, 1], sharex=axr); axs = fig.add_subplot(gs[2, 1], sharex=axr)
fig2 = plt.figure(figsize=(14, 11), facecolor=SURF)
gs2 = fig2.add_gridspec(2, 2, height_ratios=[1, 1.05], hspace=0.35, wspace=0.18)
axb1 = fig2.add_subplot(gs2[0, 0]); axb2 = fig2.add_subplot(gs2[0, 1]); axt = fig2.add_subplot(gs2[1, :])
for ax in (axm, axr, axx, axs):
    ax.set_facecolor(SURF)
    for sp in ('top', 'right'): ax.spines[sp].set_visible(False)
    for sp in ('left', 'bottom'): ax.spines[sp].set_color(GRID)
    ax.grid(True, color=GRID, lw=0.8); ax.set_axisbelow(True); ax.tick_params(colors=INK2, labelsize=10)
for i, r in enumerate(res):
    c = COL6[i]; T = [q['t'] for q in r['rows']]
    axm.plot([q['east'] for q in r['rows']], [q['south'] for q in r['rows']], color=c, lw=2, label=r['short'])
    q0 = r['rows'][0]
    axm.plot([q0['east']], [q0['south']], marker='*', ms=15, color=c, mec=INK, mew=0.8, zorder=7)
    axr.plot(T, [q['roll'] for q in r['rows']], color=c, lw=2)
    axx.plot(T, [q['xt'] for q in r['rows']], color=c, lw=2)
    axs.plot(T, [q['asp'] for q in r['rows']], color=c, lw=2)
    for key, mk in (('lvl', 's'), ('ali', 'o')):
        q = r[key]
        if q:
            axm.plot([q['east']], [q['south']], mk, ms=8, color=c, mec=SURF, mew=2, zorder=5)
            axr.plot([q['t']], [q['roll']], mk, ms=8, color=c, mec=SURF, mew=2, zorder=5)
            axx.plot([q['t']], [q['xt']], mk, ms=8, color=c, mec=SURF, mew=2, zorder=5)
            axs.plot([q['t']], [q['asp']], mk, ms=8, color=c, mec=SURF, mew=2, zorder=5)
axm.plot([0], [0], marker='^', ms=11, color=INK, zorder=6); axm.text(8, -12, '轉彎點', color=INK, fontsize=11)
axm.axvline(0, color=INK2, lw=1, ls='--'); axm.text(3, 545, '往南航線', color=INK2, fontsize=10)
axm.invert_yaxis(); axm.set_xlim(-150, 95); axm.set_ylim(560, -40)
axm.set_title('飛行軌跡 (俯視)', color=INK, fontsize=13, loc='left')
axm.set_xlabel('離航線 東(+)/西(-) (m)', color=INK2); axm.set_ylabel('沿航線離轉彎點 (m)', color=INK2)
from matplotlib.lines import Line2D
hs, ls = axm.get_legend_handles_labels()
hs += [Line2D([], [], ls='', marker='*', ms=13, color=INK2), Line2D([], [], ls='', marker='s', ms=8, color=INK2), Line2D([], [], ls='', marker='o', ms=8, color=INK2)]
ls += ['開始計時 (時間 0)', '拉平', '對正 + 拉平']
_x0 = st.mean(r['rows'][0]['east'] for r in res); _y0 = st.mean(r['rows'][0]['south'] for r in res)
axm.annotate('開始計時 (時間 0)\n飛控回報 Reached waypoint #2\n= 開始轉彎, 在轉彎點西方約 130 m',
             xy=(_x0, _y0), xytext=(-140, 75), color=INK, fontsize=10.5,
             arrowprops=dict(arrowstyle='->', color=INK, lw=1.2))
leg = axm.legend(hs, ls, loc='lower left', frameon=False, fontsize=10, title='log 檔 / 第幾次轉彎', title_fontsize=10)
for t in leg.get_texts(): t.set_color(INK)
axr.axhspan(-10, 10, color='#eeede9', zorder=0); axr.set_ylabel('滾轉角 roll (°)', color=INK2)
axr.set_title('滾轉角 (灰帶 = ±10° 視為拉平)', color=INK, fontsize=13, loc='left')
axx.axhspan(-10, 10, color='#eeede9', zorder=0); axx.set_ylabel('側偏 (m)', color=INK2)
axs.axhline(30, color=INK2, lw=1, ls='--'); axs.text(30, 29.4, '巡航 30 m/s', color=INK2, fontsize=10, ha='right', va='top')
axs.set_ylabel('空速 (m/s)', color=INK2); axs.set_xlabel('通過轉彎點後時間 (s)', color=INK2)
axs.set_title('空速 (飛控 CTUN.As)', color=INK, fontsize=13, loc='left')
axx.set_title('離航線側偏 (灰帶 = ±10 m 視為對正)', color=INK, fontsize=13, loc='left'); axx.set_ylim(-40, 40)
for ax, key, unit, ref, ttl in ((axb1, 't', 's', (10.1, 12.9), '所需時間 (s)'), (axb2, 'south', 'm', (269, 366), '離轉彎點距離 (沿航線, m)')):
    ax.set_facecolor(SURF)
    for sp in ('top', 'right'): ax.spines[sp].set_visible(False)
    for sp in ('left', 'bottom'): ax.spines[sp].set_color(GRID)
    ax.grid(True, axis='y', color=GRID, lw=0.8); ax.set_axisbelow(True); ax.tick_params(colors=INK2, labelsize=10)
    xs = list(range(len(res))); w = 0.38
    lv_v = [r['lvl'][key] if r['lvl'] else 0 for r in res]; al_v = [r['ali'][key] if r['ali'] else 0 for r in res]
    b1 = ax.bar([x - w/2 for x in xs], lv_v, w, color=[COL6[i] for i in xs], alpha=0.45, label='機翼拉平')
    b2 = ax.bar([x + w/2 for x in xs], al_v, w, color=[COL6[i] for i in xs], label='對正 + 拉平')
    for x, v, r in zip(xs, al_v, res):
        if not r['ali']: ax.text(x + w/2, 0.5 if unit == 's' else 10, '未完成', ha='center', va='bottom', fontsize=9, color=INK2, rotation=90)
    for x, v in zip(xs, al_v):
        if v: ax.text(x + w/2, v, f'{v:.1f}' if unit == 's' else f'{v:.0f}', ha='center', va='bottom', fontsize=9, color=INK)
    m_l = st.mean(v for v in lv_v if v); m_a = st.mean(v for v in al_v if v)
    fmt = (lambda v: f'{v:.1f}') if unit == 's' else (lambda v: f'{v:.0f}')
    ax.axhline(m_a, color=INK, lw=1.4, ls='-', label=f'11/04 第一架 對正平均 {fmt(m_a)}')
    ax.axhline(ref[1], color='#d03b3b', lw=1.4, ls='--', label=f'10/29 A 機 對正平均 {ref[1]:g}')
    ax.axhline(ref[0], color='#d03b3b', lw=1, ls=':', label=f'10/29 A 機 拉平平均 {ref[0]:g}')
    ax.set_xticks(xs); ax.set_xticklabels([r['name'] for r in res], color=INK, fontsize=9.5)
    ax.set_title(ttl, color=INK, fontsize=13, loc='left'); ax.set_ylabel(unit, color=INK2)
    ax.legend(frameon=False, fontsize=9, loc='upper left', ncol=3)
axb1.set_ylim(0, 25); axb2.set_ylim(0, 680)
axt.axis('off')
hdr = ['log 檔 / 第幾次', '實際高度\n(m)', '轉彎中空速\n(m/s)', '機翼拉平\n(s)', '拉平時\n離轉彎點 (m)', '對正+拉平\n(s)', '對正時\n離轉彎點 (m)', '對正時\n空速 / 地速 (m/s)', '備註']
rows = []
for r in res:
    l, a = r['lvl'], r['ali']
    _al = [q['alt'] for q in r['rows']]
    _end = a['t'] if a else r['rows'][-1]['t']; _as = [q['asp'] for q in r['rows'] if q['t'] <= _end]
    rows.append([r['name'], f"{min(_al):.0f} ~ {max(_al):.0f}", f"{min(_as):.1f} ~ {max(_as):.1f}", f"{l['t']:.1f}" if l else '—', f"{l['south']:.0f}" if l else '—',
                 f"{a['t']:.1f}" if a else '—', f"{a['south']:.0f}" if a else '—',
                 f"{a['asp']:.1f} / {a['gs']:.1f}" if a else '—',
                 r['note'] or ('完成' if a else '')])
d240 = [r for r in done if r['wk'] == 2]; l240 = [r for r in lv if r['wk'] == 2]
rows.append(['平均', '', '', f"{st.mean(r['lvl']['t'] for r in l240):.1f}", f"{st.mean(r['lvl']['south'] for r in l240):.0f}",
             f"{st.mean(r['ali']['t'] for r in d240):.1f}", f"{st.mean(r['ali']['south'] for r in d240):.0f}",
             f"{st.mean(r['ali']['asp'] for r in d240):.1f} / {st.mean(r['ali']['gs'] for r in d240):.1f}", f"n = {len(d240)}"])
rows.append(['範圍', '', '', f"{min(r['lvl']['t'] for r in l240):.1f} ~ {max(r['lvl']['t'] for r in l240):.1f}", f"{min(r['lvl']['south'] for r in l240):.0f} ~ {max(r['lvl']['south'] for r in l240):.0f}",
             f"{min(r['ali']['t'] for r in d240):.1f} ~ {max(r['ali']['t'] for r in d240):.1f}", f"{min(r['ali']['south'] for r in d240):.0f} ~ {max(r['ali']['south'] for r in d240):.0f}", '', ''])
rows.append(['10/29 A 機 平均 (對照)', '', '', '10.1', '269', '12.9', '366', '32.9 / 36.5', 'n = 6'])
rows.append(['10/8 平均 (對照)', '', '27 ~ 35', '8.0', '205', '12.1', '350', '31.4 / 34.1', 'n = 3'])
tb = axt.table(cellText=rows, colLabels=hdr, loc='upper center', cellLoc='center', colWidths=[0.15, 0.08, 0.10, 0.07, 0.10, 0.08, 0.10, 0.12, 0.20])
tb.auto_set_font_size(False); tb.set_fontsize(9.5); tb.scale(1, 1.6)
for (ri, ci), cell in tb.get_celld().items():
    cell.set_edgecolor(GRID); cell.set_facecolor(SURF)
    if ri == 0: cell.set_text_props(color=INK2, weight='bold'); cell.set_height(cell.get_height() * 1.9)
    elif ri >= len(rows) - 3: cell.set_text_props(color=INK, weight='bold'); cell.set_facecolor('#f1f0ec')
    else:
        cell.set_text_props(color=INK)
        if ci == 0 and ri - 1 < len(res): cell.get_text().set_text('■ ' + rows[ri-1][0]); cell.get_text().set_color(COL6[ri-1])
axt.set_title('各次結果  (拉平: |roll| ≤ 10° 持續 2 s;  對正+拉平: |roll| ≤ 10°、航跡 ±5°、側偏 ≤ 10 m 同時持續 3 s)', color=INK, fontsize=12, loc='left')
fig.suptitle(f'X-wing 第一架 11/04 實飛 (AUTO, 240 m): 通過轉彎點 WP2 → 對正 + 拉平 — 軌跡  (全部 {len(res)} 次)', color=INK, fontsize=15, x=0.06, ha='left', y=0.995)
fig2.suptitle(f'X-wing 第一架 11/04 實飛 (AUTO, 240 m): 通過轉彎點 WP2 → 對正 + 拉平 — 統計  (全部 {len(res)} 次, 對照 10/29、10/8)', color=INK, fontsize=15, x=0.06, ha='left', y=0.985)
fig2.text(0.06, 0.953, '時間 0 = 飛控回報 Reached waypoint #2 (開始轉彎);  淡色 = 機翼拉平, 深色 = 對正 + 拉平;  紅線 = 10/29 A 機平均 (虛線 對正+拉平, 點線 機翼拉平);  黑線 = 11/04 對正平均', color=INK2, fontsize=10.5)
fig.text(0.06, 0.968, '時間 0 = 飛控回報 Reached waypoint #2 (開始轉彎, 此時在轉彎點西方約 130 m);  距離 = 沿航線離轉彎點的距離;  當天北風約 3~7 m/s', color=INK2, fontsize=10.5)
fig.savefig(sys.argv[1].replace('.png', '_trajectory.png'), dpi=125, facecolor=SURF, bbox_inches='tight')
fig2.savefig(sys.argv[1].replace('.png', '_stats.png'), dpi=125, facecolor=SURF, bbox_inches='tight')

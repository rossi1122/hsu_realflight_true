import pickle, math, csv, datetime, sys
import numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
fm.fontManager.addfont('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc')
plt.rcParams['font.family'] = 'WenQuanYi Zen Hei'; plt.rcParams['axes.unicode_minus'] = False
SC = '/tmp/claude-0/-home-user-hsu-realflight-true/b79ff2fa-47de-5b23-89fd-e6ce021b1454/scratchpad/'
OUT = sys.argv[1]
SURF, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
COL = ['#2a78d6', '#eb6834', '#1baf7a', '#8e44ad']
TW = datetime.timezone(datetime.timedelta(hours=8))
S = (23.7222419, 120.3798848); R = 6378137
o = pickle.load(open(SC + 'car1104/car.pkl', 'rb'))
G = [g for g in o['GPS'] if g['Status'] >= 3]
cu = np.array([315964800 + g['GWk'] * 604800 + g['GMS'] / 1000 - 18 for g in G])
cn = np.array([math.radians(g['Lat'] - S[0]) * R for g in G])
cs = np.array([g['Spd'] for g in G])
def hm(u): return datetime.datetime.fromtimestamp(u, TW).strftime('%H:%M:%S')

# ── 車子 4 趟: 出發時刻 (速度 > 0.5 連續) ──
deps = []
for lab, hhmmss in (('第 1 趟 (配合第一架 run 2)', '11:25:50'), ('第 2 趟', '11:31:40'), ('第 3 趟', '11:34:55'), ('第 4 趟', '11:37:30')):
    t0 = datetime.datetime.strptime('2026-11-04 ' + hhmmss, '%Y-%m-%d %H:%M:%S').replace(tzinfo=TW).timestamp()
    idx = np.where(cu >= t0)[0]
    for i in idx:
        if np.all(cs[i:i + 5] > 0.5):
            deps.append((lab, i)); break
runs = []
for lab, i in deps:
    k = (cu >= cu[i]) & (cu <= cu[i] + 40)
    t = cu[k] - cu[i]; d = cn[i] - cn[k]; v = cs[k]; n0 = cn[i]
    j = np.where(cn[k] <= 0)[0]
    tx = t[j[0]] if len(j) else None
    vx = v[j[0]] if len(j) else None
    runs.append(dict(lab=lab, t=t, d=d, v=v, n0=n0, tx=tx, vx=vx, vmax=v[t <= 35].max(), dep=cu[i]))

def style(ax):
    ax.set_facecolor(SURF)
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'): ax.spines[s].set_color(GRID)
    ax.grid(True, color=GRID, lw=0.8); ax.set_axisbelow(True); ax.tick_params(colors=INK2, labelsize=10)

# ═════ 圖 1: 車子加速統計 ═════
fig = plt.figure(figsize=(14, 11), facecolor=SURF)
gs = fig.add_gridspec(2, 2, height_ratios=[1, 0.62], hspace=0.38, wspace=0.18)
a1 = fig.add_subplot(gs[0, 0]); a2 = fig.add_subplot(gs[0, 1]); at = fig.add_subplot(gs[1, :])
for ax in (a1, a2): style(ax)
tp = np.linspace(0, 35, 200)
vp = np.minimum(10.0, tp / 7 * 10); dp = np.where(tp < 7, 10 / 14 * tp ** 2, 35 + 10 * (tp - 7))
for c, r in zip(COL, runs):
    a1.plot(r['t'], r['v'], color=c, lw=2, label=r['lab'])
    a2.plot(r['t'], r['d'], color=c, lw=2, label=r['lab'])
    a2.axhline(r['n0'], color=c, lw=0.8, ls=':')
    if r['tx'] is not None:
        a1.plot([r['tx']], [r['vx']], 'o', color=c, ms=8, mec=SURF, mew=2, zorder=5)
        a2.plot([r['tx']], [r['n0']], 'o', color=c, ms=8, mec=SURF, mew=2, zorder=5)
a1.plot(tp, vp, '--', color=INK2, lw=1.5, label='原本假設 (7 s 加到 10 m/s)')
a1.axhline(10, color='#d03b3b', lw=1, ls=':'); a1.text(34.5, 10.2, '虛擬目標 10 m/s', ha='right', color='#d03b3b', fontsize=10)
a1.set_xlim(0, 35); a1.set_ylim(0, 11); a1.set_xlabel('出發後時間 (s)', color=INK2); a1.set_ylabel('車速 (m/s)', color=INK2)
a1.set_title('(a) 車速 (圓點 = 經過虛擬起點)', loc='left', color=INK, fontsize=13)
a1.legend(frameon=False, fontsize=9.5, loc='upper left')
a2.plot(tp, dp, '--', color=INK2, lw=1.5, label='原本假設')
a2.axhline(65, color=INK, lw=1.2, ls='--'); a2.text(0.5, 67, '明天停車位置: 起點北方 65 m', color=INK, fontsize=10)
a2.axvspan(15, 16.5, color='#f2e9c9', zorder=0); a2.text(15.75, 150, '飛機 WP2 西 150 m\n→ WP3 約 15~16.5 s', ha='center', va='top', fontsize=9.5, color=INK2)
a2.set_xlim(0, 35); a2.set_ylim(0, 160); a2.set_xlabel('出發後時間 (s)', color=INK2); a2.set_ylabel('已開距離 (m)', color=INK2)
a2.set_title('(b) 已開距離 (點線 = 該趟出發時離起點的距離)', loc='left', color=INK, fontsize=13)
at.axis('off')
hdr = ['車子趟次', '出發時刻', '出發時離起點\n(北方, m)', '出發 → 經過起點\n(s)', '經過起點時速\n(m/s)', '最高速\n(m/s)', '到 90% 最高速\n(s)', '出發後 15 / 18 / 22 s\n已開 (m)']
rows = []
for r in runs:
    def dat(T): return f"{r['d'][np.argmin(abs(r['t'] - T))]:.0f}"
    t90 = r['t'][np.argmax(r['v'] >= 0.9 * r['vmax'])]
    rows.append([r['lab'], hm(r['dep']), f"{r['n0']:.1f}", f"{r['tx']:.1f}" if r['tx'] else '—', f"{r['vx']:.1f}" if r['vx'] else '—',
                 f"{r['vmax']:.1f}", f"{t90:.1f}", f"{dat(15)} / {dat(18)} / {dat(22)}"])
tb = at.table(cellText=rows, colLabels=hdr, loc='upper center', cellLoc='center', colWidths=[0.2, 0.09, 0.11, 0.12, 0.11, 0.08, 0.11, 0.16])
tb.auto_set_font_size(False); tb.set_fontsize(10); tb.scale(1, 1.9)
for (ri, ci), cell in tb.get_celld().items():
    cell.set_edgecolor(GRID); cell.set_facecolor(SURF)
    if ri == 0: cell.set_text_props(color=INK2, weight='bold'); cell.set_height(cell.get_height() * 1.5)
    elif ci == 0: cell.get_text().set_color(COL[ri - 1])
at.set_title('各趟統計 (車子 BIN, 5 Hz GPS)', loc='left', color=INK, fontsize=12)
fig.suptitle('11/04 真車加速統計 (車子 GPS 記錄 11:24~11:43)', x=0.06, ha='left', fontsize=15, color=INK, y=0.985)
fig.text(0.06, 0.955, '起點 = 虛擬軌跡起點 (23.7222419, 120.3798848); 4 趟都在起點正北方出發往南開 (東西偏差 < 3 m)', fontsize=10.5, color=INK2)
fig.savefig(OUT + '/car_accel_stats_1104.png', dpi=120, facecolor=SURF, bbox_inches='tight')

# ═════ 圖 2: 真實俯衝時 飛機 / 虛擬目標 / 真車 ═════
L = list(csv.DictReader(open(SC + 'f1104a/x/1_test/xwing_virtual_19700103_041747_moving_A_yawpid.csv', encoding='utf-8-sig')))
def fl(x):
    try: return float(x)
    except: return float('nan')
lt = np.array([fl(r['t']) for r in L]); lu = np.array([fl(r['gps_unix']) for r in L])
ok = np.isfinite(lu)
# log t → GPS 時間 (線性回歸, 避開 gps_unix 的取樣跳動)
A = np.polyfit(lt[ok], lu[ok], 1)
fig2, axs = plt.subplots(2, 2, figsize=(15, 10.5), facecolor=SURF, gridspec_kw=dict(hspace=0.45, wspace=0.2, width_ratios=[1.35, 1]))
for row, (name, tg) in enumerate((('第一架 run 2', 1162.93), ('第一架 run 3', 1358.55))):
    ax, bx = axs[row]
    style(ax); style(bx)
    k = (lt >= tg - 20) & (lt <= tg + 22)
    tt = lt[k] - tg; ug = np.polyval(A, tg)
    g = [L[i] for i in np.where(k)[0]]
    an = np.array([fl(r['ac_n']) for r in g]); tn = np.array([fl(r['tgt_n']) for r in g])
    mode = [r['mode'] if r['guided'] == '1' else '' for r in g]
    tn = np.where(tt >= 0, tn, np.nan)
    kc = (cu >= ug - 20) & (cu <= ug + 22)
    ct = cu[kc] - ug; cnn = cn[kc]
    for m, c in (('PRE_DIVE', '#f6e3c4'), ('DIVE', '#f6cfc8'), ('PULLOUT', '#e3d7ee')):
        idx = [i for i, x in enumerate(mode) if x == m]
        if idx:
            for a_ in (ax, bx): a_.axvspan(tt[idx[0]], tt[idx[-1]], color=c, zorder=0)
            ax.text((tt[idx[0]] + tt[idx[-1]]) / 2, {'PRE_DIVE': 715, 'DIVE': 660, 'PULLOUT': 605}[m], m, ha='center', fontsize=9, color=INK2)
    ax.plot(tt, an, color=INK, lw=2, label='飛機 (北向位置)')
    ax.plot(tt, tn, color='#d03b3b', lw=2, ls='--', label='虛擬目標 (切 GUIDED 起從起點以 10 m/s 往南)')
    ax.plot(ct, cnn, color=COL[1], lw=2.4, label='真車 GPS')
    ax.axhline(0, color=INK2, lw=0.8); ax.text(-19.5, 8, '虛擬起點', color=INK2, fontsize=9)
    ax.axvline(0, color=INK2, lw=1, ls=':'); ax.text(0.3, 680, '切 GUIDED', color=INK2, fontsize=9)
    ax.set_xlim(-20, 22); ax.set_ylim(-260, 760)
    ax.set_xlabel(f'切 GUIDED 後時間 (s)   [t = 0 = {hm(ug)}]', color=INK2); ax.set_ylabel('在起點北方 (m)', color=INK2)
    ax.set_title(f'({"ab"[row]}) {name}: 位置對時間', loc='left', color=INK, fontsize=13)
    ax.legend(frameon=False, fontsize=9.5, loc='lower left')
    # 差距
    cn_i = np.interp(tt, ct, cnn, left=np.nan, right=np.nan)
    gap = cn_i - tn
    bx.plot(tt, gap, color=COL[1], lw=2.2)
    bx.axhline(0, color=INK2, lw=0.8)
    for m in ('PRE_DIVE', 'DIVE', 'PULLOUT'):
        idx = [i for i, x in enumerate(mode) if x == m]
        if idx:
            i0 = idx[0]; bx.plot([tt[i0]], [gap[i0]], 'o', color=INK, ms=6)
            bx.annotate(f'{m} {gap[i0]:.0f} m', (tt[i0], gap[i0]), xytext=(6, -14), textcoords='offset points', fontsize=9.5, color=INK)
    bx.set_xlim(-1, 22); bx.set_ylim(-20, 260)
    bx.set_xlabel('切 GUIDED 後時間 (s)', color=INK2); bx.set_ylabel('真車 - 虛擬目標 (m, + = 車子落後)', color=INK2)
    bx.set_title(f'({"cd"[row]}) {name}: 真車落後虛擬目標多少', loc='left', color=INK, fontsize=13)
fig2.suptitle('11/04 第一架兩趟真實俯衝: 飛機 / 虛擬目標 / 真車 (GPS 時間對齊)', x=0.06, ha='left', fontsize=15, color=INK, y=0.99)
fig2.text(0.06, 0.955, 'run 2: 車子在切 GUIDED 前 6.6 s 才慢慢出發 (最高約 4.5 m/s);  run 3: 車子停在起點北方約 41 m 沒有開', fontsize=10.5, color=INK2)
fig2.savefig(OUT + '/dive_car_vs_virtual_1104.png', dpi=120, facecolor=SURF, bbox_inches='tight')
for r in runs: print(r['lab'], round(r['n0'], 1), r['tx'], r['vx'], r['vmax'])

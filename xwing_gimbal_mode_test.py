#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xwing_gimbal_mode_test.py — 地面測試: 對方雲台模式鍵 (不需要 ROS / 飛控)
═══════════════════════════════════════════════════════════════════
用【實飛同一套】程式 (xwing_uart_vision.UartVisionLink) 和【同一張】對應表
(xwing_virtual_config.GIMBAL_MODE_KEYS), 模擬任務狀態切換, 確認:

  · 進 SEARCH  → 送 '1' → 對方回報 mode=1 (追蹤)
  · 進 PRE_DIVE → 送 '2' → 對方回報 mode=2 (鎖定), 雲台 pitch 轉到 −30° 並停住
  · 每次: 送出 → 對方確認 → 機械到位 各花多久

流程與判讀見 xwing_virtual_GIMBAL_GROUND_TEST.md.

用法:
  python3 xwing_gimbal_mode_test.py                     # 互動: 按鍵切任務狀態
  python3 xwing_gimbal_mode_test.py --auto              # 自動跑 SEARCH→PRE_DIVE→DIVE→PULLOUT
  python3 xwing_gimbal_mode_test.py --auto --repeat 5   # 連跑 5 輪, 統計鎖定時間
  python3 xwing_gimbal_mode_test.py --port /dev/ttyUSB0

互動按鍵:
  b = STANDBY   s = SEARCH   p = PRE_DIVE   d = DIVE   o = PULLOUT
  0 / 1 / 2 = 直接送模式鍵 (不經對應表)      q = 結束
"""

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from xwing_virtual_config import XwingVirtualConfig
from xwing_uart_vision import UartVisionLink, MODE_NAMES

KEY_TO_STATE = {'b': 'STANDBY', 's': 'SEARCH', 'p': 'PRE_DIVE',
                'd': 'DIVE', 'o': 'PULLOUT'}
AUTO_SEQ = [('STANDBY', 3.0), ('SEARCH', 6.0), ('PRE_DIVE', 2.0),
            ('DIVE', 6.0), ('PULLOUT', 3.0)]


def _f(x, nd=1):
    return f"{x:+.{nd}f}" if isinstance(x, (int, float)) and math.isfinite(x) else "  N/A"


class Keys:
    """Linux 終端單鍵讀取 (不需 Enter). 非終端 (例如管線) 時停用."""

    def __init__(self):
        self.ok = sys.stdin.isatty() and os.name != 'nt'
        self.old = None
        if self.ok:
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def get(self):
        if not self.ok:
            return ''
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return os.read(self.fd, 1).decode('ascii', errors='ignore')
        return ''

    def close(self):
        if self.old is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def main():
    cfg = XwingVirtualConfig()
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=cfg.VISION_UART_PORT)
    ap.add_argument('--baud', type=int, default=cfg.VISION_UART_BAUD)
    ap.add_argument('--auto', action='store_true', help='自動跑一輪任務狀態')
    ap.add_argument('--repeat', type=int, default=1, help='--auto 跑幾輪 (預設 1)')
    ap.add_argument('--max-confirm', type=float, default=1.0,
                    help='模式確認時間上限 s (預設 1.0)')
    ap.add_argument('--max-settle', type=float, default=1.5,
                    help='鎖定到位時間上限 s (預設 1.5 = PRE_DIVE 推頭約 1.4 s)')
    args = ap.parse_args()

    link = UartVisionLink(args.port, args.baud,
                          mode_resend_s=cfg.GIMBAL_MODE_RESEND_S,
                          mode_max_tries=cfg.GIMBAL_MODE_MAX_TRIES,
                          lock_key=cfg.GIMBAL_LOCK_KEY,
                          lock_pitch_deg=cfg.GIMBAL_LOCK_PITCH_DEG,
                          lock_tol_deg=cfg.GIMBAL_LOCK_TOL_DEG)

    os.makedirs('bench_logs', exist_ok=True)
    path = os.path.join('bench_logs',
                        f"gimbal_mode_test_{datetime.now():%Y%m%d_%H%M%S}.csv")
    fcsv = open(path, 'w', newline='', buffering=1)
    w = csv.writer(fcsv)
    w.writerow(['t', 'state', 'req_key', 'mode', 'pitch', 'roll', 'locked',
                'valid', 'off_x', 'off_y', 'rx_age_ms', 'e2e_ms', 'result_id'])

    print("=" * 72)
    print(f" 雲台模式鍵地面測試  {args.port} @ {args.baud}")
    print(f" 對應表 GIMBAL_MODE_KEYS = {cfg.GIMBAL_MODE_KEYS}")
    print(f" 鎖定判定: mode={cfg.GIMBAL_LOCK_KEY} 且 pitch = "
          f"{cfg.GIMBAL_LOCK_PITCH_DEG:+.0f}° ± {cfg.GIMBAL_LOCK_TOL_DEG:.0f}°")
    if args.auto:
        print(f" 自動模式: {' → '.join(f'{s}({d:.0f}s)' for s, d in AUTO_SEQ)}"
              f" × {args.repeat}")
    else:
        print(" 按鍵: b=STANDBY s=SEARCH p=PRE_DIVE d=DIVE o=PULLOUT | "
              "0/1/2=直接送鍵 | q=結束")
    print("=" * 72)
    link.start()

    keys = Keys()
    t0 = time.monotonic()
    state = None
    results = []          # 每次要求一筆: state, key, t_req, confirm, settle, hold_err
    cur = None
    auto_plan = [x for _ in range(max(1, args.repeat)) for x in AUTO_SEQ] if args.auto else []
    auto_i, auto_next = 0, t0
    last_print = 0.0

    def set_state(new_state, key):
        nonlocal state, cur
        state = new_state
        prev_req = link.mode_status()[0]
        link.request_mode(key)
        tm = link.mode_timing()
        if key is not None and tm['t_req'] is not None and (cur is None or cur['t_req'] != tm['t_req']):
            cur = {'state': new_state, 'key': key, 't_req': tm['t_req'],
                   'confirm': None, 'settle': None, 'hold_err': 0.0}
            results.append(cur)
        if key is None:
            what = "不送鍵 (維持對方目前模式)"
        elif key == prev_req:
            what = f"維持 '{key}' (與上一狀態同鍵, 不重送)"
        else:
            what = (f"送模式鍵 '{key}' "
                    f"({MODE_NAMES[int(key)] if key.isdigit() and int(key) < 3 else '?'})")
        print(f"\n[{time.monotonic() - t0:6.2f}s] ── 任務狀態 → {new_state}: {what}",
              flush=True)

    try:
        while True:
            now = time.monotonic()
            # ── 狀態切換: 自動排程 或 按鍵 ──
            if auto_plan:
                if now >= auto_next:
                    if auto_i >= len(auto_plan):
                        break
                    st, dur = auto_plan[auto_i]
                    auto_i += 1
                    auto_next = now + dur
                    set_state(st, cfg.GIMBAL_MODE_KEYS.get(st))
            k = keys.get()
            if k in ('q', 'Q', '\x1b'):
                break
            if k in KEY_TO_STATE and not auto_plan:
                st = KEY_TO_STATE[k]
                set_state(st, cfg.GIMBAL_MODE_KEYS.get(st))
            elif k in ('0', '1', '2') and not auto_plan:
                set_state(f'KEY{k}', k)

            # ── 讀最新遙測 + 量時間 ──
            pkt = link.latest()
            tm = link.mode_timing()
            if cur is not None and tm['t_req'] == cur['t_req']:
                if tm['t_confirm'] is not None and cur['confirm'] is None:
                    cur['confirm'] = tm['t_confirm'] - tm['t_req']
                    print(f"[{now - t0:6.2f}s]    ✓ 對方回報 mode={cur['key']} "
                          f"(要求後 {cur['confirm']:.2f}s)", flush=True)
                if tm['t_settled'] is not None and cur['settle'] is None:
                    cur['settle'] = tm['t_settled'] - tm['t_req']
                    print(f"[{now - t0:6.2f}s]    ✓ 鎖定到位 pitch={tm['pitch_settled']:+.1f}° "
                          f"(要求後 {cur['settle']:.2f}s)", flush=True)
                if (cur['settle'] is not None and pkt is not None
                        and math.isfinite(pkt['pitch'])):
                    cur['hold_err'] = max(cur['hold_err'],
                                          abs(pkt['pitch'] - cfg.GIMBAL_LOCK_PITCH_DEG))

            req, _ok = link.mode_status()
            rx_age = (now - pkt['rx_mono']) * 1000.0 if pkt else None
            w.writerow([f"{now - t0:.3f}", state or '', req or '',
                        pkt['mode'] if pkt else '', _f(pkt['pitch'], 2) if pkt else '',
                        _f(pkt['roll'], 2) if pkt else '', int(link.is_locked(pkt)),
                        int(pkt['valid']) if pkt else '',
                        _f(pkt['offset_x']) if pkt else '', _f(pkt['offset_y']) if pkt else '',
                        f"{rx_age:.1f}" if rx_age is not None else '',
                        _f(pkt['e2e_ms'], 1) if pkt and pkt['e2e_ms'] is not None else '',
                        pkt['result_id'] if pkt else ''])

            if now - last_print >= 0.5:
                last_print = now
                if pkt is None:
                    print(f"[{now - t0:6.2f}s] 等待對方資料... (串口 "
                          f"{'已開' if link.connected else '未開'}, "
                          f"收 {link.stats['bytes']} bytes, CRC 錯 {link.stats['crc']})",
                          flush=True)
                else:
                    m = pkt['mode']
                    print(f"[{now - t0:6.2f}s] {state or '-':8s} 要求={req or '-'} | "
                          f"對方 mode={m}({MODE_NAMES[m]}) pitch={_f(pkt['pitch'])}° "
                          f"{'[LOCKED]' if link.is_locked(pkt) else '        '} "
                          f"目標偏移=({_f(pkt['offset_x'], 0)},{_f(pkt['offset_y'], 0)})px "
                          f"資料延遲={rx_age:.0f}ms", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        keys.close()
        link.stop()
        fcsv.close()

    # ── 總結 ──
    print("\n" + "=" * 72)
    print(" 結果總結")
    print("=" * 72)
    if not results:
        print(" (沒有送出任何模式鍵)")
    allpass = True
    for r in results:
        line = f" {r['state']:9s} 鍵 '{r['key']}': "
        ok = True
        if r['confirm'] is None:
            line += "✗ 對方沒有回報此模式"
            ok = False
        else:
            c_ok = r['confirm'] <= args.max_confirm
            ok &= c_ok
            line += f"確認 {r['confirm']:.2f}s {'✓' if c_ok else '✗ 太慢'}"
        if r['key'] == cfg.GIMBAL_LOCK_KEY:
            if r['settle'] is None:
                line += " | ✗ pitch 沒有到 " + f"{cfg.GIMBAL_LOCK_PITCH_DEG:+.0f}°"
                ok = False
            else:
                s_ok = r['settle'] <= args.max_settle
                h_ok = r['hold_err'] <= cfg.GIMBAL_LOCK_TOL_DEG
                ok &= s_ok and h_ok
                line += (f" | 到位 {r['settle']:.2f}s {'✓' if s_ok else '✗ 太慢'}"
                         f" | 到位後最大誤差 {r['hold_err']:.1f}° {'✓' if h_ok else '✗ 沒停住'}")
        allpass &= ok
        print(line)
    print("-" * 72)
    print(f" 串口統計: {link.stats}")
    print(f" 判定: {'★ 全部通過' if results and allpass else '★ 有項目未通過, 見上'}")
    print(f" log → {path}")


if __name__ == '__main__':
    main()

# -*- coding: utf-8 -*-
"""
xwing_uart_vision.py — 接收對方實驗室 Orin NX 送來的雲台視覺資料 (UART v6/v7)
═══════════════════════════════════════════════════════════════════
架構:
    [雲台相機] → [對方 Orin NX: 辨識 + 雲台控制] ──UART──▶ [USB-UART] → [本機 Orin NX]
                                                ◀──UART── 模式鍵 '0'/'1'/'2' + v7 校時請求

協定: 直接 import 對方提供的 uart_mipi_receiver.py (封包格式 / CRC / 校時),
      保證兩邊解碼一致. 對方改協定時, 換掉那支檔案即可.

本模組做的事 (都在背景執行緒, 不會卡 50 Hz 控制迴圈):
  · 讀 UART、重新同步、驗 CRC、解出 16 個欄位
  · v7: 自動送校時請求, 算「取像 → 本機收到」的延遲, 並換算成本機時間軸的取像時刻
  · 依任務狀態送模式鍵給對方雲台 (預設關閉, 見 GIMBAL_MODE_CMD_ENABLED),
    並用回傳的 mode 欄位確認, 沒確認就重送
  · USB 拔插 / 串口錯誤 → 自動重開, 不讓主程式掛掉

主程式只要:
    vis = UartVisionLink(port, baud); vis.start()
    pkt = vis.latest()          # 最新一筆 (dict) 或 None
    vis.request_mode('1')       # 要求對方切模式 (None = 不要求)
    vis.stop()
"""

import math
import os
import struct
import threading
import time

try:
    import serial
    from uart_mipi_receiver import (
        FRAME_MAGIC, HEADER_FMT, HEADER_SIZE, CRC_SIZE, BASE_SIZE,
        BASE_FIELDS, MODE_NAMES, decode_frame, UARTClock,
    )
    UART_OK = True
    UART_IMPORT_ERROR = None
except ImportError as _e:          # 沒裝 pyserial 時, 主程式仍可在關閉視覺下執行
    UART_OK = False
    UART_IMPORT_ERROR = _e
    BASE_FIELDS = []
    MODE_NAMES = ("手動", "追蹤", "鎖定")

# 對方協定允許的 (version, flags) → payload 長度 (與 uart_mipi_receiver 相同)
_EXPECTED = None


def _expected_lengths():
    global _EXPECTED
    if _EXPECTED is None:
        _EXPECTED = {(6, 0): BASE_SIZE, (7, 0): BASE_SIZE + 12, (7, 2): 28}
    return _EXPECTED


def _finite(x):
    return x is not None and isinstance(x, (int, float)) and math.isfinite(x)


# ══════════════════════════════════════════════════════════════════
#  主程式寫進 CSV 的欄位
# ══════════════════════════════════════════════════════════════════
VISION_CSV_FIELDS = [
    'vis_link',          # 1 = 串口開著且近期有有效封包
    'vis_ver',           # 封包版本 6 / 7
    'vis_rx_age_ms',     # 最新封包是多久前收到的 (本機)
    'vis_result_id', 'vis_new',          # vis_new=1: 這拍拿到新的辨識結果
    'vis_valid',         # 1 = 有目標且數值有限
    'vis_tgt_cx', 'vis_tgt_cy', 'vis_off_x', 'vis_off_y',
    'vis_fx', 'vis_fy', 'vis_cx', 'vis_cy', 'vis_img_w', 'vis_img_h',
    'vis_gimbal_pitch', 'vis_gimbal_roll', 'vis_gimbal_mode',
    'vis_det_age_ms', 'vis_angle_age_ms',  # 對方送出時, 辨識 / 角度資料的年齡
    'vis_e2e_ms', 'vis_e2e_err_ms',      # v7: 取像 → 本機收到 (含校時誤差)
    'vis_cap_age_ms',    # v7: 控制這一拍時, 該影像已多舊
    'vis_err_u_deg', 'vis_err_v_deg',    # 影像角度誤差 (右 / 下為正)
    'vis_u_ctrl', 'vis_v_ctrl',          # 換算到虛擬 (控制) 相機像素, 可直接跟 virt_bbox_u/v 比
    'gimbal_state', 'gimbal_mode_req', 'gimbal_mode_ok',
    'gimbal_locked',     # 1 = 對方回報鎖定模式, 且 pitch 已到 −30° ± 容差 (機械到位)
    'vis_frames', 'vis_crc_err',
]


class UartVisionLink:
    """背景執行緒收 UART 視覺資料 + 送模式鍵. 所有方法都可從控制執行緒安全呼叫."""

    def __init__(self, port, baud=115200, mode_resend_s=1.0, mode_max_tries=3,
                 log=print, lock_key='2', lock_pitch_deg=-30.0, lock_tol_deg=2.0):
        if not UART_OK:
            raise ImportError(f"UART 視覺需要 pyserial 與 uart_mipi_receiver.py "
                              f"({UART_IMPORT_ERROR}); 請 pip3 install pyserial")
        self.port = port
        self.baud = int(baud)
        self.mode_resend_s = float(mode_resend_s)
        self.mode_max_tries = int(mode_max_tries)
        self.log = log

        self._lock = threading.Lock()
        self._ser = None
        self._thread = None
        self._running = False
        self._buf = bytearray()
        self.clock = UARTClock()
        self._clock_enabled = False

        self._latest = None            # 最新一筆解碼結果 (dict)
        self.lock_key = lock_key
        self.lock_pitch_deg = float(lock_pitch_deg)
        self.lock_tol_deg = float(lock_tol_deg)
        # 最近一次模式要求的時間軸 (monotonic 秒): 要求 → 對方回報同模式 → 鎖定到位
        self._timing = {'key': None, 't_req': None, 't_confirm': None,
                        't_settled': None, 'pitch_settled': None}
        self._mode_req = None          # 要求的模式鍵 (str) 或 None
        self._mode_tries = 0
        self._mode_sent_t = 0.0
        self._mode_warned = False

        self._open_fail = 0
        self.stats = {'frames': 0, 'clock': 0, 'bytes': 0, 'header': 0,
                      'crc': 0, 'payload': 0, 'discarded': 0, 'reopen': 0,
                      'mode_tx': 0, 'serial_err': 0, 'open_fail': 0}
        self.connected = False

    # ------------------------------------------------------------------
    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, name='uart_vision',
                                        daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._close()

    def latest(self):
        """最新一筆封包 (dict 副本) 或 None."""
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def request_mode(self, key):
        """要求對方雲台切到模式鍵 key ('0'/'1'/'2' 或對方新增的鍵); None = 不要求.
        同一個 key 重複呼叫不會重送; 換 key 才重新開始送."""
        if key is not None:
            key = str(key)
            if len(key) != 1:
                raise ValueError(f"模式鍵必須是單一字元, 收到 {key!r}")
        with self._lock:
            if key == self._mode_req:
                return
            self._mode_req = key
            self._mode_tries = 0
            self._mode_sent_t = 0.0
            self._mode_warned = False
            self._timing = {'key': key, 't_req': time.monotonic(), 't_confirm': None,
                            't_settled': None, 'pitch_settled': None}

    def is_locked(self, pkt):
        """這筆遙測是否「鎖定且 pitch 到位」."""
        if pkt is None or not self.lock_key.isdigit():
            return False
        return (pkt.get('mode') == int(self.lock_key) and _finite(pkt.get('pitch'))
                and abs(pkt['pitch'] - self.lock_pitch_deg) <= self.lock_tol_deg)

    def mode_timing(self):
        """最近一次模式要求的時間軸 (dict 副本):
        key, t_req, t_confirm (對方回報同模式), t_settled (鎖定 pitch 到位), pitch_settled."""
        with self._lock:
            return dict(self._timing)

    def mode_status(self):
        """(要求的鍵, 是否已由對方回報確認)."""
        with self._lock:
            req = self._mode_req
            rep = self._latest.get('mode') if self._latest else None
        return req, (req is not None and req.isdigit() and rep == int(req))

    # ------------------------------------------------------------------
    def _open(self):
        try:
            self._ser = serial.Serial(
                self.port, self.baud, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=0.01, write_timeout=0.1,
                xonxoff=False, rtscts=False, dsrdtr=False)
            self._buf.clear()
            self.connected = True
            self.log(f"[UARTVision] 已開啟 {self.port} @ {self.baud} 8N1"
                     + (f" (失敗 {self._open_fail} 次後恢復)" if self._open_fail else ""))
            self._open_fail = 0
            return True
        except (serial.SerialException, OSError) as e:
            self._ser = None
            self.connected = False
            self._open_fail += 1
            self.stats['open_fail'] += 1
            # 第一次與之後每 10 次才印, 避免 USB 拔掉時每秒洗版
            if self._open_fail == 1 or self._open_fail % 10 == 0:
                self.log(f"[UARTVision] ★ 開不了 {self.port}: {e} "
                         f"(第 {self._open_fail} 次, 每 1 s 重試)")
            return False

    def _close(self):
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self.connected = False

    def _loop(self):
        last_open_try = 0.0
        while self._running:
            if self._ser is None:
                now = time.monotonic()
                if now - last_open_try < 1.0:
                    time.sleep(0.05)
                    continue
                last_open_try = now
                if not self._open():
                    continue
                if self.stats['frames'] or self.stats['reopen']:
                    self.stats['reopen'] += 1
            try:
                if self._clock_enabled:
                    self.clock.request(self._ser)     # v7 校時, 內部每 2 s 一次
                self._service_mode_tx()
                n = self._ser.in_waiting
                data = self._ser.read(min(4096, max(1, n)))
                rx_ns = time.monotonic_ns()
                if data:
                    self.stats['bytes'] += len(data)
                    self._buf.extend(data)
                    while True:
                        got = self._extract()
                        if got is None:
                            break
                        self._handle(got[0], got[1], rx_ns)
            except (serial.SerialException, OSError) as e:
                self.stats['serial_err'] += 1
                self.log(f"[UARTVision] ★ 串口錯誤: {e} → 關閉後重開")
                self._close()
            except Exception as e:                  # 解碼以外的意外, 不讓執行緒死掉
                self.stats['serial_err'] += 1
                self.log(f"[UARTVision] ★ 例外: {type(e).__name__}: {e}")
                time.sleep(0.05)

    # ------------------------------------------------------------------
    def _extract(self):
        """從緩衝取出一個通過 CRC 的封包 → (frame bytes, values) 或 None.
        重新同步邏輯同 uart_mipi_receiver.UARTReceiver._extract_frame."""
        buf = self._buf
        expected = _expected_lengths()
        while True:
            idx = buf.find(FRAME_MAGIC)
            if idx < 0:
                keep = 1 if buf.endswith(FRAME_MAGIC[:1]) else 0
                self.stats['discarded'] += len(buf) - keep
                del buf[:len(buf) - keep]
                return None
            if idx:
                self.stats['discarded'] += idx
                del buf[:idx]
            if len(buf) < HEADER_SIZE:
                return None
            _, ver, flags, length, lx = struct.unpack(HEADER_FMT, buf[:HEADER_SIZE])
            exp = expected.get((ver, flags))
            if exp is None or length != exp or lx != (length ^ 0xFFFF):
                self.stats['header'] += 1
                del buf[0]
                continue
            size = HEADER_SIZE + length + CRC_SIZE
            if len(buf) < size:
                return None
            frame = bytes(buf[:size])
            try:
                values, _crc = decode_frame(frame)
            except ValueError as e:
                self.stats['crc' if 'CRC' in str(e) else 'payload'] += 1
                del buf[0]
                continue
            del buf[:size]
            return frame, values

    def _handle(self, frame, values, rx_ns):
        version = frame[2]
        if values is None:                    # v7 flags=2: 校時回覆
            self.clock.accept(frame, rx_ns)
            self.stats['clock'] += 1
            return
        self._clock_enabled = (version == 7)
        self.stats['frames'] += 1
        d = dict(zip((name for name, _ in BASE_FIELDS), values))
        d['ver'] = version
        d['rx_mono'] = rx_ns / 1e9
        d['e2e_ms'] = d['e2e_err_ms'] = d['cap_mono'] = None
        if version == 7:
            e2e, err, cap_mono = self._latency(frame, rx_ns)
            d['e2e_ms'], d['e2e_err_ms'], d['cap_mono'] = e2e, err, cap_mono
        d['valid'] = all(_finite(d.get(k)) for k in
                         ('target_cx', 'target_cy', 'offset_x', 'offset_y', 'fx', 'fy')) \
            and d['fx'] > 0 and d['fy'] > 0
        with self._lock:
            self._latest = d
            tm = self._timing
            k = tm['key']
            if k is not None and k.isdigit() and d['mode'] == int(k):
                now = rx_ns / 1e9
                if tm['t_confirm'] is None:
                    tm['t_confirm'] = now
                if (k == self.lock_key and tm['t_settled'] is None
                        and self.is_locked(d)):
                    tm['t_settled'] = now
                    tm['pitch_settled'] = d['pitch']

    def _latency(self, frame, rx_ns):
        """v7: (取像→收到 ms, 校時不確定 ms, 取像時刻(本機 monotonic 秒)).
        與 UARTClock.estimate 同一套算法, 但回傳數值而非文字."""
        capture, session = struct.unpack('>QI', frame[HEADER_SIZE + BASE_SIZE:-CRC_SIZE])
        if not capture:
            return None, None, None
        cands = [s for s in self.clock.samples
                 if s[1] == session and 0 <= rx_ns - s[0] <= 10_000_000_000]
        if not cands:
            return None, None, None
        t_s, _, offset, unc = min(cands, key=lambda s: s[3])
        unc += (rx_ns - t_s) * 0.0002            # 兩端合計 200 ppm 漂移假設
        value = (rx_ns + offset - capture) / 1e6
        err = unc / 1e6
        if value < -err:
            return None, None, None
        return value, err, (capture - offset) / 1e9

    def _service_mode_tx(self):
        with self._lock:
            key = self._mode_req
            rep = self._latest.get('mode') if self._latest else None
            tries, sent_t = self._mode_tries, self._mode_sent_t
        if key is None:
            return
        if key.isdigit() and rep == int(key):
            return                                  # 對方已回報同一模式
        now = time.monotonic()
        if tries >= self.mode_max_tries:
            if not self._mode_warned:
                self._mode_warned = True
                self.log(f"[UARTVision] ★ 模式鍵 '{key}' 送了 {tries} 次仍未確認 "
                         f"(對方回報 mode={rep})")
            return
        if now - sent_t < self.mode_resend_s:
            return
        if self._ser.write(key.encode('ascii')) != 1:
            raise OSError('模式鍵寫入不完整')
        self.stats['mode_tx'] += 1
        with self._lock:
            self._mode_tries = tries + 1
            self._mode_sent_t = now


# ══════════════════════════════════════════════════════════════════
#  主程式用: 把一筆封包換成 CSV 欄位
# ══════════════════════════════════════════════════════════════════
def vision_row(link, pkt, last_result_id, now_mono, ctrl_fx, ctrl_fy,
               ctrl_cx, ctrl_cy, gimbal_state, stale_s):
    """回傳 (row dict, result_id). link=None 表示視覺未啟用 → 全部留空."""
    row = dict.fromkeys(VISION_CSV_FIELDS, '')
    if link is None:
        return row, last_result_id
    req, ok = link.mode_status()
    row['gimbal_locked'] = int(link.is_locked(pkt))
    row.update(gimbal_state=gimbal_state,
               gimbal_mode_req=req if req is not None else '',
               gimbal_mode_ok=int(ok),
               vis_frames=link.stats['frames'], vis_crc_err=link.stats['crc'])
    if pkt is None:
        row['vis_link'] = 0
        return row, last_result_id

    def r(x, nd=2):
        return round(x, nd) if _finite(x) else ''

    rx_age = (now_mono - pkt['rx_mono']) * 1000.0
    rid = pkt['result_id']
    row.update(
        vis_link=int(link.connected and rx_age <= stale_s * 1000.0),
        vis_ver=pkt['ver'], vis_rx_age_ms=r(rx_age, 1),
        vis_result_id=rid, vis_new=int(rid != last_result_id),
        vis_valid=int(pkt['valid']),
        vis_tgt_cx=r(pkt['target_cx'], 1), vis_tgt_cy=r(pkt['target_cy'], 1),
        vis_off_x=r(pkt['offset_x'], 1), vis_off_y=r(pkt['offset_y'], 1),
        vis_fx=r(pkt['fx'], 1), vis_fy=r(pkt['fy'], 1),
        vis_cx=r(pkt['cx'], 1), vis_cy=r(pkt['cy'], 1),
        vis_img_w=r(pkt['image_width'], 0), vis_img_h=r(pkt['image_height'], 0),
        vis_gimbal_pitch=r(pkt['pitch']), vis_gimbal_roll=r(pkt['roll']),
        vis_gimbal_mode=pkt['mode'],
        vis_det_age_ms=r(pkt['detection_age_ms'], 1),
        vis_angle_age_ms=r(pkt['angle_age_ms'], 1),
        vis_e2e_ms=r(pkt['e2e_ms']), vis_e2e_err_ms=r(pkt['e2e_err_ms']),
        vis_cap_age_ms=(r((now_mono - pkt['cap_mono']) * 1000.0, 1)
                        if pkt['cap_mono'] is not None else ''),
    )
    if pkt['valid']:
        ex, ey = pkt['offset_x'] / pkt['fx'], pkt['offset_y'] / pkt['fy']
        row.update(vis_err_u_deg=r(math.degrees(math.atan(ex)), 3),
                   vis_err_v_deg=r(math.degrees(math.atan(ey)), 3),
                   # 同一視角換到虛擬相機: u = cx_v + fx_v · tan(角度)
                   vis_u_ctrl=r(ctrl_cx + ctrl_fx * ex, 1),
                   vis_v_ctrl=r(ctrl_cy + ctrl_fy * ey, 1))
    return row, rid


def gimbal_state_label(guided, armed, mission_mode):
    """任務狀態 → 給雲台的狀態名稱. 還沒撥 GUIDED / 還沒 ARM = STANDBY."""
    if not guided or not armed:
        return 'STANDBY'
    return mission_mode

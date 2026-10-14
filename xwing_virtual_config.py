# -*- coding: utf-8 -*-
"""
xwing_virtual_config.py — xwing_virtual 版全部參數 (純虛擬軌跡, 無視覺)
═══════════════════════════════════════════════════════════════════
任務: AUTO 繞圈對準 → 撥 GUIDED 開始 → SEARCH(朝虛擬目標直飛)
      → PRE_DIVE(推頭到 -30°) → 30° DIVE(虛擬目標投影 PID)
      → PULLOUT(120 m 拉平) → TECS 120 m 平飛
感知: ★ 無相機 / 無 YOLO / 無雲台 / 無 DKF.
      目標 = 已知 GPS 起點 + 方位 + 車速 的虛擬軌跡, 每拍用飛機真實狀態
      投影成「虛擬畫面」(u, v) 餵給 DIVE 段 PID (dive_guidance 原封不動).
機體: X-wing 固定翼 (巡航 30 m/s, 速度上限 60 m/s)

與原 config.py 的差異 (★ = 本版改的值, 原值寫在註解):
  CRUISE_AGL_M      120 → 240     巡航 / 進場高度
  DIVE_ANGLE_DEG     25 → 30      俯衝角
  PULLOUT_AGL_M      70 → 120     俯衝結束 (拉平) 高度
  TECS_HOLD_ALT_M    70 → 120     拉平後 TECS 保持高度
  V_CRUISE_MPS       20 → 30      巡航空速
  SEARCH_SPEED_MPS   16 → 30      SEARCH 空速 (= AUTO 巡航)
  TECS_SPEED_MPS     18 → 30      拉平後空速
  PULLOUT_VMAX_MPS   30 → 60      超速提前拉平門檻 (= 速度上限)
  PREDIVE_DIST_M    257 → 415.69  推頭距離 = 240 / tan30° (移動版)
  STATIC_PREDIVE_DIST_M 257 → 415.69 推頭距離 = 240 / tan30° (靜止版)
  目標座標          換成新場地 (見下方「虛擬目標」)
  其餘視覺 / 相機 / 雲台 / YOLO / 錄影參數全部移除.
"""

import math


class XwingVirtualConfig:
    # ════════════════════════════════════════════════════════════
    #  任務幾何
    # ════════════════════════════════════════════════════════════
    CRUISE_AGL_M = 240.0        # ★ 巡航/進場高度 (AUTO 繞圈高度)       原 120
    DIVE_ANGLE_DEG = 30.0       # ★ 俯衝角 (正值)                       原 25
    PULLOUT_AGL_M = 120.0       # ★ 俯衝 240 → 120 m, 到 120 m 拉平      原 70
    PULLOUT_VMAX_MPS = 60.0     # ★ 速度上限: 空速 ≥ 此值即提前拉平      原 30
                                #   (安全上限. 正常剖面峰值見 xwing_virtual_sim.py,
                                #    若 60 是結構極限, 建議改 55 留反應餘裕)
    TECS_HOLD_ALT_M = 120.0     # ★ 拉平後 TECS 保持高度 (= PULLOUT_AGL)  原 70
                                #   (進 PRE_DIVE 時就先送, 讓 TECS 提早收斂)
    TECS_SPEED_MPS = 30.0       # ★ 拉平 / DONE 的 TECS 目標空速          原 18
    SEARCH_SPEED_MPS = 30.0     # ★ SEARCH 巡航空速 (與 AUTO 一致)        原 16
    #   ⚠ 43000 CHANGE_SPEED 超出飛控空速上下限會被拒 (result=2):
    #     必須 AIRSPEED_MIN ≤ 30 ≤ AIRSPEED_MAX (舊韌體 ARSPD_FBW_MIN/MAX).
    GUIDED_ENTRY_ROLL0_S = 0.5

    # 攻角
    ALPHA_NOM_DEG = 1.6         # ★ 巡航攻角. 原 3.5 是 20 m/s 的值;
                                #   升力 ∝ V² → 3.5×(20/30)² ≈ 1.6 (估計, 待 log 校)
                                #   (SEARCH 段 pitch 交 TECS, 此值只進 log)
    ALPHA_DIVE_DEG = 0.0        # 俯衝攻角 — 瞄準鏈用這個 (沿用)

    # SEARCH → PRE_DIVE: 飛機到虛擬目標水平距 ≤ 門檻 → 直接命令 -DIVE_ANGLE
    PREDIVE_HANDOFF_PITCH_TOL_DEG = 3.0   # pitch 壓到 -30±3° 內交棒 DIVE
    ENTRY_MIN_AGL_M = 200.0     # ★ 低於此高度不准進 PRE_DIVE (原寫死 60;
                                #   240 m 進場掉到 200 m 以下代表幾何已不對)
    PREDIVE_TIMEOUT_S = 45.0

    # ════════════════════════════════════════════════════════════
    #  空速 / 油門
    # ════════════════════════════════════════════════════════════
    V_CRUISE_MPS = 30.0         # ★ 巡航空速                               原 20
    V_STALL_MPS = 12.0
    THR_TRIM = 0.45             # ⚠ 20 m/s 時的配平油門. 30 m/s 需較高,
                                #   請填飛控 AUTO 30 m/s 巡航時的 TRIM_THROTTLE/100.
                                #   (只影響 PRE_DIVE 推頭那 1~2 秒的油門)
    THR_MIN = 0.05              # DIVE 怠速
    THR_MAX = 0.85

    # ════════════════════════════════════════════════════════════
    #  安全
    # ════════════════════════════════════════════════════════════
    ABORT_MIN_AGL_M = 100.0     # ★ 低於此高度 ABORT 拉起 (原 55, 拉平高度上移)
                                #   PULLOUT 120 m 先觸發; 這是最後一道
    ABORT_TARGET_LOST_S = 1.5   # 虛擬目標永遠有效, 此值實際不會觸發

    # ════════════════════════════════════════════════════════════
    #  虛擬相機 (★ 只是控制座標系, 沒有實體相機)
    #  DIVE 段 PID 吃「虛擬目標投影的像素誤差」, 所以仍需一組內參.
    #  數值沿用原 1x 標定值 → PID 增益的物理意義與舊版一致.
    #  (姿態命令與焦距無關: 投影乘 FX、PID 換角度除 FX, 互相抵銷)
    # ════════════════════════════════════════════════════════════
    IMG_W = 1920
    IMG_H = 1080
    FX_1X = 1066.7
    FY_1X = 1059.5
    CX_1X = 959.4932
    CY_1X = 545.0292
    VIRTUAL_ZOOM_LEVEL = 1.0    # 1x: DIVE 偏離中止門檻 37.8° (寬, 不會誤判)

    CONTROL_HZ = 50.0           # 控制迴路頻率 (MAVROS + bridge)

    # ════════════════════════════════════════════════════════════
    #  DIVE 段虛擬目標 PID (語意見 dive_guidance.GuidanceConfig) — 沿用
    #  ⚠ 俯衝速度由 ~36 m/s 提高到 ~50 m/s, 舵面效率變高, 首飛請看
    #    pid_dtheta_deg / rudder 是否振盪, 必要時 KP 減半.
    # ════════════════════════════════════════════════════════════
    PID_LAT_KP = 2.0
    PID_LAT_KI = 0.20
    PID_LAT_KD = 0.0
    PID_LON_KP = 0.8
    PID_LON_KI = 0.1
    PID_LON_KD = 0.2
    PID_SP_LEAD_S = 0.0
    PID_MEAS_TAU_S = 0.10
    PID_D_TAU_S = 0.20
    PID_OUT_TAU_S = 0.0

    # ════════════════════════════════════════════════════════════
    #  DIVE / PRE_DIVE 橫向: 方向舵迴路 (沿用原值, 只改空速排程)
    # ════════════════════════════════════════════════════════════
    RUDDER_LIMIT_DEG = 30.0
    K_PSI = 0.30
    K_R = 0.15
    K_LAT_TO_RUDDER = 0.50
    RUDDER_SLEW_DPS = 20.0
    # ★ 空速排程重新啟用 (原 ((999,1.0),) = 不排程).
    #   原版俯衝峰值 ~36 m/s, 不排程還可以; 本版峰值可達 50~60 m/s,
    #   動壓是 30 m/s 的 2.8~4 倍, 同一舵量的力矩也大 2.8~4 倍.
    #   倍率 ≈ (30/V)² → 維持「舵量 × 動壓」大致不變.
    RUDDER_VSCHED = ((30.0, 1.00), (40.0, 0.56), (50.0, 0.36), (999.0, 0.25))
    ROLL_PROT_FULL_DEG = 25.0
    ROLL_PROT_ZERO_DEG = 35.0

    # ════════════════════════════════════════════════════════════
    #  虛擬目標 (控制路徑唯一的真相來源)
    # ════════════════════════════════════════════════════════════
    MOVING_TARGET = True        # True=移動 (預設); False=靜止 (停在起點)
    # ★ 新場地: 23°43'19.6"N 120°22'47.6"E
    MOVING_START_GPS = (23.7221111, 120.3798889)   # 目標起點 (切 GUIDED 時在此)
    MOVING_HEADING_GPS = (23.720997, 120.379882)   # 朝此點走 → 方位 ≈ 180.3° (往南)
    MOVING_HEADING_DEG = None   # 直接指定方位 (非 None 則覆寫上面兩點的算法)
    TARGET_SPEED_MPS = 10.0     # 目標速度 (等速, 沿用原值)
    # ★ 起點→航向點只有 ~124 m. 10 m/s 走完只要 12.4 s, 而 SEARCH+俯衝
    #   通常要 20 s 以上 → 預設讓虛擬目標「走過航向點後繼續沿同方位前進」
    #   (與原版行為相同). 若要停在航向點, 改 True.
    MOVING_STOP_AT_HEADING_GPS = False
    MOVING_TARGET_LEAD = False  # 前置瞄準 (沿用: 關)
    #   模擬比較 (推頭 415.69 m): 關 → 俯衝 ≈ −28.8°, 外插 miss ≈ 70 m (打在車後);
    #                             開 → 俯衝 ≈ −23.2°, 外插 miss ≈ 6 m.
    #   原版關掉是因為真實相機視野會丟車; 純虛擬沒有這個限制, 要開請
    #   同時把 PREDIVE_DIST_M 降低 (俯衝會變淺), 先用模擬確認.

    # 靜止版 (--static): 目標固定在這個 GPS (預設 = 移動版起點)
    TARGET_GPS = MOVING_START_GPS

    # ★ 推頭距離: 飛機到目標水平距 ≤ 此值 → PRE_DIVE (直接命令 -30°)
    #   = 巡航高度 / tan(俯衝角) = 240 / tan30° = 415.69 m (純幾何, 使用者指定)
    #   直接由 CRUISE_AGL_M / DIVE_ANGLE_DEG 算出, 改高度或角度會自動跟著變.
    #   移動 / 靜止共用同一個值.
    #   (模擬參考: 移動目標 DIVE 平均 ≈ −28.8°, 靜止 ≈ −31.2°, 原因是推頭
    #    需要 ~1.4 s 及追擊時視線變淺. 實飛後可依 log 的 glide_slope_err_deg
    #    用 --predive-dist 微調: 比 −30° 陡 → 加大; 淺 → 減小, 每 10 m ≈ 0.6°.)
    PREDIVE_DIST_M = CRUISE_AGL_M / math.tan(math.radians(DIVE_ANGLE_DEG))
    #                               ★ 移動版 415.69 m               原 257
    STATIC_PREDIVE_DIST_M = PREDIVE_DIST_M   # ★ 靜止版 415.69 m     原 257

    # ════════════════════════════════════════════════════════════
    #  雲台視覺 UART (對方實驗室 Orin NX → USB-UART → 本機)
    #  協定見 uart_mipi_receiver.py, 整合說明見 xwing_virtual_GIMBAL_UART.md
    #  ★ 目前只「接收 + 記錄」, 不進飛機控制 (控制仍是虛擬目標).
    # ════════════════════════════════════════════════════════════
    VISION_UART_ENABLED = False      # True 或命令列 --vision-uart 才啟用
    VISION_UART_PORT = '/dev/ttyUSB0'
    #   ⚠ 建議改成固定名稱 /dev/serial/by-id/usb-...,
    #     ttyUSB0/1 的編號開機時可能互換 (尤其飛控也走 USB 時).
    VISION_UART_BAUD = 115200        # 必須與對方發送端相同
    VISION_STALE_S = 2.0             # 超過這麼久沒收到有效封包 → 告警

    # ── 依任務狀態送模式鍵給對方雲台 ──
    #   對方協定目前只有 ASCII '0'=手動 '1'=追蹤 '2'=鎖定, 沒有 FPV (機身固定).
    #   ★ 預設關閉: 模式對應要先跟對方確認, 見 xwing_virtual_GIMBAL_UART.md 第 4 節.
    GIMBAL_MODE_CMD_ENABLED = False  # True 或命令列 --gimbal-cmd 才送
    GIMBAL_MODE_KEYS = {             # 任務狀態 → 模式鍵 (None = 不要求, 交給對方)
        'STANDBY':  None,            # 還沒撥 GUIDED: 對方自行處理 (原版: FOLLOW 看前方)
        'SEARCH':   '1',             # 追蹤目標 (原版: FOLLOW + 追蹤)
        'PRE_DIVE': '1',             # 追蹤目標 (原版: FOLLOW + 追蹤)
        'DIVE':     None,            # ★ 原版: FPV 機身固定. 對方新增 FPV 鍵後填入
        'PULLOUT':  None,
        'DONE':     None,
        'ABORT':    None,
    }
    GIMBAL_MODE_RESEND_S = 1.0       # 沒收到對方回報同一模式 → 隔多久重送
    GIMBAL_MODE_MAX_TRIES = 3        # 最多送幾次

    # ════════════════════════════════════════════════════════════
    #  輸出 / 斷電保護
    # ════════════════════════════════════════════════════════════
    OUT_DIR = 'flight_logs'
    FSYNC_HZ = 4.0

    def __init__(self, **overrides):
        for k, v in overrides.items():
            if not hasattr(self, k):
                raise KeyError(f"XwingVirtualConfig 沒有參數 {k}")
            setattr(self, k, v)
        # 推頭距離由高度 / 俯衝角導出: 只覆寫了高度或角度時重新算
        if ({'CRUISE_AGL_M', 'DIVE_ANGLE_DEG'} & set(overrides)
                and 'PREDIVE_DIST_M' not in overrides):
            self.PREDIVE_DIST_M = self.predive_ground_range_m
            if 'STATIC_PREDIVE_DIST_M' not in overrides:
                self.STATIC_PREDIVE_DIST_M = self.PREDIVE_DIST_M

    # ------------------------------------------------------------------
    def intrinsics_at_zoom(self, z):
        """依變焦倍率 z 縮放 1x 內參 → (fx, fy, cx, cy)."""
        z = float(z)
        fx = self.FX_1X * z
        fy = self.FY_1X * z
        cx = (self.CX_1X - self.IMG_W / 2.0) * z + self.IMG_W / 2.0
        cy = (self.CY_1X - self.IMG_H / 2.0) * z + self.IMG_H / 2.0
        return fx, fy, cx, cy

    def virtual_intrinsics(self):
        """虛擬目標 / 控制座標系內參."""
        return self.intrinsics_at_zoom(self.VIRTUAL_ZOOM_LEVEL)

    @property
    def virtual_aim_v_offset_px(self):
        """攻角補償 (虛擬像素): 縱向 setpoint 下移 fy·tan(ALPHA_DIVE)."""
        fx, fy, cx, cy = self.virtual_intrinsics()
        return fy * math.tan(math.radians(self.ALPHA_DIVE_DEG))

    @property
    def predive_ground_range_m(self):
        """純幾何推頭距離 (不含推頭補償) = 巡航高度 / tan(俯衝角)."""
        return self.CRUISE_AGL_M / math.tan(math.radians(self.DIVE_ANGLE_DEG))

    @property
    def dive_ground_range_m(self):
        """DIVE 段水平距 = (巡航 − 拉平) / tan(俯衝角)."""
        return ((self.CRUISE_AGL_M - self.PULLOUT_AGL_M)
                / math.tan(math.radians(self.DIVE_ANGLE_DEG)))

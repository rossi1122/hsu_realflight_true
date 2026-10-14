# 實飛前飛控參數對齊清單 (xwing_virtual)

> 對照原版：`PREFLIGHT_PARAMS.md` (120 m / 20 m/s / 25° 版)。本版改成
> **240 m / 30 m/s / 30° 俯衝到 120 m / 速度上限 60 m/s / 無視覺**。

這份講的是**飛控 (ArduPlane) 裡要調的參數**，以及它們要跟程式
`xwing_virtual_config.py` 對齊到什麼值。

> ⚠ **兩套參數，別搞混**
> - **飛控參數**：在 Mission Planner 改，決定飛機本身的能力與限制。
> - **程式 `xwing_virtual_config.py`**：決定任務邏輯，但**受飛控參數限制**。
>
> 程式送的命令，飛控會用它自己的參數去夾。程式叫 -30° 俯衝，但飛控
> `PTCH_LIM_MIN_DEG` 只准 -15°，飛機就只壓 15°，任務失效還找不到原因。

> ⚠ **參數名稱與預設值會隨 ArduPlane 版本變動。** ArduPlane 4.4 之後
> `ARSPD_FBW_MIN/MAX` 改名為 `AIRSPEED_MIN/MAX`，`TRIM_ARSPD_CM` (cm/s) 改名為
> `AIRSPEED_CRUISE` (m/s)。程式啟動時兩種名稱都會查。

---

## 最優先：這幾個沒對，任務直接失敗

| 飛控參數 | 原版建議 | **xwing_virtual** | 為什麼 |
|---|---|---|---|
| `PTCH_LIM_MIN_DEG` | ≤ -30 (實際設 -40) | **-40** | 俯衝 30° + PID 修正空間。必須等於程式的 `PITCH_CMD_MIN_DEG=-40` |
| `AIRSPEED_MAX` | ≥ 30 | **≥ 60** | 等於 `PULLOUT_VMAX_MPS`。太低的話俯衝時飛控會抬頭壓速度，跟俯衝命令打架 |
| `AIRSPEED_CRUISE` | 20 | **30** | 等於 `V_CRUISE_MPS` / `SEARCH_SPEED_MPS`。不同的話撥 GUIDED 會加減速 |
| `AIRSPEED_MIN` | ≤ 15 | **≤ 30** (且 > 失速 12 留裕度，例如 18–20) | `CHANGE_SPEED 30` 超出範圍會被拒 (`result=2`) |

開機要看到這幾行 (xwing_virtual 新增的檢查)：
```
[Bridge] PTCH_LIM_MIN_DEG=-40° OK (需求 -40°)
[xwing] AIRSPEED_MIN=20 m/s OK
[xwing] AIRSPEED_MAX=60 m/s OK
[xwing] AIRSPEED_CRUISE→巡航 30 m/s OK
```

---

## 完整對齊表 (飛控 ↔ xwing_virtual_config.py)

### 俯仰角
| 飛控參數 | 說明 | 對齊 config | 原版 | **xwing_virtual** |
|---|---|---|---|---|
| `PTCH_LIM_MIN_DEG` | 最大俯衝角 | `PITCH_CMD_MIN_DEG` | -40 | **-40** |
| `PTCH_LIM_MAX_DEG` | 最大抬頭角 | 拉平 / 爬升夠用即可 | — | — (≥ 15) |

### 空速
| 飛控參數 | 說明 | 對齊 config | 原版 | **xwing_virtual** |
|---|---|---|---|---|
| `AIRSPEED_CRUISE` (舊 `TRIM_ARSPD_CM`) | 巡航空速 | `V_CRUISE_MPS` | 20 m/s (2000) | **30 m/s (3000)** |
| `AIRSPEED_MAX` (舊 `ARSPD_FBW_MAX`) | 空速上限 | ≥ `PULLOUT_VMAX_MPS` | 30 | **60** |
| `AIRSPEED_MIN` (舊 `ARSPD_FBW_MIN`) | 空速下限 | ≤ `SEARCH_SPEED_MPS`，> 失速 | ≤ 15 | **18–20 (≤ 30)** |

### 油門
| 飛控參數 | 說明 | 對齊 config | 原版 | **xwing_virtual** |
|---|---|---|---|---|
| `TRIM_THROTTLE` | 配平油門 (%) | ≈ `THR_TRIM`×100 | 45 | ⚠ **30 m/s 的配平值** (AUTO 巡航時看 log，填回 `THR_TRIM`) |
| `THR_MIN` | 最小油門 (%) | ≈ `THR_MIN`×100 | 5 | 5 |
| `THR_MAX` | 最大油門 (%) | ≈ `THR_MAX`×100 | 85 | 85 |

### 高度 / TECS
| 飛控參數 | 說明 | 對齊 config | 原版 | **xwing_virtual** |
|---|---|---|---|---|
| AUTO 航點高度 | 進場高度 | `CRUISE_AGL_M` | 120 m | **240 m** (用 `xwing_virtual_mission.waypoints`) |
| (TECS 目標由程式送) | 拉平後保持 | `TECS_HOLD_ALT_M` / `TECS_SPEED_MPS` | 70 m / 18 m/s | **120 m / 30 m/s** |
| `TECS_SPDWEIGHT`、`TECS_PITCH_MAX` 等 | 速度 / 高度權衡 | 沿用調好的值 | — | — |

### GUIDED / 航向
| 飛控參數 | 說明 | 建議 |
|---|---|---|
| `GUIDED_TIMEOUT` | forced attitude 逾時 | **0.5** |
| `GUIDED_P` | 存在才代表韌體有 43002 航向控制 | 必須存在 |

### 姿態控制器增益 (★ 先別動)
| 飛控參數 | 說明 |
|---|---|
| `PTCH2SRV_*`、`RLL2SRV_*` | 姿態迴路。若 xwing 是新機身，要先在 30 m/s 調穩 |
| `YAW2SRV_*` / `KFF_RDDRMIX` | 方向舵混控 |

> ⚠ **俯衝速度從 ~36 m/s 升到 ~47–55 m/s**，動壓是原本的 1.7–2.3 倍，舵面效率也跟著變高。
> 飛控如果有開空速縮放 (`SCALING_SPEED`)，會自動補償；程式端 DIVE 的方向舵也已經加回
> 空速排程 (`RUDDER_VSCHED`)。首飛請看 `rcout*` 和 `pid_dtheta_deg` 有沒有振盪。

---

## 安全 / 故障保護

| 飛控參數 | 說明 | 建議 |
|---|---|---|
| `FS_*` (failsafe) | RC 丟失 / GPS 丟失 / 電池低的行為 | 確認會帶到安全狀態 |
| `FENCE_*` | 地理圍欄 | **建議開**。跑道範圍約 400 m × 1800 m，圍欄要包住 |
| `FENCE_ALT_MAX` | 最高高度圍欄 | **> 240 m** (例如 280 m)，否則 AUTO 到 240 m 就觸發 |
| `FENCE_ALT_MIN` | 最低高度圍欄 | 可設在 **90 m** 左右當硬底 (拉平最低點約 100 m) |
| Q_* (VTOL) | 四旋翼切換 | 依你的 SOP |

安全鏈 (由上往下觸發)：

| 層 | 原版 | **xwing_virtual** |
|---|---|---|
| 拉平門檻 `PULLOUT_AGL_M` | 70 m | **120 m** |
| 拉平後實際最低點 (模擬) | — | **~100 m** |
| ABORT `ABORT_MIN_AGL_M` (PRE_DIVE/DIVE 段) | 55 m | **100 m** |
| 推頭最低高度 `ENTRY_MIN_AGL_M` | 60 m | **200 m** |
| 超速拉平 `PULLOUT_VMAX_MPS` | 30 m/s | **60 m/s** |

> 如果 120 m 是不能低於的底線，把 `PULLOUT_AGL_M` 改成 ~140 m。
> 如果 60 m/s 是結構極限，把 `PULLOUT_VMAX_MPS` 改成 55 (拉平過程速度還會再漲 2–3 m/s)。

---

## 開飛前對齊步驟

1. `PTCH_LIM_MIN_DEG = -40`。
2. `AIRSPEED_MAX ≥ 60`、`AIRSPEED_MIN ≤ 30`、`AIRSPEED_CRUISE = 30`。
3. 圍欄 `FENCE_ALT_MAX > 240`。
4. 載入 `xwing_virtual_mission.waypoints`，前後加上 VTOL 起降。
5. AUTO 30 m/s 巡航一圈，記下 `TRIM_THROTTLE`，填回 `THR_TRIM`。
6. 開機看第一節那四行 OK。

---

## xwing_virtual_config.py 目前值一覽

```
巡航高度 CRUISE_AGL_M          = 240 m           (原 120)
俯衝角   DIVE_ANGLE_DEG        = 30°             (原 25)
拉平     PULLOUT_AGL_M         = 120 m           (原 70)
速度上限 PULLOUT_VMAX_MPS      = 60 m/s          (原 30)
推頭距離 PREDIVE_DIST_M        = 240/tan30° = 415.69 m (移動/靜止同值, 原 257)
ABORT    ABORT_MIN_AGL_M       = 100 m           (原 55)
巡航速   V_CRUISE_MPS          = 30 m/s          (原 20)
SEARCH   SEARCH_SPEED_MPS      = 30 m/s          (原 16)
油門     THR_TRIM/MIN/MAX      = 0.45 / 0.05 / 0.85  (THR_TRIM 待 30 m/s 校)
TECS     HOLD_ALT / SPEED      = 120 m / 30 m/s  (原 70 m / 18 m/s)
攻角     NOM / DIVE            = 1.6° / 0°       (原 3.5° / 0°)
目標     起點 (23.7221111, 120.3798889) → 航向點 (23.720997, 120.379882), 10 m/s
```

# 實驗執行小抄 — xwing_virtual (純虛擬軌跡)

> 對照原版：`RUN_CASES.md`。差別如下：
> - 沒有相機、YOLO、SIYI 雲台、錄影 → 不需要 `--no-annot` / `--no-video`，也不用檢查錄影訊息
> - 對方雲台走 USB-UART，**預設開啟**：SEARCH 送 `'1'` 追蹤，進 PRE_DIVE 送 `'2'` 鎖定 −30°。
>   串口用 `--vision-port` 指定；沒接雲台加 `--no-vision` (見下方「雲台」一節)
> - 主程式改成 `xwing_virtual_flight.py`
> - 新場地、240 m / 30 m/s / 30° 俯衝到 120 m

4 個 case = DIVE 橫向 A/B 版本 × 靜止 / 移動目標 (與原版相同)。

> **版本 A** = DIVE 用虛擬目標 PID 修橫向 (方向舵)
> **版本 B** = DIVE 不修橫向，凍結進 DIVE 那刻的航向 (`--dive-hold-heading`)

---

## 出發前 (在家)

```bash
python3 xwing_virtual_sim.py            # 看各段距離、俯衝角、末速
python3 xwing_virtual_sim.py --static
python3 xwing_virtual_route.py          # 重新產生路線圖 / 航點 / KML
```
如果有改 `xwing_virtual_config.py` 的高度、角度、速度或座標，一定要重跑這兩支，
確認 DIVE 平均航跡角約 −30°，拉平最低高度你可以接受。

---

## 到場地 (每次做一次)

### 1. 程式座標

目標座標已經寫在 `xwing_virtual_config.py`：
```
MOVING_START_GPS   = (23.7217999, 120.3798795)   # 任務規劃 WP4 (2026/10/19)
MOVING_HEADING_GPS = (23.7206860, 120.3798795)   # 起點正南 124 m, 方位 180.0° (往南)
TARGET_GPS         = MOVING_START_GPS            # 靜止版
```

### 2. 航線

Mission Planner → 載入 `xwing_virtual_mission.waypoints` (240 m 跑道)，
**自己加上起飛和降落航點** (X-wing `Q_ENABLE = 0`，不是 VTOL，照平常方式)，並且確認：
- 跑道範圍內沒有障礙物或禁航區 (約 400 m × 1800 m，南北向)
- 回程邊在東側；若要放西側，執行 `python3 xwing_virtual_route.py --side west` 重新產生

### 3. 飛控參數

見 `xwing_virtual_PREFLIGHT_PARAMS.md`。最少要確認：

| 參數 | 值 |
|---|---|
| `PTCH_LIM_MIN_DEG` | **-40** |
| `AIRSPEED_MAX` | **≥ 60** |
| `AIRSPEED_MIN` | **≤ 30** |
| `AIRSPEED_CRUISE` | **30** |
| `GUIDED_TIMEOUT` | **0.5** (X-wing 韌體沒有這個參數 → 不用設) |
| GUIDED 切換 | **由地面站 Mission Planner 切** (`FLTMODE_GCSBLOCK = 0`，模式開關不用改) |
| `FENCE_ALT_MAX` | **> 240** (只在有開高度圍欄時有用；X-wing `FENCE_TYPE = 4` 只檢查多邊形) |

### 4. 遙控器

方向舵微調 (trim) 歸中 (同原版)。

### 5. 雲台 (對方 Orin NX)

- 起飛前先做完 `xwing_virtual_GIMBAL_GROUND_TEST.md` 的階段 C、E (至少看過一次雲台鎖到 −30°，且確認是相對水平面)
- 程式啟動要看到 `[xwing] 雲台視覺 UART: ... 模式鍵 ON`，狀態列尾巴 `VIS link=1`
- 下面 4 個 case 都加 `--vision-port /dev/serial/by-id/usb-XXXX` (或把 `VISION_UART_PORT` 改好)
- 當天沒接雲台：加 `--no-vision`

### 6. 實飛不要帶 `--ground-test` / `--bench` / `--bench-dive`

---

## 4 個 CASE 指令

```bash
roslaunch mavros apm.launch      # 另一個 terminal

# Case 1: 移動 + 版本 A
python3 xwing_virtual_flight.py --moving

# Case 2: 移動 + 版本 B
python3 xwing_virtual_flight.py --moving --dive-hold-heading

# Case 3: 靜止 + 版本 A
python3 xwing_virtual_flight.py --static

# Case 4: 靜止 + 版本 B
python3 xwing_virtual_flight.py --static --dive-hold-heading
```

臨時改推頭距離：`--predive-dist 420`。臨時改目標起點：`--start lat,lon` (移動) / `--target lat,lon` (靜止)。

---

## 飛行流程

1. 起飛 (照平常方式) → AUTO 爬到 240 m，進跑道。
2. 程式開著，畫面會每秒印 `[SEARCH  ] ... g=0` (還沒撥 GUIDED)。
3. 飛機在 **WP2 轉彎點**轉完彎、往南對正後，**飛到 WP3 (轉彎點後約 400 m、目標起點北方 613 m) 時由地面站切 GUIDED**。
   - 有真車時：飛機**通過 WP2** 時，車子從 **WP4 北方約 95 m** 起步，約 5 s 加到 36 km/h 後定速。
4. 之後全自動：

| 段 | 觸發 | 預計 (移動 / 靜止) |
|---|---|---|
| SEARCH | 撥 GUIDED | 9.9 s / 296 m ・ 6.6 s / 197 m |
| PRE_DIVE | 到目標水平距 ≤ 415.69 m (= 240/tan30°) | 1.4 s / 43 m |
| DIVE | pitch 到 −27° 內 | 5.4 s / 187 m，空速 32 → 46 m/s |
| PULLOUT | AGL ≤ 120 m (預測 0.5 s) 或空速 ≥ 60 m/s | 0.5 s |
| DONE | TECS 120 m / 30 m/s 平飛往南 | 最低約 100 m 後回 120 m |

5. 看到 `MODE: PULLOUT → DONE` 之後，**撥回 AUTO** (飛機會爬回 240 m 回跑道) 或照 SOP 處理。

> 如果撥 GUIDED 的位置比 WP3 晚 (離目標更近)，SEARCH 會變短；離目標起點不到 415.69 m
> 時，一撥下去就會直接推頭，沒有 SEARCH 段讓航向和 TECS 穩定。
> 建議 SEARCH 至少 8–10 s，也就是撥 GUIDED 時離目標起點至少約 575 m (移動) / 655 m (靜止)。

---

## 開機訊息檢查 (每趟都看)

```
[xwing] 移動目標: 起點 (23.7217999, 120.3798795) 方位 180.0° 速度 10.0 m/s ...
[xwing] 巡航 240 m / 30 m/s, 俯衝 30° → 120 m 拉平, 速度上限 60 m/s
[xwing] 推頭門檻: 到目標水平距 ≤ 416 m (純幾何 416 m)
[Bridge] PTCH_LIM_MIN_DEG=-40° OK (需求 -40°)
[Bridge] GUIDED_P=... → 43002 航向控制可用 (SEARCH 走 bank turn)
[Bridge] GUIDED_TIMEOUT=0.5s OK         ← X-wing 沒有這個參數，不會出現這行 (正常)
[Bridge] 方向舵迴路 ON — 限幅 ±30.0°, ...
[xwing] AIRSPEED_MIN=.. m/s OK
[xwing] AIRSPEED_MAX=.. m/s OK
[xwing] AIRSPEED_CRUISE→巡航 30 m/s OK
```

**看到下面任何一條就停下來**：
- `★★★ 找不到 GUIDED_P` → SEARCH 會退回 legacy bank
- `★★★ RUDDER_VERIFIED=False` → 方向舵命令全部歸零
- `★★★ 警告: PTCH_LIM_MIN_DEG=... 淺` → 俯衝角會被飛控夾掉
- `★★★ < 速度上限 60 m/s` → 俯衝時飛控會抬頭壓速度
- `★★★ > SEARCH 30 m/s` → CHANGE_SPEED 會被拒

---

## 飛完看什麼

log 在 `flight_logs/xwing_virtual_<時間>_<case>.csv` 與 `_events.txt`：

| 看什麼 | 欄位 | 判斷 |
|---|---|---|
| 俯衝角對不對 | `glide_slope_err_deg` (DIVE 段) | 平均 > 0 (比 30° 淺) → 減小 `PREDIVE_DIST_M`；< 0 (陡) → 加大。每 10 m 約 0.6° |
| 拉平最低高度 | `agl` (PULLOUT / DONE 段最小值) | 低於可接受值 → 提高 `PULLOUT_AGL_M` |
| 最高空速 | `V` | 接近 60 → 檢查 `AIRSPEED_MAX`，考慮把 `PULLOUT_VMAX_MPS` 降到 55 |
| 舵面是否飽和 / 振盪 | `rcout1..4`、`pid_dtheta_deg`、`rudder_sent_deg` | 振盪 → PID KP 減半 |
| 成績 | events.txt 的 `★★★ 成績: miss` | 移動目標前置瞄準關閉時預期 ~70 m (打在車後)，靜止 ~2 m |
| 地圖對照 | `lat/lon` (飛機)、`tgt_lat/tgt_lon` (虛擬目標) | 可直接匯入地圖 |

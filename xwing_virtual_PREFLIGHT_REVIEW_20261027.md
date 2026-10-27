# X-wing 實飛前總檢查 (2026/10/27，週四飛測用)

> 比對對象：原專案 `config.py` + `run_vision_flight.py` (HERO2180、相機 + YOLO + SIYI 雲台)
> 新專案：`xwing_virtual_config.py` + `xwing_virtual_flight.py` (X-wing、純虛擬軌跡、對方雲台只記錄)
> 程式版本：GitHub `claude/kind-clarke-r77kuq` 最新 commit

---

## 1. 改動的參數 (共同參數 55 個，改了 15 個)

### 1.1 任務幾何 / 速度 (照任務需求改)

| 參數 | 原專案 | 新專案 | 原因 |
|---|---|---|---|
| `CRUISE_AGL_M` | 120 | **240** | 巡航 / 進場高度 |
| `DIVE_ANGLE_DEG` | 25 | **30** | 俯衝角 |
| `PULLOUT_AGL_M` | 70 | **120** | 拉平高度 |
| `TECS_HOLD_ALT_M` | 70 | **120** | 拉平後保持高度 |
| `PULLOUT_VMAX_MPS` | 30 | **60** | 速度上限 (到 60 m/s 提前拉平) |
| `V_CRUISE_MPS` | 20 | **30** | 巡航空速 |
| `SEARCH_SPEED_MPS` | 16 | **30** | SEARCH 空速 (= AUTO 巡航) |
| `TECS_SPEED_MPS` | 18 | **30** | 拉平後空速 |
| `PREDIVE_DIST_M` / `STATIC_PREDIVE_DIST_M` | 257 | **415.69** | = 240 / tan30° |
| `ABORT_MIN_AGL_M` | 55 | **100** | 拉平高度上移，安全底線跟著上移 |
| `ALPHA_NOM_DEG` | 3.5 | **1.6** | 30 m/s 的巡航攻角 (只進 log) |

### 1.2 目標位置 (新場地)

| 參數 | 原專案 | 新專案 |
|---|---|---|
| `MOVING_START_GPS` | (22.913647, 120.312259) | **(23.7214954, 120.3799117)** |
| `MOVING_HEADING_GPS` | (22.9149299, 120.3121517)，355.5° | **(23.7203815, 120.3799117)，180.0° 往南** |
| `TARGET_GPS` (靜止) | (22.913677, 120.312260) | **= 起點** |

### 1.3 傳給狀態機 / bridge 時不同的設定

| 設定 | 原專案 | 新專案 | 原因 |
|---|---|---|---|
| `ENTRY_MIN_AGL_M` (低於此高度不准推頭) | 60 (寫死) | **200** | 240 m 進場，低於 200 m 代表幾何不對 |
| `GIMBAL_FPV_SETTLE_S` | 0.3 | **0** | 控制不用實體雲台，進 DIVE 當拍 PID 接手 |
| `BENCH_GEOMETRY_AGL_M` (只有 --bench) | 120 | **240** | 掛機幾何跟著巡航高度 |

### 1.4 完全沒改 (與原專案相同)

- **方向舵**：`RUDDER_VSCHED ((999,1.0),)`、`RUDDER_LIMIT_DEG 30`、`K_PSI 0.30`、`K_R 0.15`、`K_LAT_TO_RUDDER 0.50`、`RUDDER_SLEW_DPS 20`、`ROLL_PROT 25/35°` (10/22 依老師決定改回原值)
- **DIVE PID**：`PID_LAT 2.0/0.20/0.0`、`PID_LON 0.8/0.1/0.2`、濾波時間常數全部相同
- **油門**：`THR_TRIM 0.45`、`THR_MIN 0.05`、`THR_MAX 0.85`
- **其他**：`GUIDED_ENTRY_ROLL0_S 0.5`、`PREDIVE_HANDOFF_PITCH_TOL_DEG 3`、`PREDIVE_TIMEOUT_S 45`、虛擬相機內參、`MOVING_TARGET_LEAD False`、`TARGET_SPEED_MPS 10`
- **attitude_bridge.py / dive_guidance.py 控制邏輯**：沒有改

### 1.5 新增的參數

| 參數 | 值 | 作用 |
|---|---|---|
| `VISION_UART_PORT` | CP2102 by-id 路徑 | 對方雲台 UART (只記錄) |
| `GIMBAL_MODE_KEYS` | SEARCH `'1'`、PRE_DIVE/DIVE `'2'` | 依任務狀態送雲台模式鍵 |
| `REARM_OFF_S` | 1.0 s | 離開 GUIDED 超過 1 s → 這趟結束並重置 |
| `ARM_MIN_MARGIN_M` | 100 m | 切 GUIDED 時離起點至少 416 + 100 = 516 m |
| `ARM_MAX_DIST_M` | 1500 m | 離起點超過 1500 m 不啟動 |
| `ARM_MAX_BEARING_OFF_DEG` | 45° | 起點要在機頭前方 45° 內 |
| `ARM_GROUND_AGL_M` | 20 m | 在地面 (實驗室) 不檢查上面三項 |

### 1.6 拿掉的參數 (37 個)

全部是相機 / YOLO / SIYI 雲台 / 錄影 / 影像補償用的 (`CAM_*`、`YOLO_*`、`SIYI_*`、`GCOMP_*`、`GIMBAL_*` 舊雲台、`ZOOM_LEVEL` 等)。控制不再用真實影像，所以不需要。

---

## 2. 程式行為的改動

| 改動 | 說明 |
|---|---|
| 目標來源 | 不用相機 / YOLO，改用虛擬軌跡 (起點 + 方位 + 10 m/s)；DIVE 段 PID 吃虛擬目標投影，控制路徑與原專案相同 |
| 方向舵 | DIVE 一律用方向舵 PID (版本 A)，`--dive-hold-heading` 已移除 |
| 重複切 GUIDED | 每次切 GUIDED = 新的一趟 (目標回起點重新計時)；中途切出 1 s 後重置，不用重開程式 |
| 切 GUIDED 啟動檢查 (空中) | 高度 ≥ 200 m、離起點 516~1500 m、起點在機頭前方 45° 內；不符合 → 不啟動，只保持航向直飛 |
| 雲台 | 對方雲台 UART：只記錄資料 + 送模式鍵，**不進控制**；接不上不影響飛行 |
| 開機檢查 | 讀飛控 `AIRSPEED_MIN/MAX/CRUISE`、`PTCH_LIM_MIN_DEG`、`GUIDED_P`，不符合印 ★★★ |
| log | 多 `run` 欄位 (第幾趟)、雲台欄位；events 記錄 DIVE 方向舵設定、切 GUIDED 時離起點距離 |

---

## 3. 驗證狀態

| 項目 | 結果 |
|---|---|
| 程式編譯 (所有 xwing 檔案 + bridge + 狀態機) | ✓ |
| 重複切 GUIDED 端到端測試 `test_guided_rearm.py` | ✓ PASS |
| 任務模擬 (移動) | ✓ SEARCH 9.9 s → DIVE −28.8° → 最高 46.9 m/s → 拉平最低 105 m |
| 任務模擬 (靜止) | ✓ DIVE −31.1°，最高 48.2 m/s，拉平最低 97 m |
| X-wing 掛機 (10/17)：方向舵 Test A/B/E、bench F1 | ✓ |
| 雲台整合 (10/20)：兩台相機 | ✓ 鎖定 0.4~1.05 s |
| X-wing 地面測試 (10/22)：新起點、方向舵原參數、重複切 GUIDED | ✓ 程式錯誤 0 |
| 獨立程式審查 (重複 GUIDED / 啟動檢查) | ✓ 找到的問題已修 |

---

## 4. 飛之前一定要確認

1. **Orin NX 程式是最新版**：開機要看到
   - `DIVE 方向舵: 方向舵 PID (版本 A); 空速排程 RUDDER_VSCHED=((999.0, 1.0),)`
   - `目標=移動 起點 (23.7214954, 120.3799117) 方位 180.0°`
2. **Mission Planner 航點跟新起點對齊**：轉彎點 23.7305953、切 GUIDED 點 23.7270021、起點 23.7214954，經度都是 120.3799117 (沒移的話在舊 WP3 切 GUIDED 離起點約 647 m，仍可啟動)
3. **圍欄**：`FENCE_ENABLE = 1` (之前 A 機參數是 0)，多邊形包住航線、起點以南的俯衝 / 拉平段、繞圈範圍
4. **A 機參數** (如果飛 A 機)：油門 `SERVO4_MAX/TRIM` 是否故意改；若 A、B 是兩台不同飛機，羅盤 / 加速度計要各自重新校正
5. **遙控器訊號**：10/8 在約 100 m 低空遠處發生過 RC 失控保護 (約 6 s)，確認天線擺放
6. **電池電壓**：接電池後 Mission Planner 要讀到正常電壓 (BATT2 < 46 V 會觸發 RTL)
7. **開程式時飛控不能在 GUIDED**

## 5. 要注意 (不擋飛行，第一趟飛完看 log)

| 項目 | 說明 |
|---|---|
| 方向舵權限不隨空速縮小 | DIVE 末段約 45 m/s，同樣舵量效果比 30 m/s 大；看 DIVE 段 `rudder_sent_deg`、`roll_deg`、`r_dps` 有沒有左右擺 |
| 拉平最低約 105 m | 模擬值；ABORT 100 m 只在推頭 / 俯衝檢查，拉平時不會誤觸發。看 log 實際最低高度 |
| WP2 → WP3 400 m 餘裕小 | 10/8 實測轉彎後約 350 m (最遠 383 m) 才對正；400 m 只多 0.5~3 s，晚對正仍可啟動 (45° 內) |
| `THR_TRIM` 0.45 | 30 m/s 實際配平油門未量，從 .BIN 讀回再填 |
| 43002 航向控制 | 第一次在 X-wing 空中使用，看 SEARCH 航向是否穩定 |
| 雲台第 10 項、手機量角度、有目標追蹤 | 未驗證，只影響雲台資料 |
| 車子 | 停在起點北方約 95 m，飛機過轉彎點喊開，約 7 s 加到 36 km/h (10/12 實測) |

## 6. 實飛指令

```bash
roslaunch mavros apm.launch                                                   # Terminal 1
python3 xwing_virtual_flight.py --moving --case flight1 2>&1 | tee console_flight1.txt   # Terminal 2
```
沒接雲台加 `--no-vision`。流程：`xwing_virtual_prepare_flight.txt`；指令：`xwing_virtual_FLIGHT_DAY_QUICK.txt`。

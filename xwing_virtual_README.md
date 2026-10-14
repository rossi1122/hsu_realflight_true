# xwing_virtual — 純虛擬軌跡俯衝版 (無視覺)

這一版從原本的 vision_dive (`run_vision_flight.py`) 改出來，條件如下：

| 項目 | 原版 | **xwing_virtual** |
|---|---|---|
| 巡航高度 | 120 m | **240 m** |
| 巡航速度 | 20 m/s (SEARCH 16 m/s) | **30 m/s** |
| 俯衝角 | 25° | **30°** |
| 俯衝高度 | 120 → 70 m | **240 → 120 m** |
| 速度上限 (超速提前拉平) | 30 m/s | **60 m/s** |
| 感知 | 相機 + YOLO + 雲台 (只記錄) + 虛擬目標 (控制) | **只有虛擬目標** |
| 目標起點 | (22.913647, 120.312259) | **23°43'19.6"N 120°22'47.6"E = (23.7221111, 120.3798889)** |
| 目標航向點 | (22.9149299, 120.3121517)，方位 355.5° | **(23.720997, 120.379882)，方位 180.3° (往南)** |

> **控制邏輯沒有變。** 原版的飛機本來就只吃「虛擬目標」的投影誤差，真實相機和
> YOLO 只負責記錄。所以這一版拿掉整條視覺鏈，飛機的控制路徑還是同一條：
> `MAVROS → 虛擬目標投影 → 狀態機 / PID → AttitudeBridge → 飛控`。
> 改的只有參數、目標座標，以及把視覺硬體從主程式拿掉。

![路線圖](xwing_virtual_route.png)

---

## 1. 檔案變更總覽

### 1.1 新增的檔案 (全部以 `xwing_virtual` 命名)

| 新檔案 | 來源 / 對照的原檔 | 內容 |
|---|---|---|
| `xwing_virtual_config.py` | `config.py` | 新任務參數。只留控制需要的參數，視覺、相機、雲台、YOLO、錄影參數全部拿掉 |
| `xwing_virtual_flight.py` | `run_vision_flight.py` | 實飛主程式。拿掉視覺執行緒、相機、雲台、錄影和真實 PID 記錄 |
| `xwing_virtual_common.py` | (新) `run_vision_flight.py` 與 `test_logic_sim.py` 中重複的組裝段落 | 實飛和模擬共用：建虛擬目標、`GuidanceConfig`、bridge 參數覆寫 |
| `xwing_virtual_sim.py` | `test_logic_sim.py` | 任務剖面模擬。改用點質量模型 (含阻力、俯仰速率、過載)，算各段距離 |
| `xwing_virtual_route.py` | (新) | 產生路線圖、Mission Planner 航點、KML |
| `xwing_virtual_route.png` | (產出) | 路線圖：俯視、放大、高度剖面 |
| `xwing_virtual_mission.waypoints` | (產出) | Mission Planner AUTO 跑道 (240 m) |
| `xwing_virtual_route.kml` | (產出) | Google Earth 檢視 |
| `xwing_virtual_route_traj.csv` | (產出) | 模擬軌跡 (含 GPS) |
| `xwing_virtual_README.md` | `README.md` | 本文件 |
| `xwing_virtual_PREFLIGHT_PARAMS.md` | `PREFLIGHT_PARAMS.md` | 飛控參數對齊表 (新數值) |
| `xwing_virtual_RUN_CASES.md` | `RUN_CASES.md` | 到場地執行小抄 (新指令) |
| `xwing_virtual_prepare.txt` | `prepare.txt` | 開機啟動步驟 |
| `xwing_virtual_RUDDER_BENCH.md` | (新) | X-wing 方向舵掛機測試流程 |
| `xwing_virtual_GIMBAL_UART.md` | (新) | ★ 對方實驗室雲台視覺 UART 整合說明 (接線、協定、任務狀態回傳、待確認問題) |
| `xwing_uart_vision.py` | (新) | 背景執行緒接收雲台視覺 UART (v6/v7)、送模式鍵；目前只記錄，不進控制 |
| `xwing_uart_sender_sim.py` | (新) | 模擬對方 Orin NX 發送端，沒有硬體也能測 |
| `uart_mipi_receiver.py` | 對方提供 (未修改) | UART 協定定義與獨立接收測試程式 |

### 1.2 沿用、**完全沒改**的原模組

xwing_virtual 直接 import 這四個檔，只用參數覆寫，不改程式碼：

| 檔案 | 作用 |
|---|---|
| `dive_guidance.py` | 狀態機 SEARCH → PRE_DIVE → DIVE → PULLOUT → DONE，以及 DIVE 段 PID |
| `attitude_bridge.py` | 姿態命令 → MAVROS `SET_ATTITUDE_TARGET`、43002 航向、TECS 目標 |
| `virtual_source.py` | 虛擬目標 (靜止 / 移動) 與投影成虛擬畫面 |
| `virtual_target_geom.py` | GPS ↔ NED、落點外插、miss distance |

### 1.3 xwing_virtual **不再使用**的原檔案

這些檔案留在專案裡沒有刪，原版 `run_vision_flight.py` 仍然可以照常跑：

| 類別 | 檔案 |
|---|---|
| 相機 / YOLO / DKF | `vision.py`, `vision_driver_car_dive_with_dkf_v3.py`, `dkf_tracker_gimbal.py`, `ray_projection_distance.py`, `cameras.py` |
| 雲台 | `gimbal.py`, `siyi_sdk.py`, `siyi_message.py`, `crc16_python.py`, `utils.py` |
| 真實影像 PID 記錄 | `realvision_pid_logger.py`, `gimbal_pid_compensation.py`, `attitude_time_history.py` |
| 錄影 | `video_writer.py` |
| 視覺後處理 | `postprocess_compare.py`, `car_track.py` |
| 視覺 / 雲台測試 | `test_camera_bench.py`, `test_gimbal.py`, `test_dive_ground.py` |
| 其他 | `check_version.py` (檢查原版檔案指紋) |

> `test_rudder_bench.py` 只測方向舵映射、與視覺無關，xwing 機若換了機身仍可拿來掛機驗證。

---

## 2. 參數對照 (`config.py` → `xwing_virtual_config.py`)

### 2.1 有改的參數

| 參數 | 原值 | 新值 | 原因 |
|---|---|---|---|
| `CRUISE_AGL_M` | 120 | **240** | 需求 |
| `DIVE_ANGLE_DEG` | 25 | **30** | 需求 |
| `PULLOUT_AGL_M` | 70 | **120** | 需求：俯衝 240 → 120 m |
| `TECS_HOLD_ALT_M` | 70 | **120** | 拉平後保持在拉平高度 |
| `V_CRUISE_MPS` | 20 | **30** | 需求 (也就是 bridge 的 `V_REF_MPS`) |
| `SEARCH_SPEED_MPS` | 16 | **30** | 必須等於 AUTO 巡航，撥 GUIDED 才不會加減速 |
| `TECS_SPEED_MPS` | 18 | **30** | 拉平後回巡航速度 |
| `PULLOUT_VMAX_MPS` | 30 | **60** | 需求：速度上限。舊值 30 會讓飛機一進 DIVE 就判定超速 |
| `PREDIVE_DIST_M` (移動) | 257 | **415.69** | = 240 / tan30°，由 `CRUISE_AGL_M` / `DIVE_ANGLE_DEG` 直接算出，見 4.3 |
| `STATIC_PREDIVE_DIST_M` (靜止) | 257 | **415.69** | 與移動版相同 |
| `ABORT_MIN_AGL_M` | 55 | **100** | 拉平高度上移，安全底線跟著上移 |
| `ENTRY_MIN_AGL_M` | 60 (寫死在主程式) | **200** (移到 config) | 240 m 進場時，若低於 200 m 代表幾何已經不對，不准推頭 |
| `ALPHA_NOM_DEG` | 3.5 | **1.6** | 升力 ∝ V²，3.5 × (20/30)² ≈ 1.6。SEARCH 的 pitch 交給 TECS，這個值只會進 log |
| `RUDDER_VSCHED` | `((999, 1.0),)` 不排程 | **`((30,1.0),(40,0.56),(50,0.36),(999,0.25))`** | 俯衝峰值從 ~36 m/s 升到 ~47–55 m/s，動壓變成 2.8–4 倍，方向舵權限按 (30/V)² 縮 |
| `MOVING_START_GPS` | (22.913647, 120.312259) | **(23.7221111, 120.3798889)** | 新場地 |
| `MOVING_HEADING_GPS` | (22.9149299, 120.3121517) | **(23.720997, 120.379882)** | 新場地 |
| `TARGET_GPS` (靜止) | (22.913677, 120.312260) | **= 目標起點** | 新場地 |
| `GIMBAL_FPV_SETTLE_S` | 0.3 | **0** | 沒有實體雲台要切 FPV，DIVE 當拍就開始 PID |

### 2.2 新增的參數

| 參數 | 值 | 說明 |
|---|---|---|
| `MOVING_STOP_AT_HEADING_GPS` | `False` | 起點到航向點只有 **124 m**，10 m/s 只要 12.4 s 就走完。`False` 表示虛擬目標過了航向點繼續沿同方位前進 (與原版行為相同)；`True` 表示停在航向點 |
| `PREDIVE_TIMEOUT_S` | 45 | 原本是 `dive_guidance` 預設值，移到 config |

### 2.3 沿用不變的參數

PID 增益 (`PID_LAT_*`、`PID_LON_*`)、方向舵限幅 `RUDDER_LIMIT_DEG=30`、`K_LAT_TO_RUDDER`、
roll 保護、虛擬相機內參 (1x)、`TARGET_SPEED_MPS=10`、`MOVING_TARGET_LEAD=False`、
`THR_TRIM/MIN/MAX`、`PREDIVE_HANDOFF_PITCH_TOL_DEG=3`。

> ⚠ **`THR_TRIM=0.45` 是 20 m/s 的配平油門**，30 m/s 需要更多油門。請填飛控在 AUTO 30 m/s
> 巡航時的 `TRIM_THROTTLE`/100。這個值只影響 PRE_DIVE 推頭那約 1.4 秒的油門。

### 2.4 刪掉的參數 (視覺專用)

`ZOOM_LEVEL`、`FX/FY/CX/CY` 以外的相機設定、`YOLO_*`、`CAPTURE_LATENCY_MS`、`CAM_*`、
`GCOMP_*`、`REALVISION_PID_*`、`SIYI_*`、`GIMBAL_*`、`GATE_*`、`ACQUIRE_*`、
`TRACK_GATE_*`、`VIDEO_FPS`。

---

## 3. 主程式對照 (`run_vision_flight.py` → `xwing_virtual_flight.py`)

| 項目 | 原版 | xwing_virtual |
|---|---|---|
| 執行緒 | 視覺 (~30 Hz) + 控制 (50 Hz) + 取像 + 錄影 ×2 | **只有控制 (50 Hz)** (加上 bridge 自己的發送執行緒) |
| 硬體需求 | ROS + MAVROS + 相機 + SIYI 雲台 + GPU (YOLO) | **ROS + MAVROS + numpy** |
| 啟動檢查 | 雲台連線失敗或取像失敗 → FATAL 結束 | 拿掉；**新增飛控空速參數檢查** (`AIRSPEED_MIN/MAX/CRUISE`，新舊參數名都查) |
| 控制輸入 | `virt.get_fields()` → `sm.update()` | **相同** |
| 真實 PID 記錄 | `GimbalCompPIDLogger` 每拍算兩套真實 PID | 拿掉 |
| 掛機合成 bbox (`--inject-bbox`) | 有 | 拿掉 (虛擬目標本身就是合成的) |
| 輸出 | 主 CSV + 視覺 CSV + 標註影片 + raw 影片 + events | **主 CSV + events** |
| 主 CSV 欄位 | 160 欄 (含 real_*、gcomp_*、vis_*)，另有視覺 CSV 54 欄 | 80 欄。拿掉 `frame_idx`、`vis_*`、`img_to_cmd_ms`、`real_*`、`gcomp_*`；**新增 `tgt_lat`、`tgt_lon`** (虛擬目標當下 GPS，方便在地圖上畫) |
| 檔名 | `flight_YYYYmmdd_HHMMSS_<tag>.csv` | `xwing_virtual_YYYYmmdd_HHMMSS_<tag>.csv` |
| 命令列 | `--static/--moving/--start/--speed/--target/--predive-dist/--bench/--bench-pitch/--bench-dive/--ground-test/--dive-hold-heading/--case/--tgt-heading/--no-video/--no-annot/--inject-*` | 同左，但拿掉 `--no-video`、`--no-annot`、`--inject-*`、`--predive` |
| `BENCH_GEOMETRY_AGL_M` | 寫死 120 | `= CRUISE_AGL_M` (240) |

---

## 4. 任務各段距離 (★ 你要的計算)

### 4.1 純幾何 (理想、瞬間推頭)

| 量 | 算式 | 值 |
|---|---|---|
| 推頭點到目標的水平距 | 240 / tan30° | **415.7 m** |
| DIVE 水平距 | (240 − 120) / tan30° | **207.8 m** |
| DIVE 斜距 (航跡長) | (240 − 120) / sin30° | **240.0 m** |
| 拉平時到目標的水平距 | 120 / tan30° | **207.8 m** |
| DIVE 時間 (若維持 30 m/s) | 240 / 30 | 8.0 s |

### 4.2 模擬結果 (實際會飛的樣子)

用 `xwing_virtual_sim.py` 跑，狀態機與實飛同一套。模型設定：L/D = 8、pitch 速率 20°/s、
過載 2.5 g；撥 GUIDED 時飛機在目標起點後方 700 m。

**移動目標** (10 m/s 往南，飛機從北邊追擊)：

| 段 | 時長 | **水平距離** | 航跡長 | 高度 | 空速 | 到目標水平距 |
|---|---|---|---|---|---|---|
| **SEARCH** | 14.2 s | **426 m** | 426 m | 240 → 240 m | 30 → 30 m/s | 700 → 416 m |
| **PRE_DIVE** | 1.4 s | **43 m** | 44 m | 240 → 235 m | 30 → 31.6 m/s | 416 → 387 m |
| **DIVE** | 5.4 s | **187 m** | 213 m | 234 → 131 m | 31.6 → 45.8 m/s | 386 → 254 m |
| PULLOUT (交棒 0.5 s) | 0.5 s | 21 m | 23 m | 131 → 120 m | 45.9 → 46.6 m/s | 254 → 238 m |
| 拉平改出 (到航跡角 ≥ −2°) | 1.9 s | 85 m | — | 最低 **105 m** | ~47 m/s | — |
| **合計 (撥 GUIDED → 改平)** | ~23 s | **~760 m** | | | | |

**靜止目標** (固定在起點)：

| 段 | 時長 | **水平距離** | 航跡長 | 高度 | 空速 | 到目標水平距 |
|---|---|---|---|---|---|---|
| **SEARCH** | 9.5 s | **284 m** | 284 m | 240 → 240 m | 30 → 30 m/s | 700 → 416 m |
| **PRE_DIVE** | 1.4 s | **43 m** | 44 m | 240 → 235 m | 30 → 31.6 m/s | 416 → 372 m |
| **DIVE** | 5.0 s | **166 m** | 195 m | 234 → 133 m | 31.6 → 46.4 m/s | 372 → 206 m |
| PULLOUT | 0.5 s | 20 m | 24 m | 132 → 120 m | 46.4 → 47.5 m/s | 205 → 185 m |
| 拉平改出 | 2.2 s | 97 m | — | 最低 **97 m** | ~48 m/s | — |

**SEARCH 的長度由「在哪裡撥 GUIDED」決定**：

| 撥 GUIDED 時距目標起點 | 移動 SEARCH | 靜止 SEARCH |
|---|---|---|
| 600 m | 276 m / 9.2 s | 184 m / 6.1 s |
| **700 m (建議)** | **426 m / 14.2 s** | **284 m / 9.5 s** |
| 800 m | 576 m / 19.2 s | 384 m / 12.8 s |
| 1000 m | 876 m / 29.2 s | 584 m / 19.5 s |

算式：移動版 SEARCH 水平距 = 30 × (D_guided − 415.69) / (30 − 10)，靜止版 = D_guided − 415.69。
建議 SEARCH 至少 8–10 s，讓 43002 航向和 TECS 先穩定。

### 4.3 推頭距離 = 240 / tan30° = 415.69 m

`PREDIVE_DIST_M` 與 `STATIC_PREDIVE_DIST_M` 都用純幾何值，在 `xwing_virtual_config.py` 裡直接寫成
`CRUISE_AGL_M / tan(DIVE_ANGLE_DEG)`。改巡航高度或俯衝角，推頭距離會自動跟著變。

實際平均俯衝角會跟 30° 差約 1°，模擬結果如下：

| 目標 | DIVE 平均航跡角 | 原因 |
|---|---|---|
| 移動 | **≈ −28.8°** (略淺) | 從後方追車，車往前跑，視線越來越淺，PID 把俯衝跟著拉淺 |
| 靜止 | **≈ −31.1°** (略陡) | 推頭需要約 1.4 s，期間水平多飛 43 m、只掉 5 m |

`python3 xwing_virtual_sim.py --sweep` 可以掃 380–480 m 看每個門檻的平均俯衝角
(每 10 m 約差 0.6°)。實飛後看 log 的 `glide_slope_err_deg`，若想修正，可用
`--predive-dist` 臨時覆寫：平均比 −30° 陡就加大，淺就減小。

### 4.4 速度與高度 (⚠ 一定要看)

| 情境 | DIVE 末速 | 最高空速 | 拉平後最低高度 |
|---|---|---|---|
| 有阻力 (L/D = 8) | 46 m/s | 47–48 m/s | **~97–105 m** |
| 無阻力 (上界) | ~55 m/s | 61–63 m/s | **~86–97 m** |

1. **拉平門檻是 120 m，但飛機會往下沉到約 100 m 才改平。** 從 ~47 m/s、−30° 拉回平飛需要
   約 2 s、85–97 m 水平距離。如果 **120 m 是不能低於的底線**，請把 `PULLOUT_AGL_M` 改成
   **~140 m** (`TECS_HOLD_ALT_M` 可以維持 120)。
2. 正常剖面不會碰到 60 m/s。60 m/s 的超速拉平是安全網。如果 **60 m/s 是機體結構極限**，
   拉平過程速度還會再漲 2–3 m/s，建議把 `PULLOUT_VMAX_MPS` 降到 **55**。
3. 這些數字來自點質量估算模型。L/D、pitch 速率、過載都是假設值，請以實飛 log 為準。

### 4.5 移動目標的 miss

前置瞄準 (`MOVING_TARGET_LEAD`) 維持關閉 (跟原版一樣)。飛機一直瞄車的「當下位置」，
拉平瞬間沿速度外插的落點會落在車後方 **約 70 m** (靜止目標約 2 m)。
模擬打開前置瞄準時，miss 約 6 m，但俯衝會變淺到約 −23°。原版關掉前置瞄準是因為真實相機
視野會丟車，純虛擬沒有這個限制；要打開的話，請同時調小 `PREDIVE_DIST_M`，並先用模擬確認。

---

## 5. 飛行路線規劃

![路線圖](xwing_virtual_route.png)

### 5.1 進場方式

目標從起點**往南** (180.3°) 走，飛機**從北邊往南**進場 (同向追擊，跟原版「從後方追車」一致)，
沿「起點 → 航向點」這條線的延長線飛。

### 5.2 AUTO 跑道 (`xwing_virtual_mission.waypoints`，全部相對高度 240 m)

| 航點 | 位置 | 緯度, 經度 |
|---|---|---|
| WP1 進場邊起點 | 目標起點北方 1300 m | 23.7337890, 120.3799612 |
| **WP2 撥 GUIDED 點** | 目標起點北方 700 m | **23.7283992, 120.3799278** |
| WP3 進場邊終點 | 目標起點南方 500 m | 23.7176196, 120.3798611 |
| WP4 回程轉彎 | WP3 往東 400 m | 23.7175992, 120.3837859 |
| WP5 回程邊終點 | WP1 往東 400 m | 23.7337686, 120.3838860 |
| DO_JUMP → WP1 | 無限繞 | |

- WP1 → WP2 有 600 m 直線 (20 s)，讓飛機轉完彎後先對正。
- **飛過 WP2 時撥 GUIDED。** 沒撥的話，AUTO 會沿進場邊繼續飛到 WP3，再從東側繞回來重來。
- 回程邊放東側；如果場地西側比較空，執行 `python3 xwing_virtual_route.py --side west`。
  側移 400 m 大約是 30 m/s、坡度 30° 轉彎半徑 (~160 m) 的 2.5 倍。
- ★ **VTOL 起飛、降落航點請自己加在前後。** 跑道範圍約 400 m × 1800 m，請先確認場地、
  空域和障礙物。

### 5.3 任務關鍵點 (移動目標，模擬)

| 事件 | 飛機位置 | AGL | 車的位置 |
|---|---|---|---|
| 撥 GUIDED (任務開始) | 23.7283992, 120.3799278 (目標起點北 700 m) | 240 m | 起點 |
| PRE_DIVE 推頭 | 23.7245671, 120.3799041 (目標起點北 273 m) | 240 m | 起點南 142 m (已過航向點) |
| DIVE 開始 | 23.7241729, 120.3799017 | 234 m | 起點南 157 m |
| PULLOUT 拉平 | 23.7224894, 120.3798913 (目標起點北 42 m) | 131 m | 起點南 211 m |
| 改平 (航跡角 ≥ −2°) | 目標起點南約 43 m | ~105 m | 起點南 ~230 m |

拉平之後，飛機以 TECS 120 m / 30 m/s 往南直飛。之後撥回 AUTO，飛機會回到跑道 (爬回 240 m)。

> **車的位置注意**：撥 GUIDED 後 12.4 s 車就走到航向點，PRE_DIVE 時已經過了航向點 (預設
> `MOVING_STOP_AT_HEADING_GPS=False`，沿同方位繼續走)。若要車停在航向點，改成 `True`，
> 再跑一次 `xwing_virtual_sim.py --sweep` 重新校推頭距離 (車停下來等於變成靜止目標，
> 平均俯衝角會接近靜止目標的 ≈ −31°)。

---

## 6. 使用方式

```bash
# 1. 在家: 模擬 + 產生路線圖與航點 (不需要 ROS)
python3 xwing_virtual_sim.py                  # 移動目標各段距離
python3 xwing_virtual_sim.py --static
python3 xwing_virtual_sim.py --sweep          # 掃推頭距離
python3 xwing_virtual_route.py                # 路線圖 + .waypoints + .kml

# 2. 機上 (ROS + MAVROS)
roslaunch mavros apm.launch
python3 xwing_virtual_flight.py               # 移動目標
python3 xwing_virtual_flight.py --static      # 靜止目標
```

場地執行步驟見 `xwing_virtual_RUN_CASES.md`，飛控參數見 `xwing_virtual_PREFLIGHT_PARAMS.md`。

---

## 7. 驗證狀況

| 項目 | 結果 |
|---|---|
| `xwing_virtual_sim.py` 移動 / 靜止 | 完整跑完 SEARCH → PRE_DIVE → DIVE → PULLOUT → DONE |
| `xwing_virtual_flight.py` | 用假的 rospy/MAVROS (無 ROS 環境) 把模擬軌跡回放進主迴圈：移動、靜止都正確走完 5 個模式，`errors=0`，CSV 正常寫出，miss 與模擬一致 (68.0 m / 1.7 m) |
| 原版檔案 | 沒有修改 |
| **尚未做** | 沒有在真的 ROS / MAVROS / 飛控 / 飛機上跑過。PID 增益在 ~50 m/s 俯衝下沒有實測。L/D、俯仰速率等模型參數是假設值 |

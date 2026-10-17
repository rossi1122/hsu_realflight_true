# X-wing 完整測試流程 (xwing_virtual + 對方雲台)

> 從出發前準備、地面測試，到實際飛行、飛行後整理，照順序一步一步做。
> 細節與判讀另見：
> - 雲台：`xwing_virtual_GIMBAL_GROUND_TEST.md`
> - 方向舵：`xwing_virtual_RUDDER_BENCH.md`
> - 指令小抄：`xwing_virtual_prepare_ground.txt`、`xwing_virtual_prepare_flight.txt`
>
> 本文件只用**移動目標**。

---

## 0. 總覽

```
第 0 步  出發前準備 (實驗室)      程式、飛控參數、串口、跟對方協調
   │
第 1 步  雲台地面測試             雲台會不會鎖到 −30°、鎖定是不是相對水平面
   │     (要對方 Orin NX + 雲台)
   │
第 2 步  飛機地面測試 (拆槳)      舵面方向、模式開關、方向舵 Test A/B/E、整機掛機
   │     (室外 GPS, 飛控 + MAVROS)
   │
第 3 步  飛行日                   到場設定 → 開機 → 起飛 → 撥 GUIDED → 俯衝 → 回 AUTO → 降落
   │
第 4 步  飛行後                   收 log、看結果、雲台復原
```

### 0.1 人員

| 角色 | 工作 |
|---|---|
| **飛手** | 遙控器。起飛降落；GUIDED 中不碰搖桿；出狀況時把模式開關撥到 FBWA 離開 GUIDED |
| **機載操作員** | 連到我們的 Orin NX 終端機 (你平常的連線方式)，執行程式、看訊息、必要時輸入 `x` 中止 |
| **地面站** | Mission Planner：**切 GUIDED / 切回 AUTO**，看模式、高度、空速、圍欄 |
| **對方人員** | 對方 Orin NX + 雲台開機、確認他們的程式在跑 |
| (選) **開車的人** | 真的有車當目標時：撥 GUIDED 那一刻從起點出發 |

### 0.2 能不能飛：判斷表

| 項目 | 不通過時 |
|---|---|
| 飛控參數 (`PTCH_LIM_MIN_DEG`、`AIRSPEED_MAX`)、地面站切得到 GUIDED | **不飛** |
| 方向舵 Test A (yaw 欄位 = 方向舵、送 180 = 歸中) | **不飛** (PRE_DIVE 靠它把方向舵歸中，版本 A、B 都會用到) |
| 整機掛機 F1：PRE_DIVE 時兩片 V 尾有差動 | **不飛** |
| 方向舵 Test B 正負號不對、或 Test E 剩 < 50% | 可以飛，但只能用**版本 B** (`--dive-hold-heading`) |
| 方向舵還沒測 B / E | 可以飛，只能用**版本 B** |
| 雲台測試 (C / D / E) 不通過 | **可以飛**：控制只用虛擬目標，雲台資料只記錄。只是這趟的雲台資料不能用 |
| 開機時 `VIS link=0` | **可以飛**：不影響控制，只是收不到雲台資料、鎖定鍵送不出去 |

---

## 第 0 步：出發前準備 (實驗室)

### 0-1. 程式更新 (我們的 Orin NX)

```bash
cd ~/hsu_realflight_true                     # 換成你放專案的路徑
git fetch origin claude/kind-clarke-r77kuq
git checkout claude/kind-clarke-r77kuq
git pull
pip3 install pyserial
python3 xwing_virtual_sim.py                 # 不需要 ROS, 看各段距離
```

`xwing_virtual_sim.py` 要看到：推頭距離 415.7 m、DIVE 平均航跡角約 −29°、拉平最低高度約 100 m。

### 0-2. 改飛控參數 (Mission Planner) ★

連上 X-wing → **Config → Full Parameter List** → 改值 → **Write Params** → 飛控重開機 → **Refresh Params** 再看一次確認。

| 參數 | 目前 | 改成 | 原因 |
|---|---|---|---|
| `PTCH_LIM_MIN_DEG` | −35 | **−40** | 俯衝 −30° 需要餘裕，−35 會被飛控夾住 |
| `AIRSPEED_MAX` | 50 | **60** | 程式速度上限 60 m/s。⚠ 先確認機體能承受 60 m/s |
| MAVROS 那個序列埠的鮑率 | SERIAL1 = 115、SERIAL2 = 57 | 若 MAVROS 走 SERIAL2 → 改 **921** | 57600 傳不完 50 Hz 的姿態資料 |

**GUIDED 由地面站切換**：Mission Planner → Flight Data → Actions → 模式選 GUIDED → **Set Mode**。
`FLTMODE_GCSBLOCK = 0` 已允許地面站切模式，模式開關不用改。
⚠ 中止時飛手要把模式開關**撥到跟目前不同的位置** (例如 FBWA) 才會離開 GUIDED——ArduPlane 只在開關位置「改變」時才切模式。

**不要改的**：`AIRSPEED_MIN = 25`、`AIRSPEED_CRUISE = 30`、`GUIDED_P = 5000`、`TRIM_THROTTLE = 45` (程式的 `THR_TRIM = 0.45` 跟它一致)。

**順便記下來**：
- 韌體版本：Mission Planner 連線時 **Messages** 分頁的 `ArduPlane V4.x.x`：__________
- X-wing 的參數表沒有 `GUIDED_TIMEOUT`，所以程式開機**不會**印 `[Bridge] GUIDED_TIMEOUT=...` 那行，這是正常的。
  影響：撥 GUIDED 後 roll=0 會維持約 3 秒 (韌體內建逾時)，SEARCH 開始轉向會晚一點。

**MAVROS 接哪個埠**：確認 Orin NX 接的是飛控的哪個孔 (USB / TELEM1 = SERIAL1 / TELEM2 = SERIAL2)，
`apm.launch` 裡的 `fcu_url` 鮑率要跟飛控一致。例：
```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyTHS0:115200     # 例: 接 TELEM1
```

### 0-3. 雲台串口固定名稱

1. 拔掉雲台的 USB-UART → `ls -l /dev/serial/by-id/` → 記下現有的
2. 插上 → 再執行一次 → 多出來的那行就是雲台
3. 填進 `xwing_virtual_config.py`：
   ```python
   VISION_UART_PORT = '/dev/serial/by-id/usb-XXXX'
   ```

### 0-4. 跟對方確認

- [ ] 他們的發送程式會不會開機自動執行？要手動的話，誰負責按
- [ ] 模式鍵 `'0'` 手動、`'1'` 追蹤 (目標置中)、`'2'` 鎖定 (pitch −30°) — 跟我們的理解一致
- [ ] 他們自己也會存 log 嗎？飛行後要一份
- [ ] (有真車時) 車子在哪裡出發、無線電怎麼喊

---

## 第 1 步：雲台地面測試 (要對方 Orin NX + 雲台)

接線：對方 TX → 我們 RX、對方 RX → 我們 TX、GND 共接、VCC 不接。

> ⚠ 同一個串口一次只能給一支程式用，換下一個測試前先把上一支關掉 (`q` 或 Ctrl+C)。

### 1-1. 連線確認

```bash
python3 uart_mipi_receiver.py --port /dev/serial/by-id/usb-XXXX
```
- [ ] 看到 `CRC=OK`，模式和 pitch 有數值
- [ ] 按 `2` → 對方雲台往下轉；按 `1` → 回追蹤
- [ ] Ctrl+C 關掉

### 1-2. 看雲台鎖到 −30° (階段 C)

雲台 (或飛機) 放平。

```bash
python3 xwing_gimbal_mode_test.py
```

| 按鍵 | 要看到 |
|---|---|
| `0` | 請對方把雲台轉到接近水平。**往下轉時回報的 pitch 要變負的** |
| `s` | `✓ 對方回報 mode=1` |
| `p` | `✓ 對方回報 mode=2`、`✓ 鎖定到位 pitch=-29.x°`、狀態列 `[LOCKED]`；**雲台實際往下轉並停住** |
| (量) | 手機水平儀貼相機側面：光軸往下 30° ± 2° |
| (等 10 s) | `[LOCKED]` 不消失 |
| `d` | `維持 '2' (與上一狀態同鍵, 不重送)`，雲台不動 |
| `q` | 總結：`★ 全部通過` |

### 1-3. 自動重複 5 輪 (階段 D)

```bash
python3 xwing_gimbal_mode_test.py --auto --repeat 5
```
- [ ] `★ 全部通過`，5 次到位都 ≤ 1.5 s，`crc` = 0

### 1-4. 鎖定是相對水平面還是相對機身 (階段 E) ★

雲台裝在飛機上 (實飛的裝法)，飛機放平。

```bash
python3 xwing_gimbal_mode_test.py
```
1. 按 `p` → 等 `[LOCKED]`
2. **抬高機尾**，機頭朝下約 15°，停 5 s。看相機會不會跟著機身轉
3. 放平，再**抬高機頭**約 10°，停 5 s
4. 按 `q`

| 看到的 | 結論 |
|---|---|
| 相機一直保持水平面下 30°，回報 pitch 一直 −30 | ✓ 相對水平面，DIVE 時目標會在畫面中 |
| 相機跟著機身轉 | ✗ 相對機身，DIVE 時目標會出畫面 → 跟對方討論。**仍可飛** (視覺只記錄) |

---

## 第 2 步：飛機地面測試 (拆槳、室外有 GPS)

### 2-1. 安全

- [ ] **拆掉螺旋槳**
- [ ] 飛機固定在架上，或一人壓住
- [ ] 飛控 **disarmed** (不用解鎖，舵面照樣會動)
- [ ] 遙控器開機，油門最低，方向舵 trim 歸中
- [ ] 室外，等 GPS 3D fix

### 2-2. 開機順序與 MAVROS 連線

開機順序：遙控器 → 對方 Orin NX + 雲台 → 我們的 Orin NX → 飛控。

```bash
# Terminal 1
roslaunch mavros apm.launch            # (fcu_url 依 0-2 節)

# Terminal 2
rostopic echo -n1 /mavros/state        # 要看到 connected: True
```

### 2-3. MANUAL 舵面方向 (遙控器)

飛控切 **MANUAL**，**站在機尾後面往前看**：

| 搖桿 | 應該看到 |
|---|---|
| 副翼往右 | 右副翼後緣往上、左副翼後緣往下 |
| 拉桿 (抬頭) | 兩片 V 尾後緣**一起往上** |
| 方向舵往右 | 右片 V 尾後緣往**下**、左片後緣往**上** |

- [ ] 三個都對 (不對就先修舵機正反向 / 連桿，不要往下做)

### 2-4. 模式開關

依序撥每個位置，看 Mission Planner HUD 的模式：

- [ ] **Mission Planner 切得到 Guided** (Actions → GUIDED → Set Mode)
- [ ] 在 Guided 時，飛手把開關撥到 FBWA → 立刻離開 Guided (中止動作)
- [ ] 撥得到 AUTO、FBWA、MANUAL
- [ ] 最後撥回 **MANUAL**

### 2-5. 找舵面通道

```bash
python3 test_rudder_bench.py --list-channels      # 輸入 yes; 10 秒內動搖桿
```

只打**右舵**，看 ch2、ch3 哪個 PWM 變大：

| | 通道 |
|---|---|
| 打右舵時 PWM **變大**的 V 尾通道 → `R` | ______ (照公式應該是 3) |
| 另一片 V 尾 → `E` | ______ (應該是 2) |
| 副翼 → `A` | ______ (應該是 1) |

### 2-6. 方向舵 Test A、B、E

每個測試開始時程式會問 `yes`，提示「不是 GUIDED」時用 2-4 節的開關切到 **GUIDED**，按 Enter。

```bash
python3 test_rudder_bench.py --test A --rudder-ch 3 --aileron-ch 1 --elevator-ch 2
python3 test_rudder_bench.py --test B --rudder-ch 3 --aileron-ch 1 --elevator-ch 2
python3 test_rudder_bench.py --test E --rudder-ch 3 --elevator-ch 2
```
(3 / 2 / 1 換成 2-5 節實際查到的)

| 測試 | 通過 | 結果 |
|---|---|---|
| **A** | `→ 假設【成立】` | □ 成立　□ 不動　□ 形狀不一致 → **不飛** |
| **B** | 正舵量時機頭往**右**偏 (右片後緣下、左片後緣上)；副翼不跟著動 | □ 對 → 可用版本 A　□ 反 → 只用版本 B |
| **E** | 俯衝時方向舵權限剩 ≥ 85% (先照 `RUDDER_BENCH.md` 7.1 節確認結果有效) | ___% |

測完把飛控切回 **MANUAL**。

### 2-7. 整機掛機 (F1)：任務狀態機 + 雲台自動鎖定 ★

1. Mission Planner 讀飛機目前 GPS：`LAT`、`LON`
2. 虛擬目標起點 = (`LAT + 0.0054`, `LON`)，在飛機北方約 600 m，往南 10 m/s 開過來
3. 執行 (Terminal 1 的 MAVROS 保持開著)：
   ```bash
   python3 xwing_virtual_flight.py --moving --bench --ground-test \
           --start <LAT+0.0054>,<LON> --tgt-heading 180 --case gimbal_bench_F1
   ```
4. 開機訊息確認：`雲台視覺 UART: ... 模式鍵 ON`、狀態列 `VIS link=1`
5. 撥 **GUIDED**，依序要看到：
   ```
   ★ GUIDED 上升沿 — 種點 + ARM (任務開始)
   雲台狀態 → SEARCH: 模式鍵 '1'
   雲台回報模式 '1' 已確認
   ... 約 18.5 s ...
   ★★ MODE: SEARCH → PRE_DIVE (... 到目標水平距=416m ...)
   雲台狀態 → PRE_DIVE: 模式鍵 '2'
   雲台回報模式 '2' 已確認
   ★ 雲台鎖定到位 pitch=-29.x° (要求後 x.xx s, 任務狀態 PRE_DIVE)
   ```
6. **進 PRE_DIVE 時看尾翼** ★：程式命令低頭 −30°、方向舵歸中
   - [ ] 兩片 V 尾**往同一個方向**動 (只有升降舵、低頭方向)
   - [ ] **沒有**一片上、一片下的差動 → 有差動就**不飛**
7. 雲台 (有接的話)：
   - [ ] 一進 PRE_DIVE，雲台往下轉到 −30° 停住
   - [ ] 「鎖定到位」要求後 ≤ 1.5 s：______ s
8. PRE_DIVE 45 s 後會自動 ABORT (`PREDIVE_TIMEOUT`)，掛機時正常
9. 撥回 **MANUAL** → Terminal 2 輸入 `q` 按 Enter
10. 雲台復原：`python3 xwing_gimbal_mode_test.py` → 按 `1` → `q`

### 2-8. (選做) 異常測試

在 2-7 執行中：拔掉 USB-UART → events 出現「★★ 雲台視覺資料鏈中斷 ... ← 控制不受影響」，程式不當機；
插回 → 「雲台視覺資料鏈恢復」。

---

## 第 3 步：飛行日

### 3-1. 到場設定

1. **目標起點**：確認 `MOVING_START_GPS = (23.7221111, 120.3798889)` 在場地上是對的位置。
   要移動起點時，飛行指令加 `--start lat,lon`。
2. **航線**：Mission Planner 載入 `xwing_virtual_mission.waypoints` (240 m 跑道)：

   | 航點 | 說明 |
   |---|---|
   | WP1 | 進場邊起點 (目標起點北方約 1300 m) |
   | WP2 | **撥 GUIDED 的參考點** (目標起點北方 700 m) |
   | WP3 | 進場邊終點 (目標起點南方 500 m) |
   | WP4、WP5 | 東側回程邊 |
   | DO_JUMP | 回 WP1，無限繞 |

   - 起飛和降落照你們平常 X-wing 的方式加在前後 (`Q_ENABLE = 0`，**不是 VTOL**)
   - 如果你有自己規劃的 GUIDED 切換點，以你的為準：**轉彎完成、航向已對正目標起點、離目標起點至少約 575 m**
3. **圍欄**：多邊形 (`FENCE_TYPE = 4`) 要包住整條跑道、俯衝和拉平段。
   碰到圍欄會 RTL，而且 `FENCE_OPTIONS = 1`，回到圍欄內之前**不能切模式**。
4. 上傳航線和圍欄，再讀回來確認。
5. (有真車時) 車子停在目標起點，**撥 GUIDED 那一刻**往南出發，**36 km/h (10 m/s)**。用無線電倒數。

### 3-2. 開機

順序：遙控器 → 對方 Orin NX + 雲台 → 我們的 Orin NX → 飛控 → 等 GPS fix。

- [ ] 對方的發送程式在跑
- [ ] 遙控器方向舵 trim 歸中
- [ ] Mission Planner 再確認一次：`PTCH_LIM_MIN_DEG = -40`、`AIRSPEED_MAX = 60`

### 3-3. 起飛前雲台快速確認 (主程式還沒開)

```bash
python3 xwing_gimbal_mode_test.py      # 按 p → 雲台往下轉、[LOCKED] → 按 1 → 按 q
```

### 3-4. 開程式 (起飛前就開好)

```bash
# Terminal 1
roslaunch mavros apm.launch

# Terminal 2 (二選一)
python3 xwing_virtual_flight.py --moving --dive-hold-heading --case flight1   # 版本 B
python3 xwing_virtual_flight.py --moving --case flight1                       # 版本 A (10/17 掛機通過，建議第二趟)
```

> ⚠ **開程式前飛控要在 AUTO / FBWA / MANUAL，不能在 GUIDED。** 程式啟動時如果飛控已在 GUIDED，
> 會立刻當成「GUIDED 上升沿」開始任務 (10/17 地面測試確認)。

### 3-5. 開機訊息檢查 (有問題就不飛)

要看到：
```
[xwing] 移動目標: 起點 (23.7221111, 120.3798889) 方位 180.3° 速度 10.0 m/s ...
[xwing] 巡航 240 m / 30 m/s, 俯衝 30° → 120 m 拉平, 速度上限 60 m/s
[xwing] 推頭門檻: 到目標水平距 ≤ 416 m (純幾何 416 m)
[xwing] 雲台視覺 UART: ... (資料只記錄); 模式鍵 ON
[UARTVision] 已開啟 ... @ 115200 8N1
[Bridge] PTCH_LIM_MIN_DEG=-40° OK
[Bridge] GUIDED_P=5000.000 → 43002 航向控制可用
[Bridge] 串流率設定完成 6/6
[Bridge] 方向舵迴路 ON — ...
[xwing] AIRSPEED_MIN=25 m/s OK
[xwing] AIRSPEED_MAX=60 m/s OK
[xwing] AIRSPEED_CRUISE→巡航 30 m/s OK
每秒: [SEARCH  ] ... g=0 | VIS link=1 ...
```
(X-wing 不會出現 `GUIDED_TIMEOUT` 那行，正常。)

看到這些就**停下來**：

| 訊息 | 問題 |
|---|---|
| `★★★ 警告: PTCH_LIM_MIN_DEG=... 淺` | 參數沒改到 |
| `★★★ < 速度上限 60 m/s` | `AIRSPEED_MAX` 沒改到 |
| `★★★ 找不到 GUIDED_P` | 不應該發生 (這台有)；表示連錯飛控或參數讀取失敗 |
| `★★★ RUDDER_VERIFIED=False` | 方向舵命令全部歸零 |
| `串流率設定完成` 少於 6/6 | MAVROS 連線或鮑率有問題 |
| `模式鍵 OFF` / `VIS link=0` 一直不變 | 雲台沒連上 (可以飛，但這趟沒有雲台資料) |

### 3-6. 起飛 → AUTO

1. 照平常方式起飛 → 切 **AUTO**，爬到 240 m 進跑道
2. 程式畫面每秒印 `[SEARCH  ] ... g=0 | VIS link=1 ...` (還沒撥 GUIDED)
3. 先**繞一圈不撥 GUIDED**，確認：
   - [ ] 高度約 240 m、空速約 30 m/s
   - [ ] 轉彎後在 WP1 → WP2 之間有對正往南
   - [ ] `VIS link=1`

### 3-7. 撥 GUIDED (任務開始)

1. 飛機轉完彎、在 WP1 → WP2 對正往南後，**飛過 WP2 (或你規劃的 GUIDED 點) 時**，地面站 / 操作員喊「GUIDED」
2. **地面站** 在 Mission Planner 按 Set Mode **GUIDED** (事先選好 GUIDED，到點只按一下)；(有車的話) 車子同時出發
   - 經數傳切換約有 0.5–1 s 延遲 (約 15–30 m)，可以稍微提早按
   - 確認 HUD 顯示 Guided、程式出現 `★ GUIDED 上升沿`
3. 之後全自動。操作員照畫面念出狀態：

| 時間 (撥 GUIDED 後) | 畫面 | 飛機 |
|---|---|---|
| 0 s | `★ GUIDED 上升沿 — 種點 + ARM`、`雲台狀態 → SEARCH: 模式鍵 '1'` | 前幾秒機翼保持水平 (韌體內建逾時)，之後朝目標修航向 |
| 約 14 s | `★★ MODE: SEARCH → PRE_DIVE`、`雲台狀態 → PRE_DIVE: 模式鍵 '2'`、`★ 雲台鎖定到位` | 開始推頭 |
| 約 15.5 s | `★★ MODE: PRE_DIVE → DIVE` | 30° 俯衝，空速 32 → 46 m/s |
| 約 21 s | `★★ MODE: DIVE → PULLOUT` | 約 120 m 拉平 |
| 約 21.5 s | `★★ MODE: PULLOUT → DONE` | 平飛往南，最低約 100 m 後回到 120 m |

4. 看到 `MODE: PULLOUT → DONE` 之後，地面站切回 **AUTO**，飛機爬回 240 m、回跑道。

### 3-8. 中止與緊急處理

| 狀況 | 誰 | 怎麼做 |
|---|---|---|
| 姿態、高度、速度看起來不對 | **飛手** | **模式開關撥到 FBWA** (要跟目前位置不同)，最快 |
| 同上，飛手來不及 | 地面站 | Mission Planner 切 AUTO 或 FBWA |
| 程式異常但飛機穩定 | 操作員 | Terminal 2 輸入 `x` 按 Enter → ABORT：拉起 2.5 s 後放手 → 飛手切 AUTO |
| 高度低於 100 m (PRE_DIVE / DIVE 中) | 程式 | 自動 ABORT 拉起 |
| 碰到圍欄 | 飛控 | 自動 RTL (回圍欄內前不能切模式) |
| 雲台資料中斷、模式鍵沒確認 | — | **不用處理**，只記在 events，不影響控制 |

### 3-9. 再飛一趟

程式到 DONE (或 ABORT) 後要重開才能再飛：

1. 飛機在 **AUTO** 繞圈時，Terminal 2 輸入 `q` 按 Enter
2. 重新執行 3-4 節的指令，`--case` 換名字 (例 `flight2`)
3. 確認開機訊息 (3-5 節)
4. 下一圈飛過 GUIDED 點時再撥 GUIDED

### 3-10. 降落

1. 照平常方式降落、上鎖 (disarm)
2. Terminal 2 輸入 `q` 按 Enter (或 Ctrl+C)
3. 雲台還停在鎖定 → `python3 xwing_gimbal_mode_test.py` → 按 `1` → `q`

---

## 第 4 步：飛行後

### 4-1. 收 log

| log | 位置 |
|---|---|
| 我們的主 log (50 Hz) | `flight_logs/xwing_virtual_<日期_時間>_<case>.csv` |
| 事件 | `flight_logs/xwing_virtual_<日期_時間>_<case>_events.txt` |
| 雲台測試 | `bench_logs/gimbal_mode_test_<時間>.csv` |
| 方向舵測試 | `bench_logs/rudder_bench_<時間>_*.csv` |
| 飛控 .BIN | Mission Planner → DataFlash Logs → Download (⚠ `LOG_DISARMED = 0`，地面掛機時沒有記) |
| 對方的 log | 跟對方要 |

### 4-2. 先看 events

```bash
ls -lt flight_logs/ | head
cat flight_logs/xwing_virtual_<日期_時間>_flight1_events.txt
```

| 看什麼 | 記錄 |
|---|---|
| 撥 GUIDED → PRE_DIVE 幾秒 (SEARCH 長度) | ______ s |
| `★ 雲台鎖定到位` 要求後幾秒 | ______ s |
| PULLOUT 時的 AGL、空速 | ______ m、______ m/s |
| `★★★ 成績: miss = ... m` (落點誤差) | ______ m |
| 有沒有 ABORT、原因 | |
| `雲台視覺 UART 統計` 的 `crc`、`mode_tx` | |

### 4-3. 把 log 給我

把 `flight_logs/` 這趟的 `.csv`、`_events.txt` 和飛控 `.BIN` 傳上來，就可以分析這趟：
SEARCH 航向收斂、俯衝角、拉平高度、miss distance，以及雲台資料跟虛擬目標的比對。

---

## 附錄：整個流程勾選表

| # | 項目 | 結果 |
|---|---|---|
| 0-2 | `PTCH_LIM_MIN_DEG = -40` | □ |
| 0-2 | `AIRSPEED_MAX = 60` | □ |
| 0-2 | MAVROS 序列埠 / 鮑率確認 | □ |
| 0-2 | 韌體版本記錄 | ________ |
| 0-3 | `VISION_UART_PORT` 填好 | □ |
| 1-1 | 雲台連線 CRC=OK | □ |
| 1-2 | 雲台鎖到 −30° (實測 ____°)、到位 ____ s | □ |
| 1-3 | 5 輪全部通過 | □ |
| 1-4 | 鎖定相對水平面 | □ 水平面　□ 機身 |
| 2-3 | MANUAL 舵面方向正確 | □ |
| 2-4 | 地面站切得到 GUIDED、開關撥 FBWA 能離開 | □ |
| 2-5 | R = __ E = __ A = __ | |
| 2-6 | Test A 成立 | □ (不成立不飛) |
| 2-6 | Test B 正負號 | □ 對 (版本 A 可用)　□ 反 |
| 2-6 | Test E 權限 ____% | |
| 2-7 | F1：PRE_DIVE V 尾無差動 | □ (有差動不飛) |
| 2-7 | F1：雲台自動鎖定 ____ s | □ |
| 3-5 | 開機訊息全部 OK | □ |
| 3-6 | AUTO 繞一圈：240 m / 30 m/s / 對正 / VIS link=1 | □ |
| 3-7 | 版本 □ A　□ B，GUIDED → DONE 完成 | □ |

# X-wing 地面測試報告 (2026/10/17)

> 測試清單：`xwing_virtual_TEST_CHECKLIST.txt` 第一階段 (第 1–6 項)
> 原始資料：`test_records/20261017_xwing/`
>
> **結論：第一階段全部通過。X-wing 可以準備實飛 (先飛版本 B，再飛版本 A)。**
> 雲台 (第 7–11 項) 等對方相機到了再做。

---

## 0. 總表

| # | 項目 | 結果 | 依據 |
|---|---|---|---|
| 1 | X-wing 舵機配置 | ✓ | SERVO1/7 副翼、2/5 V 尾右、3/6 V 尾左、4 油門 |
| 2 | 飛控參數 | ✓ | `PTCH_LIM_MIN_DEG = −45`、`AIRSPEED_MAX = 60` |
| 3 | 地面站切 GUIDED | ✓ | Mission Planner 切換成功，程式偵測到 GUIDED 上升沿 |
| 4 | 舵面通道 | ✓ | **R = 3 (V 尾左)、E = 2 (V 尾右)、A = 1 (副翼)** |
| 5 | 方向舵 Test A / B / E、方向 | ✓ | 送 180 = 歸中、45° = 滿舵、正舵量 = 右偏航 |
| 6 | 整機掛機 F1 | ✓ | 撥 GUIDED 後 18.2 s 進 PRE_DIVE，方向舵全程歸中 |
| 7–11 | 雲台 | 未做 | 對方相機尚未接上 |
| — | 實飛指令 (實驗室) | ✓ | 程式、MAVROS、飛控之間命令正常 |

---

## 1. 測試環境

| 項目 | 內容 |
|---|---|
| 地點 | 實驗室 (約 22.9954, 120.2235) |
| 飛機 | X-wing (`Q_ENABLE = 0`，V 尾混控，`MIXING_GAIN = 1`) |
| 另外用來驗證程式 | 逆行者 VTOL (一般尾翼：ch1 副翼、ch2 升降、ch3 油門、ch4 方向舵) |
| MAVROS 連線 | `/dev/ttyACM0` = 飛控 **USB (SERIAL0)**，不受鮑率限制 |
| GUIDED 切換 | **地面站 Mission Planner** (`FLTMODE_GCSBLOCK = 0`)，模式開關不用改 |
| 雲台 | 未接 (`/dev/ttyUSB0` 不存在)，視覺資料鏈中斷屬預期 |
| 安全 | 拆槳、未解鎖；油門輸出全程 1000 (X-wing) / 1100 (逆行者) |

---

## 2. 遇到的問題與解法

### 2.1 MAVROS 打不開飛控：`Permission denied`

```
FCU: DeviceError:serial:open: Permission denied
```

- 原因：使用者不在 `dialout` 群組，沒有 `/dev/ttyACM0` 的權限。
- 解法：
  ```bash
  sudo usermod -aG dialout $USER     # 永久, 重新登入後生效
  sudo chmod 666 /dev/ttyACM0        # 臨時, USB 重插或重開機後失效
  ```

### 2.2 一開始測到的不是 X-wing

10/15、10/17 上午的 log 舵機輸出是 ch1 = 1540、ch2 = 1500、ch3 = 1100、ch4 = 1580，跟 X-wing 參數對不上。
確認後是**逆行者 VTOL** (一般尾翼)。用逆行者驗證了程式 ↔ MAVROS ↔ 飛控這條路，X-wing 特有的部分之後在 X-wing 上重測。

### 2.3 實飛指令在實驗室不會進 PRE_DIVE (正常)

實飛指令 (沒有 `--bench`) 的虛擬目標在 82 km 外的場地，而且地面 AGL ≈ 0 < 進入門檻 200 m，所以永遠停在 SEARCH。
地面測試要用 `--bench --ground-test --start <緯度+0.0054>,<經度> --tgt-heading 180`。

### 2.4 有一次沒有切成 GUIDED

16:12、16:13 兩次掛機飛控全程 MANUAL，目標距離停在 599 m 不動 (虛擬目標要切 GUIDED 才開始移動)。
16:20 重做，地面站切 GUIDED 成功。

### 2.5 ★ 程式啟動時飛控已經在 GUIDED → 立刻開始任務

16:32 那次，開程式時飛控還停在上一次的 GUIDED，程式在 0.15 s 就判定「GUIDED 上升沿」並開始任務。

- **規則：開程式前，飛控一定要在 AUTO / FBWA / MANUAL，不能在 GUIDED。**
- 實飛時如果要在空中重開程式，要在 AUTO 繞圈時重開 (流程本來就是這樣)。
- 程式維持原本行為，不修改 (2026/10/17 決定)。

---

## 3. 飛控參數 (第 1、2 項)

完整參數：`test_records/20261017_xwing/params/`

### 3.1 這次修改的

| 參數 | 修改前 | 修改後 | 程式需求 | 結果 |
|---|---|---|---|---|
| `AIRSPEED_MAX` | 50 | **60** | ≥ 60 | ✓ |
| `PTCH_LIM_MIN_DEG` | −35 | **−45** | ≤ −40 | ✓ (比需求深，FBWA 推桿最多也能到 −45°) |

其他差異 (`ARSPD_OFFSET`、`BARO1_GND_PRESS`、`INS_GYROFFS_*`、`COMPASS_DEC`、`STAT_*`、`SR0_*`) 都是飛控開機自動校正或統計值，不是人為修改。

### 3.2 確認沒問題的

| 參數 | 值 | 說明 |
|---|---|---|
| `SERVO1/7_FUNCTION` | 4 | 副翼 |
| `SERVO2/5_FUNCTION` | 80 | V 尾右 |
| `SERVO3/6_FUNCTION` | 79 | V 尾左 |
| `SERVO4_FUNCTION` | 70 | 油門 (MIN 1000) |
| `MIXING_GAIN` | 1 | V 尾混控增益 (見 5.3) |
| `AIRSPEED_MIN` / `AIRSPEED_CRUISE` | 25 / 30 | ✓ |
| `GUIDED_P` | 5000 | 43002 航向控制器存在 |
| `TRIM_THROTTLE` | 45 | 與程式 `THR_TRIM = 0.45` 一致 |
| `ARSPD_USE` | 1 | 有空速計 |
| `FLTMODE_GCSBLOCK` | 0 | 地面站可切模式 |
| `BATT2_LOW_VOLT` | 46 | 主電池低電壓 (約 12S)；`BATT_LOW_VOLT 7.2` 應是航電電池 |

### 3.3 注意事項

- X-wing 韌體**沒有 `GUIDED_TIMEOUT`** 參數。實測強制姿態的內建逾時約 **3 秒** (見 6.3)。
- `AIRSPEED_MAX = 60` 只是允許上限，請確認機體能承受 60 m/s。
- `STICK_MIXING = 1`：GUIDED 中飛手不要碰搖桿。
- `LOG_DISARMED = 0`：未解鎖的地面測試不會記進飛控 .BIN。

---

## 4. 舵面通道 (第 4 項)

| 代號 | 通道 | 功能 |
|---|---|---|
| **R** | **3** | V 尾左 (正方向舵 → PWM 變大) |
| **E** | **2** | V 尾右 |
| **A** | **1** | 副翼 |

```bash
python3 test_rudder_bench.py --test A --rudder-ch 3 --aileron-ch 1 --elevator-ch 2
python3 test_rudder_bench.py --test B --rudder-ch 3 --aileron-ch 1 --elevator-ch 2
python3 test_rudder_bench.py --test E --rudder-ch 3 --elevator-ch 2
```

---

## 5. 方向舵測試 (第 5 項)

### 5.1 Test A：yaw 欄位 = 方向舵命令 — 成立 (做了 3 次)

| 送出 yaw | 方向舵量 | 第 1 次 ΔPWM (ch3 / ch2) | 第 2 次 | 第 3 次 |
|---|---|---|---|---|
| 180 | 0 | −1 / 0 | +1 | 0 |
| 176 | +4° | +43 / −44 | +44 | +44 |
| 170 | +10° | +109 / −111 | +110 | +111 |
| 160 | +20° | +221 / −222 | +221 | +222 |
| 135 / 90 / 45 / 0 | 飽和 | +498 (1980) / 1000 | +499 | +500 |
| 315 / 270 / 225 | 反向飽和 | −482 (1000) / 1981 | −481 | −481 |
| 200 | −20° | −222 / +222 | −222 | −222 |

- **送 180 = 方向舵歸中**，與 HERO2180 (2026/09/11) 相同 → PRE_DIVE 的「方向舵強制歸中」在 X-wing 上正確。
- 兩片 V 尾鏡像差動 → V 尾混控正常。
- 副翼全程 1516–1519 → 方向舵不影響滾轉。
- 三次差距 ≤ ±1 µs，非常穩定。

### 5.2 Test B：線性與正負號 — 通過

| 舵量 | +3 | +5 | +10 | +20 | +30 | +45 | +60 |
|---|---|---|---|---|---|---|---|
| ΔPWM | +32 | +55 | +110 | +221 | +333 | +499 | +499 |
| **舵量** | −3 | −5 | −10 | −20 | −30 | −45 | −60 |
| ΔPWM | −33 | −55 | −111 | −222 | −333 | −481 | −481 |

- 線性、對稱，**約 11 µs / 度**；45° = 滿舵。
- 負邊 −481 是因為中立點 1481、下限 1000，正常。

**方向確認：** 程式正舵量 → ch3 ↑ / ch2 ↓；飛控自己在 GUIDED 左轉時加的協調舵 (`KFF_RDDRMIX`) → ch3 ↓ / ch2 ↑。兩者相反，表示程式的「正 = 右偏航」與飛控定義一致。
`RUDDER_SIGN = +1` 不用改。

### 5.3 Test E：俯衝時方向舵權限 — 有效，地面結果偏悲觀

| pitch 命令 | 兩片 V 尾基準 | ±5° 差動量 | 權限 |
|---|---|---|---|
| 0° | 1481 / 1481 | 110 | 100% |
| −10° | 1295 / 1295 | 111 | 101% |
| −20° | 1109 / 1109 | 111 | 101% |
| −28.7° | **1000 / 1000** | 56 | **51%** |

- 有效性：B 欄基準隨 pitch 改變 (1481 → 1000)，飛控有吃到 pitch 命令。
- −28.7° 的 51% 是因為**地面上飛機水平、命令 −28.7°**，誤差 28.7° 讓升降舵打到底 (約 18.6 µs / 度)。
- 實際俯衝時飛機真的在 −30°，誤差只有幾度，升降舵不會頂到底，方向舵權限應接近 −10°、−20° 那兩列 (≈ 100%)。
- 升降舵真的會打到底只有 PRE_DIVE 推頭約 1.4 s，那段方向舵本來就強制歸中。

### 5.4 X-wing 與 HERO2180 的差異

| | HERO2180 | X-wing |
|---|---|---|
| 滿舵 ΔPWM | 約 305 | **約 499** |
| 每 1° 命令 | 約 6.8 µs | **約 11 µs** |

X-wing 方向舵約 **1.6 倍**敏感 (`MIXING_GAIN = 1` + V 尾行程大)。方向舵 PID 增益是在 HERO2180 上調的，版本 A 在 X-wing 上可能修過頭 → **第一趟先飛版本 B**。

---

## 6. 程式與飛控 (第 6 項 + 實飛指令)

### 6.1 整機掛機 F1 (16:20) — 通過

指令：
```bash
python3 xwing_virtual_flight.py --moving --bench --ground-test \
        --start 23.0007419,120.2235203 --tgt-heading 180 --no-vision --case bench_F1
```

| 時間 | 事件 | 結果 |
|---|---|---|
| 13.75 s | 地面站切 GUIDED → 任務 ARM；TECS 240 m / 30 m/s ok | ✓ |
| 13.75–31.97 s | SEARCH，目標距離 598 → 416 m (10 m/s) | ✓ |
| **31.97 s** | **PRE_DIVE** (切 GUIDED 後 **18.2 s**，預估 18.5 s)；TECS 120 m ok | ✓ |
| 76.99 s | PRE_DIVE 45 s 逾時 → ABORT (地面正常) | ✓ |
| 79.51 s | DONE | ✓ |

**PRE_DIVE 時 V 尾 (最關鍵)：**

| 時間 | ch2 | ch3 | 飛控目標 pitch |
|---|---|---|---|
| SEARCH | 1470 | 1492 | 0° |
| PRE_DIVE +1 s | 1166 | 1166 | −30° |
| PRE_DIVE +2 s ~ 結束 | 1000 | 1000 | −30° |

PRE_DIVE 45 s (2251 筆) **ch2 = ch3**，平均差 0.05 µs → **方向舵確實歸中，只有升降舵在動**。

其他：
- SEARCH：`--ground-test` 保持機頭 170.4°，副翼不動 ✓
- ABORT：+12° 拉起，兩片一起到 1704，無差動 ✓
- DONE 交還飛控：+12° 維持約 3 s 後回中 → 內建逾時約 3 s ✓
- 油門：程式命令 0.85，ch4 全程 1000 (未解鎖) ✓
- `errors = 0`，50 Hz 穩定

### 6.2 實飛指令在實驗室 (16:33) — 正常

| 時間 | 事件 |
|---|---|
| 0–36 s | MANUAL，程式等待 |
| 36.47 s | 地面站切 GUIDED → 任務開始 |
| 36–39 s | 飛控目標坡度維持 0° (內建逾時約 3 s) |
| 39 s 起 | 43002 改航向一直被接受 (0 → 122 次)，目標坡度 −22° (左轉，方向正確) |

X-wing 在 GUIDED 左轉時的舵面：

| 通道 | 中立 | 左轉 |
|---|---|---|
| ch1 副翼 | 1505 | 1243 |
| ch2 V 尾右 | 1480 | 1695 |
| ch3 V 尾左 | 1480 | 1266 |

ch2 / ch3 平均不變 (1480) → 純方向舵 (飛控的協調舵)。飛機在架上轉不動，所以舵量會一直累加到接近滿舵，正常。

### 6.3 強制姿態的內建逾時約 3 秒 (逆行者、X-wing 都一樣)

| 測試 | 切 GUIDED | 飛控目標坡度開始跟 43002 |
|---|---|---|
| 逆行者 14:20 | 17.42 s | 約 21 s |
| X-wing 16:33 | 36.47 s | 約 39–42 s |

撥 GUIDED 後前約 3 秒機翼保持水平，之後才開始轉向。SEARCH 設計約 14 s，影響不大。

---

## 7. 結論與實飛前待辦

### 7.1 可以飛

| 項目 | 結論 |
|---|---|
| 版本 B (`--dive-hold-heading`) | ✓ **第一趟用這個** |
| 版本 A (DIVE 用方向舵 PID) | ✓ 方向、歸中都正確，可以飛；但增益偏高，建議第二趟再飛 |

### 7.2 實飛前待辦

- [ ] 練一次中止：GUIDED 中飛手把模式開關撥到 FBWA → 立刻離開 GUIDED
- [ ] 確認機體能承受 60 m/s
- [ ] 確認兩組電池電壓在 Mission Planner 都有顯示
- [ ] 開程式前飛控**不能在 GUIDED**
- [ ] 沒接雲台時加 `--no-vision`

### 7.3 實飛指令

```bash
roslaunch mavros apm.launch                                    # Terminal 1

# Terminal 2 — 第一趟 (版本 B)
python3 xwing_virtual_flight.py --moving --dive-hold-heading --case flight1_B \
        2>&1 | tee console_flight1_B.txt

# 第二趟 (版本 A)
python3 xwing_virtual_flight.py --moving --case flight2_A \
        2>&1 | tee console_flight2_A.txt
```

### 7.4 之後

- 雲台第 7–11 項 (對方相機到了再做)
- 實飛後分析：SEARCH 航向收斂、俯衝角、拉平高度、miss distance；版本 A 的方向舵是否擺動、要不要降增益

---

## 8. 原始資料索引 (`test_records/20261017_xwing/`)

| 檔案 | 內容 |
|---|---|
| `params/x-wing_before.param` | 修改前參數 |
| `params/x-wing_20261017_final.param` | 修改後參數 (`AIRSPEED_MAX 60`、`PTCH_LIM_MIN_DEG −45`) |
| `rudder_bench/rudder_bench_20261017_152728_A.csv` | Test A 第 1 次 |
| `rudder_bench/rudder_bench_20261017_152959_B.csv` | Test B |
| `rudder_bench/rudder_bench_20261017_154629_A.csv`、`..._154922_A.csv` | Test A 第 2、3 次 |
| `rudder_bench/rudder_bench_20261017_155625_E.csv` | Test E |
| `flight_program/xwing_virtual_20261017_162037_bench_F1.*` | 整機掛機 F1 (通過) |
| `flight_program/xwing_virtual_20261017_163321_moving_A_yawpid.*` | 實飛指令，地面站切 GUIDED |
| `flight_program/xwing_virtual_20261017_163245_moving_A_yawpid_events.txt` | 啟動時已在 GUIDED → 立刻開始任務 (2.5 節) |
| `flight_program/nixingzhe_vtol_20261017_142045_events.txt` | 逆行者 VTOL 的 GUIDED 測試 |

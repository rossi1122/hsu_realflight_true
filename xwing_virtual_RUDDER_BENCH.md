# X-wing 方向舵掛機測試流程 (test_rudder_bench.py)

> 目的：確認 X-wing 上「程式送出的方向舵命令」和「飛控實際打出的舵面」之間的對應關係與正負號，
> 決定 DIVE 段能不能用 PID 控制方向舵 (版本 A)，還是要用 `--dive-hold-heading` (版本 B)。
>
> 背景：`attitude_bridge.py` 裡的方向舵設定是 2026/09/11 在 **HERO2180** 上量的：
> ```python
> YAW_FIELD_OFFSET_DEG = 180.0   # 飛控收到 = 180 − 送出值
> YAW_FIELD_SIGN = -1.0
> RUDDER_VERIFIED = True         # 掛機驗證過才 True; False → 方向舵命令一律歸零
> RUDDER_SIGN = +1.0             # 正舵量 = 往右偏航
> ```
> X-wing 的舵機接線、通道、正反向、尾翼混控都可能不同，**實飛前必須重新測。**

> ⚠ **測完之前，實飛請一律加 `--dive-hold-heading`** (DIVE 不打方向舵、只修 pitch)。

> ✅ **2026/10/17 X-wing 已測完，全部通過** (詳見 `xwing_virtual_TEST_REPORT_20261017.md` 第 4、5 節)：
> - 通道：`--rudder-ch 3 --aileron-ch 1 --elevator-ch 2` (R = V 尾左、E = V 尾右、A = 副翼)
> - Test A：送 180 = 歸中、45° = 滿舵，與 HERO2180 相同 (做 3 次一致)
> - Test B：線性、對稱，約 11 µs / 度 (HERO2180 約 6.8)，正舵量 = 右偏航 → `RUDDER_SIGN = +1` 不用改
> - Test E：−28.7° 時 51%，是地面固定在架上造成的；實際俯衝應接近 100%
> - 結論：版本 A 可用；X-wing 方向舵約 1.6 倍敏感，建議第一趟先飛版本 B

---

## 0. 安全規定 (每一次都要做)

- [ ] **拆掉螺旋槳**
- [ ] 飛機固定在架上，或由一人壓住
- [ ] 舵面、連桿附近淨空，手指遠離
- [ ] 飛控保持 **disarmed** (不需要 arm；ArduPlane disarmed 時舵面照樣會動，程式送的油門一律為 0)

---

## 1. 準備

### 1.1 人員與器材

| 項目 | 說明 |
|---|---|
| 操作員 ×1 | 操作機載電腦、Mission Planner |
| 觀察員 ×1 | **站在機尾正後方、面向機頭方向**看舵面 |
| X-wing | 拆槳、固定 |
| 機載電腦 + 飛控 | 與實飛相同的接線與 MAVROS 連線 |
| 遙控器 | 開機。**方向舵 trim 歸中**，測試中搖桿不要碰 |
| Mission Planner | 切模式、查參數 |
| GPS | 建議在室外、有 GPS fix 的地方做 (ArduPlane 沒有位置資訊時可能切不進 GUIDED) |

### 1.2 飛控狀態

- [ ] 飛控通電，Mission Planner 已連線
- [ ] 若有**安全開關 (safety switch)**，先按掉，否則所有輸出被鎖住、舵面不會動
- [ ] `GUIDED_TIMEOUT` 不用改 (程式以 20 Hz 持續送命令，不會逾時)

### 1.3 在 Mission Planner 查尾翼配置

**Config → Full Parameter List**，找 `SERVO1_FUNCTION` ~ `SERVO8_FUNCTION`，填下表：

| 輸出通道 | `SERVOn_FUNCTION` | 代表 |
|---|---|---|
| SERVO1 | | |
| SERVO2 | | |
| SERVO3 | | |
| SERVO4 | | |
| SERVO5 | | |
| SERVO6 | | |

常見數值：

| 數值 | 意思 | 對本測試的影響 |
|---|---|---|
| `4` | 副翼 | `--aileron-ch` |
| `19` | 升降舵 | `--elevator-ch` |
| `21` | **方向舵 (獨立一片)** | 最單純，`--rudder-ch` 填這個通道號，**Test E 可以跳過** |
| `79` / `80` | V 尾左 / V 尾右 | 方向舵與升降舵共用舵面，**Test E 必做** |
| `77` / `78` | 飛翼左 / 右升降副翼 | **沒有方向舵** → DIVE 方向舵迴路不能用，實飛一律加 `--dive-hold-heading`，本測試不用做 |
| 其他混控 (例如 X 型尾翼四片) | — | 先記下配置，Test E 的算法可能要調整 |

另外記下：`MIXING_GAIN` = ________ (V 尾混控增益，Test E 判讀用)

---

## 2. 在家先做：離線預測 (不需要飛控)

```bash
python3 test_rudder_bench.py --predict-only
```

會印出「程式送出某個 yaw 值 → 飛控應該讀到多少 → 舵量多少」的對照表。
例如送 176 → 飛控讀到 4° → 舵量 400 (滿舵是 4500)。先看懂這張表，現場才知道 Test A 應該長什麼樣子。

---

## 3. 現場：啟動

```bash
# Terminal 1
roslaunch mavros apm.launch

# Terminal 2：到專案資料夾 (有 test_rudder_bench.py 和 attitude_bridge.py 的地方)
cd <專案資料夾>
```

每次執行 `test_rudder_bench.py` 都會先問：

```
★ 已確認【拆槳】、飛機已固定、舵面附近淨空了嗎? 輸入 yes 繼續:
```

確認過才輸入 `yes`。接著看到 `✓ 飛控模式 = ..., armed = False` 代表連線成功。

> 若出現 `✗ 收不到 /mavros/state` → MAVROS 沒有跑起來或沒連上飛控，先檢查 Terminal 1。

---

## 4. 步驟 1：找出舵面通道

```bash
python3 test_rudder_bench.py --list-channels
```

畫面會連續 10 秒顯示：

```
ch1=1500 ch2=1500 ch3=1000 ch4=1595 ...  mode=MANUAL
```

這 10 秒內用**遙控器**分別撥 **方向舵、升降舵、副翼** 搖桿，看哪一格數字跟著變。

| 舵面 | 通道號 | 中立 PWM |
|---|---|---|
| 方向舵 (或 V 尾左) → `R` | | |
| 升降舵 (或 V 尾右) → `E` | | |
| 副翼 → `A` | | |

> ArduPlane 預設 1=副翼、2=升降舵、3=油門、4=方向舵，但 **X-wing 不一定一樣，一定要實際確認。**
> 以下指令中的 `R`、`E`、`A` 請換成你查到的通道號。

---

## 5. 步驟 2：Test A — 確認 yaw 欄位就是方向舵命令

```bash
python3 test_rudder_bench.py --test A --rudder-ch R --aileron-ch A --elevator-ch E
```

1. 程式提示「不是 GUIDED」→ 用 Mission Planner (**Flight Data → Actions → 選 GUIDED → Set Mode**) 或遙控器切到 **GUIDED**，按 Enter。
2. 程式先送 yaw=180 當基準 (理論舵量 0)，再依序送
   `180, 176, 170, 160, 135, 90, 45, 0, 315, 270, 225, 200, 180`，每點約 2 秒，總共約 30 秒。
3. 每一列顯示：送出值、預測舵量、實測 PWM、ΔPWM (與基準差)。

**應該看到的形狀** (下表是 HERO2180 的實測數字；X-wing 數字會不同，但**形狀要一樣**)：

| 送出 yaw | 飛控讀到 | HERO2180 ΔPWM | X-wing ΔPWM (填) | 說明 |
|---|---|---|---|---|
| 180 | 0° | 0 | | 中立 |
| 176 | +4° | +28 | | 小量往一邊 |
| 170 | +10° | +71 | | |
| 160 | +20° | +142 | | |
| 135 | +45° | +305 | | 滿舵 |
| 90 / 45 / 0 | > 45° | 都 = 135 那格 | | 飽和 |
| 315 / 270 / 225 | < −45° | 都一樣 (另一邊滿舵) | | 飽和 |
| 200 | −20° | 反向 | | |

**程式自動判讀：**

| 畫面訊息 | 意思 | 下一步 |
|---|---|---|
| `→ 假設【成立】` | 對應關係與 HERO2180 相同 | 繼續 Test B |
| `→ 方向舵幾乎沒動` | 先檢查 `--rudder-ch` 是否正確、安全開關是否已關。都對仍不動 → 此韌體行為不同 | **DIVE 不可用方向舵迴路**，實飛加 `--dive-hold-heading` |
| `→ 有反應但形狀跟預測不一致` | 對應關係不同 | 保留 CSV，交給程式端重新計算 `YAW_FIELD_OFFSET_DEG` / `YAW_FIELD_SIGN` |

> 正、負兩邊滿舵的 PWM 大小不同是**正常的**：舵機中立點 (TRIM) 不在行程正中間。

Test A 結果：□ 成立　□ 不動　□ 形狀不一致

---

## 6. 步驟 3：Test B — 確認正負號 (最重要)

```bash
python3 test_rudder_bench.py --test B --rudder-ch R --aileron-ch A --elevator-ch E
```

**觀察員站在機尾正後方、面向機頭方向。**

程式依序指定舵量
`0, +3, +5, +10, +20, +30, +45, +60, 0, −3, −5, −10, −20, −30, −45, −60, 0`，
每點約 1.7 秒，總共約 30 秒。

### 6.1 判斷正負號 (決定 `RUDDER_SIGN`)

看**正舵量 (+10、+20、+30)** 時，方向舵**後緣**往哪邊偏：

| 正舵量時方向舵後緣往… | 機頭會往 | 結論 |
|---|---|---|
| **右** (從後面看) | 右 | 與 HERO2180 相同，`RUDDER_SIGN = +1.0`，**不用改** |
| **左** | 左 | 方向相反，`RUDDER_SIGN` 要改成 **`−1.0`** |

> V 尾 / X 型尾翼是多片一起動：看「整體往哪邊推」。後緣一起往右偏 = 往右偏航。

### 6.2 其他檢查

| 項目 | 應該 | 不對時 |
|---|---|---|
| ΔPWM 對舵量 | ±45° 內大致成正比 | 只要方向一致仍可用 |
| +60 與 +45 的 ΔPWM | **相同** (已飽和) | 不同 → 45° 不是滿舵，記下數字回報 |
| **副翼欄** | **幾乎不動** | 跟著動 → 方向舵命令污染了滾轉，回報 |

### 6.3 記錄

| 舵量 | +10 | +20 | +30 | +45 | +60 | −10 | −20 | −30 | −45 | −60 |
|---|---|---|---|---|---|---|---|---|---|---|
| ΔPWM | | | | | | | | | | |

正舵量時後緣往：□ 右 (`RUDDER_SIGN=+1`)　□ 左 (`RUDDER_SIGN=−1`)
副翼是否跟著動：□ 否　□ 是

---

## 7. 步驟 4：Test E — 俯衝時方向舵還剩多少作用

> 只有**方向舵與升降舵共用舵面** (V 尾 `79/80`、X 型尾翼等) 需要做。獨立方向舵 (`21`) 可跳過。

```bash
# V 尾：R = V 尾左通道，E = V 尾右通道
python3 test_rudder_bench.py --test E --rudder-ch R --elevator-ch E
```

會再問一次 `yes`：這個測試會讓舵面大幅偏轉，**手離開舵面與連桿**。

程式在 pitch = 0、−10、−20、−28.7° 時，各打 ±5° 方向舵，量兩片舵面的差動量，最後印出：

```
pitch=  -28.7° → 方向舵權限剩   XX%   OK / ★ 明顯衰退 / ★★★ 幾乎失效
```

| 俯衝 (−28.7°) 時剩下 | 結論 |
|---|---|
| ≥ 85% | DIVE 方向舵迴路可用 |
| 50–85% | 可用但變弱，左右修正能力差 |
| < 50% | **DIVE 不要用方向舵**，實飛加 `--dive-hold-heading` |

### ⚠ 7.1 Test E 有效性檢查 (一定要看)

`test_rudder_bench.py` 所有測試都用 **mask 7** 送命令。專案之前實測發現 **mask 7 時飛控不吃 pitch 命令**
(實飛程式後來改用 mask 128，見 `attitude_bridge.py` 第 554–561 行)。方向舵在 mask 7 下有反應，
所以 Test A、B 不受影響；但 Test E 要靠 pitch 把舵面推下去，結果可能不準。

**檢查方法**：看每個 pitch 的「基準」那一列，B 欄 (`--elevator-ch`) 的 PWM 有沒有隨 pitch 0 → −28.7° 明顯改變。

| pitch | 0 | −10 | −20 | −28.7 |
|---|---|---|---|---|
| A 欄基準 PWM | | | | |
| B 欄基準 PWM | | | | |
| 權限剩 % | 100 (基準) | | | |

- **B 欄有明顯變化** → 結果有效
- **B 欄幾乎沒變** → pitch 沒被套用，權限會一直顯示約 100%，**結果不能用**。需要把 Test E 改成 mask 128 再測。

> 另外：本版實飛俯衝命令是 −30°，Test E 固定測到 −28.7° (舊版值)，差距小，可直接參考。

---

## 8. (可選) Test C — 鍵盤即時微調

```bash
python3 test_rudder_bench.py --test C --rudder-ch R --aileron-ch A --elevator-ch E
```

| 按鍵 | 動作 |
|---|---|
| `d` / `a` | 舵量 +1° / −1° |
| `D` / `A` | 舵量 +5° / −5° |
| `w` / `s` | pitch +1° / −1° |
| `z` | 全部歸零 |
| `m` | 切換 mask (7 → 129 → 133) |
| `q` | 結束 (自動歸零) |

適合想自己動手、反覆確認方向時使用。

> **Test D 不用做**：它測的是舊的 mask (7 / 129 / 133)，不是實飛現在用的 128 / 132 / 198。

---

## 9. 測完收尾

- [ ] 用 Mission Planner 把飛控**切回 MANUAL 或 FBWA** (離開 GUIDED)
- [ ] 確認舵面回到中立
- [ ] 保留 log：`bench_logs/rudder_bench_<時間>_A.csv`、`_B.csv`、`_E.csv`

---

## 10. 依結果決定怎麼飛

| Test A | Test B | Test E | 決定 |
|---|---|---|---|
| 成立 | 後緣往右 | ≥ 85% 或不需要做 | **版本 A 可用**，設定不用改 |
| 成立 | **後緣往左** | ≥ 85% 或不需要做 | 把 `RUDDER_SIGN` 改成 `−1.0` 後，版本 A 可用 |
| 成立 | 任一 | 50–85% | 版本 A 可用但效果打折；建議先飛版本 B 比較 |
| **不成立 / 不動** | — | — | **只能用版本 B**：`--dive-hold-heading` |
| — | — | **< 50%** | **只能用版本 B**：`--dive-hold-heading` |
| 形狀不一致 | — | — | 把 CSV 交給程式端重算對應關係，算好之前用版本 B |
| 沒有方向舵 (飛翼) | — | — | **只能用版本 B** |

```bash
# 版本 A (DIVE 用 PID 打方向舵)
python3 xwing_virtual_flight.py --moving

# 版本 B (DIVE 不打方向舵，只修 pitch)
python3 xwing_virtual_flight.py --moving --dive-hold-heading
```

### 10.1 要改的設定在哪

目前這幾個值寫在 `attitude_bridge.py` 的 `AttitudeBridgeConfig` 裡，**xwing 版沒有覆寫**：

| 參數 | 行號 | 現值 (HERO2180) | X-wing 測試後 |
|---|---|---|---|
| `YAW_FIELD_OFFSET_DEG` | 141 | 180.0 | |
| `YAW_FIELD_SIGN` | 142 | −1.0 | |
| `RUDDER_VERIFIED` | 146 | True | |
| `RUDDER_SIGN` | 147 | +1.0 | |

> 直接改 `attitude_bridge.py` 會連原版 `run_vision_flight.py` 一起變。
> 建議改成在 `xwing_virtual_config.py` 加覆寫 (原檔不動)。

---

## 11. 常見問題

| 狀況 | 可能原因 | 處理 |
|---|---|---|
| 切不進 GUIDED | 沒有 GPS / 位置估測 | 到室外等 GPS fix |
| 舵面完全不動 | 安全開關沒關 / 通道填錯 | 按掉 safety switch；重跑 `--list-channels` |
| 中立 PWM 每次不同 | 遙控器 trim 或搖桿疊加到 GUIDED 命令 | trim 歸中，搖桿放開不要碰 |
| `★ 讀不到通道 N 的 RCOut` | `--rudder-ch` 超出範圍 | 重新確認通道號 |
| 測到一半想停 | — | 按 `Ctrl+C`，程式會自動把舵面歸零再結束 |

---

## 12. 結果總表 (測完填)

| 項目 | 結果 |
|---|---|
| 日期 / 地點 | |
| 尾翼配置 (`SERVOn_FUNCTION`) | |
| 方向舵 / 升降舵 / 副翼 通道 | R = ___ / E = ___ / A = ___ |
| Test A | □ 成立　□ 不動　□ 形狀不一致 |
| Test B 正舵量後緣方向 | □ 右　□ 左 |
| Test B 副翼是否跟動 | □ 否　□ 是 |
| Test E 俯衝剩餘權限 | ____ %　(B 欄有變化：□ 是　□ 否) |
| 結論 | □ 版本 A　□ 版本 A (改 `RUDDER_SIGN=−1`)　□ 版本 B |
| log 檔名 | |

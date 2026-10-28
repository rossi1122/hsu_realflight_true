# xwing_virtual 飛完畫圖 — 使用說明

飛完後一個指令出全部圖、一份 PDF，加上一份 `summary.txt`（時間線、各段數字、哪些參數可能要調）。
參考學長 realflight_plot 的圖表配置（Fig 1~9、虛擬目標 vs 真車 GPS），改寫成讀 **xwing_virtual 的 log**。

| 程式 | 對應學長的 | 做什麼 |
|---|---|---|
| `xwing_plot_report.py` | `plot_flight_report*.py` / `plot_moving_report*.py` | 主報告：Fig 1~10 + report.pdf + summary.txt（加 `--car-bin-dir` 會再畫 Fig 11） |
| `xwing_plot_target_gps.py` | `plot_target_gps_comparison.py` | 虛擬目標 vs 真車 GPS（BIN），可單獨跑 |

學長原本那幾支**不能直接讀 X-wing 的 log**：它們需要 `_vision.csv`（機上 YOLO）、`real_*` / `real_comp_pid_*` 欄位、`_raw.mp4`（`redetect_yolo.py`），X-wing 這版沒有 YOLO 也沒錄影，所以沒有放進來。

只需要 `numpy` + `matplotlib`（不用 pandas、pymavlink、ROS）。

---

## 1. 飛測前一天先確認（Orin 或筆電）

```bash
cd ~/catkin_ws/src/hsu_realflight_true
python3 -c "import numpy, matplotlib; print('OK')"
```
- 印出 `OK` 就可以。沒有的話（需要網路）：`pip3 install numpy matplotlib`
- 在筆電畫也可以：把整個專案資料夾（至少 `xwing_plot_report.py`、`xwing_plot_target_gps.py`、`xwing_virtual_config.py`）跟 log 一起拷過去，指令完全一樣（Windows 把 `python3` 換成 `python`）。

**不用飛機先試一次整個流程**（產生模擬的一趟 log + 假車子 BIN，再自動畫圖）：
```bash
python3 test_plot_report.py              # 一趟
python3 test_plot_report.py --two-runs   # 同一份 log 切兩次 GUIDED
```
最後出現 `[test] 完成` 就代表能畫，結果在 `plot_results/xwing_virtual_<時間>_plottest/run1/report.pdf`。
模擬的飛機模型很粗，圖裡的數字不代表實飛結果，只用來確認流程。

用之前的地面測試 log 試也可以（沒推頭，所以只有部分圖有內容）：
```bash
python3 xwing_plot_report.py flight_logs/<地面測試 log>.csv --full
```

---

## 2. 飛完後怎麼畫（照抄）

**(1) 只畫飛機（最常用）**
```bash
cd ~/catkin_ws/src/hsu_realflight_true
python3 xwing_plot_report.py --latest
```
`--latest` = `flight_logs/` 裡最新的那份 log。要畫指定的 log：
```bash
ls -lt flight_logs/ | head
python3 xwing_plot_report.py flight_logs/xwing_virtual_20261030_XXXXXX_flight1.csv
```

**一天有好幾份 log：全部一次畫**
```bash
python3 xwing_plot_report.py --all                    # flight_logs/ 每一份都畫, 最後印總表
python3 xwing_plot_report.py <log1.csv> <log2.csv>    # 只畫指定的幾份
```

**(2) 加上真車 GPS 比較（Fig 11）**
1. 車子飛控的 SD 卡 → `LOGS/` 裡當天的 `.BIN`，複製到專案底下的 `car_bin/` 資料夾（只放**當天**的就好，程式會用 GPS 時間自己挑對的那一個）。
2. 執行：
```bash
mkdir -p car_bin          # 第一次才需要
python3 xwing_plot_report.py --latest --car-bin-dir car_bin
```

**(3) 只畫某一趟 / 地面測試**
```bash
python3 xwing_plot_report.py <log.csv> --run 2     # 只畫第 2 趟
python3 xwing_plot_report.py <log.csv> --full      # 地面測試 / 沒有 GUIDED: 整份畫出來
```

一份 log 裡切了幾次 GUIDED（`run` 欄位 1、2、3…），就會**每一趟各出一個資料夾**。

---

## 3. 輸出在哪、給老師看什麼

```
plot_results/<log 檔名>/run1/
    report.pdf          ← 全部圖合成一份，直接開給老師看
    summary.txt         ← 時間線 + 數字 + 自動檢查（⚠ = 建議看的參數）
    fig1_overview.png ... fig11_target_gps.png
```
`summary.txt` 的內容也會直接印在終端機上。

所有圖的時間軸：**0 s = 撥 GUIDED 的那一刻**。底色：藍 SEARCH、黃 PRE_DIVE、綠 DIVE、紫 PULLOUT、灰 DONE、紅 ABORT。

---

## 4. 每張圖在看什麼 → 對應哪些參數

| 圖 | 內容 | 不對勁時看的參數 |
|---|---|---|
| Fig 1 總覽 | AGL（240 / 200 / 120 / 100 m 參考線）、空速（60 上限）、pitch 命令 vs 實際、到目標距離、油門 | 整體流程有沒有照設計走 |
| Fig 2 俯衝剖面 | (a) **地面座標側視**：飛機實際飛的高度 vs 沿目標方向的距離，對照從推頭點畫的 30° 線，目標畫在地面上（□），虛線是當下的視線；(b) **相對移動目標**的距離–高度（目標往前跑，所以看起來比實際陡，不是飛錯）；(c) 航跡角 γ、θ | `DIVE_ANGLE_DEG`、`ALPHA_DIVE_DEG`、撥 GUIDED 的高度/距離 |
| Fig 3 姿態追蹤 | pitch 命令 vs 實際（RMSE）、heading 命令 vs 實際、roll（25/35° roll 保護線） | 飛控 pitch 迴路（pitch 追不上 → 先調飛控，不是加 PID）、`GUIDED_P`（SEARCH 轉向）、`ROLL_PROT_*` |
| Fig 4 影像誤差 | DIVE 中虛擬目標在畫面上的橫向 / 縱向誤差（deg），右邊是目標在畫面上的軌跡 | `PID_LAT_KP/KI`、`PID_LON_KP/KI/KD` |
| Fig 5 PID 各項 | DIVE 中 P / I / D 各自的大小與輸出 | 哪一項主導；I 一直累積 = 有固定偏差 |
| Fig 6 方向舵 | raw / cmd / sent、±限幅、被空速或 roll 限制的區間、偏航角速度 r | `K_LAT_TO_RUDDER`、`RUDDER_LIMIT_DEG`、`RUDDER_SLEW_DPS`、`ROLL_PROT_*` |
| Fig 7 軌跡 | 俯視圖：飛機（依模式上色）vs 虛擬目標；(b) DIVE 放大（東西向放大看側偏） | 進場方向、推頭時側偏多少 |
| Fig 8 預測落點 | DIVE 中依當下速度直線外推的落點誤差（沿航跡 / 側向） | 側向收斂 = 方向舵在作用；沿航跡 = 俯衝角 / 拉平時機 |
| Fig 9 雲台 UART | 連線、偵測、模式鍵（要求 vs 回報）、雲台 pitch（−30° 鎖定線）、相機誤差 vs 虛擬誤差 | 雲台有沒有在 PRE_DIVE 鎖到 −30°；沒接雲台就不出這張 |
| Fig 10 系統 | 電壓、控制迴路週期（20 ms = 50 Hz）、nz | Orin 有沒有卡頓、電池 |
| Fig 11 真車 GPS | 同一時刻虛擬目標 vs 真車的位置差、車速 vs 10 m/s | 車子何時出發 / 停的位置、`TARGET_SPEED_MPS`、撥 GUIDED 時機 |
| Fig 12 雲台當控制輸入 | 把雲台量到的目標轉到虛擬相機，離線丟進同一套 DivePID，算出「用雲台資料控制會下的命令」跟實際命令比較（DIVE 有雲台資料才有） | 見下面第 5.1 節 |

---

## 5. summary.txt 的自動檢查（門檻）

| 檢查 | ⚠ 的條件 | 提示的方向 |
|---|---|---|
| 撥 GUIDED 高度 | 跟 240 m 差 > 20 m（AGL < 20 m 視為地面測試，不檢查） | Mission Planner 航點高度 |
| 推頭前航向誤差 | 推頭前 2 s 平均 > 5° | 撥 GUIDED 太晚 / `GUIDED_P` |
| 推頭用時 | PRE_DIVE > 4 s | 飛控 pitch 角速度上限 / pitch 增益、`PREDIVE_HANDOFF_PITCH_TOL_DEG` |
| 俯衝角 | DIVE 平均 γ 跟 −30° 差 > 3° | pitch 追蹤、`ALPHA_DIVE_DEG` |
| pitch 追蹤 | DIVE RMSE > 3° | 飛控 pitch 迴路 |
| 空速 | ≥ 55 m/s 提醒；到 60 = 超速提前拉平 | 俯衝角、`THR_MIN`、撥 GUIDED 高度 |
| 影像誤差 | 擺盪（過零 > 0.8 Hz 且 RMSE > 1°）/ 後半段一直偏 > 1.5° / RMSE > 5° | 擺盪 → `KP` 太大；一直偏 → `KI` 不夠或方向舵 / pitch 不夠力 |
| 方向舵 | 打滿 > 20 % 時間；被 roll 保護限制 > 10 % | `K_LAT_TO_RUDDER`、`ROLL_PROT_*` |
| roll | DIVE 中 > 25° | 方向舵–滾轉耦合 |
| 拉平 | 最低 AGL < 100 m（✗）；比 120 m 多掉 > 15 m（⚠） | `PULLOUT_AGL_M` |
| 控制迴路 | > 40 ms 的拍數 > 2 %，或單次 > 200 ms | Orin 負載 |
| 電壓 | 讀到 ≈ 0 V 超過 5 % | 第二顆電池監測沒接 |
| 雲台 | UART 連線 < 95 %；PRE_DIVE 後一直沒鎖定到位 | 線材、對方程式 |

這些只是**根據 log 的提示**，判斷前還是要看圖。數字和圖要一起看，例如「影像誤差一直偏」要再看 Fig 6 方向舵是不是已經打滿。

### 5.1 Fig 12：雲台資料能不能當控制命令

**做法**
1. 雲台模型：pitch / roll **相對地面**（自穩），yaw 跟機頭（沒有 yaw 軸）。
2. 每一筆雲台偵測：用「取像那一刻」（`vis_cap_age_ms`）的飛機航向 / pitch，把目標方向轉到 DIVE 用的虛擬相機（機身固定、光軸 = 機頭），得到虛擬畫面上的 (u, v)。
3. 用同一套 `dive_guidance.DivePID`、同樣的增益和 setpoint，**離線重跑** DIVE 段，算出 pitch 命令和方向舵（= `K_LAT_TO_RUDDER` × dpsi）。
4. 同時用「虛擬量測」再重跑一次當**重放檢查**：這條應該跟實際命令完全重合（差 < 0.5°），代表重放是對的。
5. 有車子 BIN 時，再用真車 GPS 算「車子理論上在虛擬畫面的哪裡」（綠虛線），把差異拆成兩部分：
   - **雲台 − 真車**＝雲台本身量得準不準（偏差、跳動、延遲、座標假設）
   - **真車 − 虛擬目標**＝車子沒有開在虛擬軌跡上（跟 Fig 11 同一件事）

**圖（由上到下）**：橫向偏移、縱向偏移（藍＝虛擬，橘點＝雲台，綠虛線＝真車）、pitch 命令、方向舵命令、雲台偵測與影像延遲。

**summary.txt 的【雲台資料當控制命令】判斷**
| 項目 | ✓ 條件 |
|---|---|
| 重放檢查 | 虛擬量測重放 vs 實際：pitch < 0.5°、方向舵 < 1° |
| 偵測率 | DIVE ≥ 90 % |
| 影像延遲 | 中位數 ≤ 150 ms |
| 雲台 vs 真車 偏差 | 平均 ≤ 1°（有偏差 → 雲台零點 / 安裝角；縱向偏差可用 `--gimbal-pitch-offset <偏差值>` 重跑確認） |
| 雲台 vs 真車 跳動 | RMSE ≤ 2° |

⚠ **命令差是開環重放**：飛機實際是照虛擬目標飛，沒有照雲台命令飛，所以誤差不會被修掉、積分會一直累積。只看「一開始往哪個方向修、修多少」，不要把命令的 RMSE 當成控制效果。真正的控制效果要等實際用雲台閉迴路飛過才知道。

沒有給車子 BIN 時，無法分辨「雲台不準」還是「車子不在虛擬位置」，summary 會提醒加 `--car-bin-dir`。

選項：`--gimbal-pitch-offset`（pitch 零點修正，度）、`--gimbal-pitch-sign`、`--gimbal-roll-sign`（預設 1，往下為負）。

不用飛機先試：`python3 test_plot_report.py --vision`（假雲台看真車，車子比虛擬目標晚 4 s 出發）。

---

## 6. 常見狀況

- **Orin 時鐘是 1970**：檔名會是 `19700101_...`，但不影響畫圖。對真車用的是飛控的 GPS 時間（`gps_unix` 欄位），不是 Orin 的時鐘。所以每趟請用不同的 `--case`，才分得出是哪一趟。
- **Fig 11 沒畫**：`summary.txt` 會寫原因，常見的是：
  - BIN 沒有涵蓋 PRE_DIVE~DIVE 的時間（拿錯天、車子飛控沒錄到）；
  - 資料夾裡有兩個 BIN 都涵蓋（只留當天那個）；
  - 這趟沒有進 DIVE。
- **Fig 11 的「北向差」**：目標往南開，北向差為正 = 車子在虛擬目標**後面**（車子出發晚了）。
- **只想單獨比對真車**：
  ```bash
  python3 xwing_plot_target_gps.py flight_logs/<log>.csv --bin-dir car_bin
  ```

# 雲台視覺 UART 整合說明 (xwing_virtual)

> 對象：本實驗室 (飛控 / 任務程式) 與 對方實驗室 (雲台相機 + 辨識 Orin NX)。
> 協定來源：對方提供的 `uart_mipi_receiver.py` (UART v6 / v7，big-endian，CRC-16/CCITT-FALSE)。
>
> **目前狀態：只接收、記錄，不進飛機控制。** 控制仍用 GPS 虛擬目標，視覺資料先拿來比對與驗證。

---

## 1. 系統架構

```
┌──────────────── 對方實驗室 ────────────────┐        ┌──────────────── 本實驗室 ─────────────────┐
│                                            │        │                                           │
│  [雲台相機] ─MIPI─▶ [Orin NX (對方)]        │  UART  │  [USB-UART 模組] ─USB─▶ [Orin NX (本機)]  │
│                     · 目標辨識              │ ─────▶ │                         · xwing_virtual_   │
│                     · 雲台控制 (追蹤/鎖定)  │ 115200 │                           flight.py        │
│                     · 打包 v7 遙測 (~83 B)  │ ◀───── │                         · MAVROS ──▶ 飛控 │
│                                            │        │                                           │
└────────────────────────────────────────────┘        └───────────────────────────────────────────┘
       對方 → 本機：目標像素、偏移、內參、雲台 pitch/roll、模式、時間戳 (v7)
       本機 → 對方：模式鍵 '0'/'1'/'2' (ASCII 1 byte)、v7 校時請求
```

---

## 2. 硬體接線與系統設定 (本機 Orin NX)

### 2.1 接線

| 對方 UART | USB-UART 模組 | 說明 |
|---|---|---|
| TX | **RX** | 交叉 |
| RX | **TX** | 交叉 (回傳模式鍵與校時請求要用) |
| GND | GND | **一定要共地** |
| VCC | **不要接** | 兩邊各自供電 |

- USB-UART 模組要選 **3.3 V TTL** 電位 (Orin NX UART 是 3.3 V)。**不要用 RS-232 電位的模組**，會燒掉腳位。
- 鮑率 115200 8N1，無流量控制 (與對方程式預設相同)。

### 2.2 固定裝置名稱 (重要)

`/dev/ttyUSB0`、`/dev/ttyUSB1` 的編號開機時可能互換。若飛控也走 USB 串口，可能抓錯裝置。
建議改用固定路徑：

```bash
ls -l /dev/serial/by-id/
# 例: usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0 -> ../../ttyUSB0
```

把這個路徑填到 `xwing_virtual_config.py` 的 `VISION_UART_PORT`，或執行時用 `--vision-port` 指定。

### 2.3 權限與套件

```bash
sudo usermod -aG dialout $USER     # 串口權限, 重新登入後生效
pip3 install pyserial
python3 -m serial.tools.list_ports -v
```

### 2.4 先用對方的接收程式確認連線

```bash
python3 uart_mipi_receiver.py --port /dev/ttyUSB0
```

看到 `CRC=OK`、欄位有數值，就代表接線與鮑率正確。按 `0`/`1`/`2` 可以測試切模式。

---

## 3. 收到的資料 (對方 → 本機)

| # | 欄位 | 單位 | 說明 |
|---|---|---|---|
| 0–1 | target_cx, target_cy | px | 目標中心；無目標時為 NaN |
| 2–3 | offset_x, offset_y | px | 目標相對光學中心的偏移，**右 / 下為正** |
| 4–7 | fx, fy, cx, cy | px | 相機內參 (隨變焦改變，每筆都送) |
| 8–9 | image_width, image_height | px | 影像大小 |
| 10–11 | pitch, roll | deg | 雲台角度 (⚠ **沒有 yaw**) |
| 12 | mode | — | 0 手動、1 追蹤、2 鎖定 |
| 13 | result_id | — | 辨識結果編號 (換新結果才會變) |
| 14–15 | detection_age_ms, angle_age_ms | ms | 送出時辨識結果 / 角度資料的年齡 |
| v7 | capture_ns, session | — | 取像時間戳 (對方時鐘) + 校時 session |

- v7 封包 83 bytes，115200 鮑率下線上傳輸約 7.2 ms。30 Hz 時約佔頻寬 22%，足夠。
- 本機用 v7 校時把「取像時刻」換到本機時間軸，可量出 **取像 → 本機收到** 的延遲 (`vis_e2e_ms`)。

---

## 4. ★ 要不要回傳任務狀態給對方雲台？— 要

### 4.1 為什麼要

原版 (`gimbal.py`，自己控 SIYI 雲台) 依任務階段切雲台模式：

| 任務階段 | 原版雲台行為 | 理由 |
|---|---|---|
| 還沒撥 GUIDED (STANDBY) | FOLLOW，yaw=0 跟機頭，固定低頭 | 對準時目標就在畫面中央 |
| SEARCH / PRE_DIVE | FOLLOW + **追蹤目標** | 飛機在轉彎 / 推頭時，雲台補償讓目標留在畫面內 |
| **DIVE** | **FPV (相機固定在機身上)** | DIVE 的 PID 要的是「目標相對**機頭**偏多少」 |
| PULLOUT / DONE | — | — |

**關鍵在 DIVE**：如果 DIVE 時雲台還在追蹤，雲台會一直把目標拉回畫面中央，影像誤差永遠約等於 0。
這時 PID 會以為已經對準，不會修正。所以要嘛雲台切成**機身固定**，要嘛要拿到雲台的完整角度
(pitch **和 yaw**) 自己換算回機身座標。

對方的雲台由對方控制，它不知道飛機現在在哪一個階段，**所以一定要由我們告訴它。**

### 4.2 對方現有協定的兩個缺口

| 缺口 | 影響 | 需要對方 |
|---|---|---|
| 模式只有 0 手動 / 1 追蹤 / 2 鎖定，**沒有 FPV (機身固定)** | DIVE 沒有對應的模式可以切 | 新增 FPV 模式 (例如鍵 `'3'`)，或確認「鎖定」是否就是機身固定 |
| 遙測只有 pitch / roll，**沒有 yaw** | 雲台左右追蹤時，無法換算目標相對機頭的方位；也無法確認 FPV 時 yaw 是否歸零 | 遙測加入雲台 yaw (相對機頭) |

### 4.3 兩種做法

| | **做法 A：我們直接送模式鍵** (現在就能用) | **做法 B：我們送任務狀態，對方決定雲台模式** (建議長期採用) |
|---|---|---|
| 送什麼 | ASCII `'0'`/`'1'`/`'2'` (1 byte) | 有 CRC 的封包：任務狀態 + 建議模式 |
| 對方要改 | 不用改 (除非新增 FPV 鍵) | 新增一種封包類型 |
| 優點 | 現在就能測 | 有 CRC 不怕雜訊；責任清楚 (雲台怎麼動由對方決定)；對方可以記錄我們的任務狀態 |
| 缺點 | 單一 byte 沒有檢查碼，線路雜訊可能誤切模式 | 要等對方實作 |
| 本專案 | **已實作** (`--gimbal-cmd`，預設關閉) | 本文提出格式，雙方確認後再實作 |

#### 做法 A：目前的對應 (`xwing_virtual_config.py` 的 `GIMBAL_MODE_KEYS`)

| 我們的狀態 | 送出的鍵 | 對方模式 | 備註 |
|---|---|---|---|
| STANDBY (未撥 GUIDED) | 不送 | (對方自行決定) | 原版是 FOLLOW 看前方 |
| SEARCH | `'1'` | 追蹤 | |
| PRE_DIVE | `'1'` | 追蹤 | 與 SEARCH 相同鍵，不會重送 |
| DIVE | **不送** | — | ★ 待對方新增 FPV 鍵後填入 |
| PULLOUT / DONE / ABORT | 不送 | — | |

- 狀態一改變就送一次。對方遙測回報的 `mode` 欄位一致才算確認，沒確認就每 1 s 重送，最多 3 次，之後印出警告。
- 每次切換都寫進 `events.txt`。主 CSV 有 `gimbal_state`、`gimbal_mode_req`、`gimbal_mode_ok`、`vis_gimbal_mode` 可以對照。

#### 做法 B：建議的任務狀態封包 (待雙方確認)

沿用對方的封包框架 (magic `A5 5A`、version 7、LENGTH / LENGTH_XOR、CRC-16/CCITT-FALSE)，新增 **flags = 3**：

| 位元組 | 欄位 | 型別 | 值 |
|---|---|---|---|
| 0 | mission_state | uint8 | 0 STANDBY、1 SEARCH、2 PRE_DIVE、3 DIVE、4 PULLOUT、5 DONE、6 ABORT |
| 1 | gimbal_request | uint8 | 0 手動、1 追蹤、2 鎖定、3 FPV、255 不要求 |
| 2–3 | seq | uint16 | 每送一筆 +1 |
| 4–7 | t_ms | uint32 | 本機 monotonic 毫秒 (記錄對齊用) |

- payload 8 bytes，整包 18 bytes。狀態改變時立刻送，平常每 0.5 s 重送一次 (keepalive)。
- 對方超過 1 s 沒收到時：維持目前模式 (或回到安全模式，由對方決定)。
- ⚠ 必須等對方接收端改成「先找 `A5 5A` 封包框架」再送。舊版若逐 byte 找 `'0'`/`'1'`/`'2'`，可能把封包內容誤判成模式鍵。

### 4.4 DIVE 切 FPV 的時機

雲台從「追蹤」轉到「機身固定」需要時間 (轉動 + 穩定)。建議：

- 在 **進 DIVE 那一刻** 送 FPV。此時飛機已經壓到約 −27°，目標在機身座標下接近畫面中央，切過去不會丟目標。
- 對方回報切換完成要多久，我們就把 `GIMBAL_FPV_SETTLE_S` 設成那個時間。這段期間 `dive_guidance` 會先維持 −30°，不讓 PID 吃還在轉動的畫面。
  (目前純虛擬版設成 0；原版 SIYI 是 0.3 s。)

---

## 5. 要跟對方確認的問題

| # | 問題 | 為什麼要知道 |
|---|---|---|
| 1 | 「1 追蹤」「2 鎖定」的確切行為是什麼？「鎖定」是鎖定目標、鎖定地面方向，還是固定在機身上？ | 決定 SEARCH / DIVE 該送哪個鍵 |
| 2 | 可以新增 **FPV (機身固定，相機光軸對齊機頭)** 模式嗎？鍵要用什麼？ | DIVE 必要 |
| 3 | FPV 切換完成要多久？ | 設定 `GIMBAL_FPV_SETTLE_S` |
| 4 | 遙測能加入**雲台 yaw** (相對機頭) 嗎？ | 追蹤模式下換算機身座標 |
| 5 | pitch / roll 是相對**水平面**還是相對**機身**？ | 換算視線角 (SIYI A8 mini 的 pitch 相對水平面) |
| 6 | `offset_x/y` 是否就是 `target − (cx, cy)`？正負號是右 / 下為正？ | 與我們 PID 的符號一致 (我們是右 / 下為正) |
| 7 | 遙測頻率多少 Hz？沒有目標時還會持續送嗎？ | 判斷資料鏈中斷 |
| 8 | 雲台安裝方向：yaw=0、pitch=0 時，光軸是否對齊機頭、平行機身縱軸？ | FPV 時的安裝角補償 |
| 9 | 是否願意採用 4.3 的任務狀態封包 (做法 B)？ | 長期介面 |
| 10 | 對方 Orin NX 與相機的電源、開機時間、斷電後是否自動重啟發送 | 實飛流程 |

---

## 6. 本專案的變動

| 檔案 | 狀態 | 內容 |
|---|---|---|
| `uart_mipi_receiver.py` | **新增 (對方提供，未修改)** | 協定定義 + 對方的獨立接收測試程式。本專案直接 import 它的解碼與校時，對方改協定時換掉這支即可 |
| `xwing_uart_vision.py` | **新增** | 背景執行緒收 UART：重新同步、CRC、解碼、v7 校時與延遲、送模式鍵並確認、USB 拔插自動重開 |
| `xwing_uart_sender_sim.py` | **新增** | 模擬對方 Orin NX 發送端 (v7 遙測 + 校時回覆 + 模式鍵)。對方硬體到之前就能測 |
| `xwing_virtual_config.py` | 修改 | 新增 `VISION_UART_*`、`GIMBAL_MODE_*` 參數 (預設全部關閉) |
| `xwing_virtual_flight.py` | 修改 | 新增 `--vision-uart`、`--vision-port`、`--gimbal-cmd`；依任務狀態送模式鍵；視覺資料寫入主 CSV；資料鏈中斷告警 |

**飛機控制完全沒變**：視覺資料只寫進 log。沒開 `--vision-uart` 時，行為與之前相同。

### 6.1 設計原則

- **UART 在獨立執行緒**：讀串口、等資料都不會卡住 50 Hz 控制迴圈。實測加上視覺後，每拍控制耗時中位數仍約 0.1 ms。
- **視覺斷線不影響飛行**：串口錯誤會自動關閉、每秒重試，主程式只會在 events.txt 記一筆「資料鏈中斷」。
- **沒裝 pyserial 也能飛**：開了 `--vision-uart` 但缺套件時，只印警告，然後繼續用虛擬目標飛。

---

## 7. 使用方式

### 7.1 不需要任何硬體：虛擬串口測試

```bash
# Terminal 1: 模擬對方 Orin NX
python3 xwing_uart_sender_sim.py --pty
#   → 印出「接收端請用 /dev/pts/N」

# Terminal 2: 用對方的接收程式驗證 (可按 0/1/2 切模式)
python3 uart_mipi_receiver.py --port /dev/pts/N
```

### 7.2 兩個 USB-UART 模組對接 (TX↔RX 交叉、GND 共接)

```bash
python3 xwing_uart_sender_sim.py --port /dev/ttyUSB1      # 模擬對方
python3 uart_mipi_receiver.py --port /dev/ttyUSB0         # 本機接收
```

### 7.3 實飛 / 掛機

```bash
roslaunch mavros apm.launch

# 只接收 + 記錄 (不送模式鍵)
python3 xwing_virtual_flight.py --moving --vision-uart

# 指定串口 + 依任務狀態送模式鍵
python3 xwing_virtual_flight.py --moving --vision-uart \
        --vision-port /dev/serial/by-id/usb-XXXX --gimbal-cmd
```

開機會印出：

```
[xwing] 雲台視覺 UART: /dev/ttyUSB0 @ 115200 (只記錄); 模式鍵 ON
[UARTVision] 已開啟 /dev/ttyUSB0 @ 115200 8N1
```

每秒狀態列的最後一段：

```
... | VIS link=1 det=1 gmode=1(req 1)
       │      │      └ 對方回報模式 (括號內: 我們要求的鍵)
       │      └ 1 = 這筆有目標
       └ 1 = 近 2 s 內有收到有效封包
```

---

## 8. 主 CSV 新增欄位

| 欄位 | 說明 |
|---|---|
| `vis_link` | 1 = 串口開著且 2 s 內有有效封包 |
| `vis_ver` | 封包版本 6 / 7 |
| `vis_rx_age_ms` | 最新封包是多久前收到的 |
| `vis_result_id`、`vis_new` | 辨識結果編號；`vis_new=1` 表示這拍拿到新結果 (50 Hz 控制比 ~30 Hz 遙測快，會重複) |
| `vis_valid` | 1 = 有目標且數值有效 |
| `vis_tgt_cx/cy`、`vis_off_x/y` | 對方原始像素值 |
| `vis_fx/fy/cx/cy`、`vis_img_w/h` | 對方送來的內參 |
| `vis_gimbal_pitch/roll`、`vis_gimbal_mode` | 雲台角度、對方回報的模式 |
| `vis_det_age_ms`、`vis_angle_age_ms` | 對方送出時資料的年齡 |
| `vis_e2e_ms`、`vis_e2e_err_ms` | v7：取像 → 本機收到的延遲與校時誤差 |
| `vis_cap_age_ms` | v7：控制這一拍時，影像已經多舊 |
| `vis_err_u_deg`、`vis_err_v_deg` | 影像角度誤差 atan(offset / f)，右 / 下為正 |
| `vis_u_ctrl`、`vis_v_ctrl` | 換算到虛擬相機像素，**可直接跟 `virt_bbox_u/v` 比**，看真實影像與虛擬目標差多少 |
| `gimbal_state` | 給雲台的任務狀態 (STANDBY / SEARCH / PRE_DIVE / DIVE …) |
| `gimbal_mode_req`、`gimbal_mode_ok` | 我們要求的模式鍵；對方是否已回報同一模式 |
| `vis_frames`、`vis_crc_err` | 累計有效封包數、CRC 錯誤數 |

---

## 9. 下一階段：把視覺接進控制 (尚未實作)

1. 對方提供 FPV 模式，並確認第 5 節的問題。
2. 實飛幾趟「只記錄」，比對 DIVE 段的 `vis_u_ctrl/v_ctrl` 與 `virt_bbox_u/v`，確認符號、延遲 (`vis_cap_age_ms`) 和雜訊。
3. 先做**影子 PID**：用真實影像算一套 PID 命令，只記錄不送 (原版 `realvision_pid_logger.py` 的做法)，與虛擬 PID 比較。
4. 驗證後才切換 DIVE 的 PID 輸入來源：
   - `vis_u_ctrl/v_ctrl` 取代虛擬投影；
   - 設定 `GIMBAL_FPV_SETTLE_S`；
   - 定義丟失目標的處理：例如超過 `ABORT_TARGET_LOST_S` 就中止拉起，或暫時退回虛擬目標。

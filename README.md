# 雲台補償與真實影像 PID 同步記錄版

將壓縮檔內全部 9 個 .py 檔放進原 vision_dive 資料夾，覆蓋同名檔案。新增模組必須一併放入。其他檔案使用現有版本。

啟動指令不變：

```bash
python3 run_vision_flight.py --static
python3 run_vision_flight.py --moving
```

## 三套結果

- 原 pitch_cmd / heading_cmd / pid_*：虛擬目標實際控制。
- real_pitch_cmd / real_heading_cmd / real_pid_*：真實 YOLO 未補償的獨立 PID，只記錄。
- real_comp_pitch_cmd / real_comp_heading_cmd / real_comp_pid_*：真實 YOLO 經雲台視線補償的另一個獨立 PID，只記錄。

三個 PID 狀態互不共用。補償前後命令差異也包含各自的濾波、積分與命令歷程，不只是當拍幾何偏差。

## 如何補償

YOLO 像素用真實相機內參轉為視線。依取像附近的雲台與飛機角度算出實際相機旋轉，將視線轉到 NED，再轉到虛擬相機座標並投影。補償後座標使用虛擬倍率的焦距及同拍虛擬 setpoint，送進原 DivePID。不是將雲台角度直接加在最後命令。

目前虛擬投影的參考矩陣使用 pitch、地速方向 course，roll=0。補償刻意遵守這個現有模型，沒有更改虛擬控制。真實相機旋轉仍包含雲台 roll；這也包含實際相機與 roll=0 虛擬參考的差異。它不是全三維機體姿態投影的虛擬模型。

## 角度模型與設定

GCOMP_ENABLED = True 開啟補償記錄。

GCOMP_ATTITUDE_FRAME 預設為 'earth_pitch_roll_body_yaw'：沿用原專案假設，雲台 pitch/roll 相對水平面，yaw 是相對機體的偏移。yaw 世界方向近似為 IMU 飛機 yaw 加雲台 yaw 偏移。採用 yaw → roll → pitch 軸順序，矩陣為 Ry @ Rx @ Rz，再乘相機軸排列矩陣。

這是可設定的模型，不是已確認此架雲台韌體的量測定義。原 gimbal.py 與 ray_projection_distance.py 的註解提供水平 pitch 與相對 yaw 的依據；SDK 協議列出了 yaw/roll/pitch 順序，但無法僅靠現有檔案確認實機所有角度語意。pitch/roll 若實際全部相對機體，將設定切為 'body_relative'。兩者無法自動辨識。

沿用 SIYI_YAW_SIGN、SIYI_PITCH_SIGN、GIMBAL_PITCH_OFFSET_DEG，新增 GCOMP_ROLL_SIGN、GCOMP_ROLL_OFFSET_DEG、GCOMP_YAW_OFFSET_DEG。初始 roll sign=1、offset=0，尚未校正。

地面確認方式：固定機身，左右與上下轉雲台，確認像素轉換方向；固定目標與機身位置、只旋轉雲台，補償後目標方向應維持不變；再改變機身 pitch/roll，確認水平角／機體角模型選擇。若要和虛擬像素完全一致，機鼻 yaw 與 course 也需要分開核對。

## 取像時間配對

siyi_sdk.py 在每次實際收到 0x0D 姿態資料時保存單調接收時間與角度，不用反覆讀取舊角度冒充新量測。飛機 IMU callback 同樣保存 pitch/roll/yaw 與當時最新速度推算的 course。

在視覺幀處理結束後，用 cap_mono - CAPTURE_LATENCY_MS/1000 配對歷史。可用前後樣本時採角度插值，並處理 ±180° 換邊；否則使用足夠近的樣本，且記錄 nearest。最大距離 GCOMP_MAX_SYNC_GAP_S = 0.15。沒有可信時間樣本時不補零，補償 PID 保持命令並凍結積分。

SDK 無感測器量測時間戳，相機 cap_mono 亦是軟體擷取時間，不能宣稱是精確曝光同步。GCOMP_GIMBAL_RX_DELAY_S / GCOMP_POSE_RX_DELAY_S 預設 0；設定接收時間減量測時間的延遲後，歷史查詢會使用曝光估計時刻加上該延遲。IMU/course 異步與硬體內部延遲仍存在。nearest/interpolated 是接收時間軸的配對方式。

## CSV 欄位

主 CSV 保留全部原欄位，新增補償前與補償後 PID 命令及各項。real_comp_pid_updated=1 才代表該拍實際更新，real_comp_pid_detected=1 才表示有效輸入。DIVE 等待期間尚未更新 PID，離開 DIVE 命令留空。超過 REALVISION_PID_MAX_AGE_S=0.5 的影像視為失效。

主 CSV 與視覺 CSV 都新增：

| 欄位 | 用途 |
|---|---|
| gcomp_valid / gcomp_reason | 幾何轉換是否成功與失效原因 |
| gcomp_pose_method / gcomp_gimbal_method | 插值或最近樣本配對 |
| gcomp_pose_gap_ms / gcomp_gimbal_gap_ms | 配對時間距離 |
| gcomp_gimbal_yaw/pitch/roll | 配對後的 SDK 原始角度 |
| gcomp_ac_yaw/pitch/roll/course | 配對後的飛機角度 |
| gcomp_u/v | 補償後的虛擬座標系像素 |
| gcomp_u_err_deg / gcomp_v_err_deg | 主 CSV 同拍 setpoint 的補償後幾何誤差，未經 PID 位置濾波 |
| real_comp_pid_u_err_deg / real_comp_pid_v_err_deg | 實際補償 PID 的濾波後角度誤差，僅在主 CSV |

gcomp_u_err_deg / v_err_deg 在視覺 CSV 留空，因為對應的主控制 setpoint 必須在控制週期才確定。影像漏偵或補償失效時 PID 誤差留空，不代表誤差為零。未補償與補償使用各自的偵測有效旗標。

## 檔案變更

run_vision_flight.py：記錄取像姿態、傳遞原子視覺資料包、兩套真實 PID 與 CSV。
config.py：補償模型、角度符號與時間配對設定。
siyi_sdk.py：接收姿態歷史。
gimbal.py：查詢時間配對姿態。
attitude_time_history.py / gimbal_pid_compensation.py：新增同步與幾何模組。
realvision_pid_logger.py / dive_guidance.py：沿用前一版。
attitude_bridge.py：與你提供的最新版相同，沒有修改。

圖五繪圖程式尚未修改，仍讀舊的離線重放結果。要畫本次即時結果，需使用 real_pitch_cmd 與 real_comp_pitch_cmd 等新增欄位。

## 驗證

Python 語法檢查；相機與參考一致的倍率換算；已知 pitch 2° / 合計6° 與 yaw 符號測試；80 組含 roll 的射線往返測試；角度跨 ±180° 與過期歷史拒絕；100 拍雙 PID 同步等價；缺失補償姿態時保持命令。尚未在 ROS、相機、雲台或飛控實機執行，也未驗證實機角度模型。

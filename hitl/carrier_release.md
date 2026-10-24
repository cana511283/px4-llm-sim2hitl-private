# 母機攜帶與水平釋放：第一階段規格

使用者於 2026-10-21 選定：無風、100 m 離地高度、30 節（1852/3600 換算，15.433333 m/s）、水平釋放。最終任務是標準五邊航線後降落；本階段只驗證釋放與接手，**沒有五邊導航或落地驗證**。

## 模擬接法

- 模型保持原 Simulink 生成碼不變。在 plant 包裝層加入朝北、零姿態／角速度的運動約束。
- 從地面、零速度開始。10 s 地面等待 → 40 s quintic 平順爬升與加速 → 10 s 定高定速攜帶 → 自由飛行。速度與高度的一、二階導數在段落邊界連續。
- 釋放時繼承位置、速度、姿態、角速度、致動器／濾波器／限速器、模型與感測器時間。外部約束解除時加速度可以改變，這是支承力消失的效果，不強行平滑自由飛行物理。
- 攜帶期間 IMU 比力由規定的運動加速度減重力得到；PlantTruth 中氣動／推力／地面力分項不含母機約束反力，不能在這一段直接把它們相加當總外力。反力等於質量乘規定加速度減自由力總和，角向反力矩為負的自由力矩。
- 第一版不含母機尾流、分離碰撞、釋放衝量、風、傾斜或旋轉。不能把本次結果當真實母機分離驗證。
- 空投情境的差壓量測使用固定種子的 0.1 Pa 標準差白雜訊（全程含釋放後），只作用於感測器輸出，不改真實速度或運動約束。原本一般飛行的零差壓雜訊預設不變。這是數值模擬設定，尚未由實體空速計量測校準。新增雜訊會消耗該感測器實例的亂數序列，因此同次 GPS／IMU 雜訊樣本也會不同，但重設同種子仍可重現。

### 2026-10-21 首次空投測試中止原因

`outputs/carrier-hitl-20261021-01` 在釋放前中止：最後 565 筆 HIL_SENSOR 的差壓 float 均為 1.444945693 hPa，持續 2.256 s。PX4 原始空速仍有限值，隨後有效空速來源變成 -1，VFR_HUD 回報 NaN，自動流程停止並完成 recover。此結果不是空投成功。

鎖定版本 PX4 的 `src/modules/airspeed_selector/AirspeedValidator.cpp::check_airspeed_data_stuck` 將飛行中超過兩秒不變的 IAS 視為卡值；更底層 `sensors.cpp`／`DataValidator.cpp` 也檢查重複差壓值。原先沒有差壓雜訊，而母機恰好定高定速，使這兩層檢查面對理想、完全不變的輸入。修正在感測器模型，不停用 ASPD_DO_CHECKS、不用地速代替空速，不更改故障鎖定／自動恢復參數。

`validate_carrier_abi.py` 增加真實模型／感測器 ABI 的十秒定速段測試，包含 hPa float 量化、約 50 ms 平均後 IAS 的兩秒卡值條件，以及同種子逐位元重現。離線檢查不能代替 FC 實測；失敗後重新測試須經使用者同意，使用新輸出資料夾保留首次失敗證據。

## SITL

`sitl/scenarios/carrier_release_stabilized.json` 用既有真值／arming stub 的精簡 SITL，不驗證實體解鎖或 EKF。獨立 build suffix `_carrier_trial` 保留原 baseline 執行檔。

釋放前搖桿與油門為零；釋放後進入對應 30 節平飛配平俯仰的 Stabilized 設定值與配平油門，觀察 40 s。這是姿態穩定測試，沒有高度保持。水平釋放不等於配平平飛：原模型需要約 3.44° 俯仰／攻角和對應的舵面、油門，釋放瞬間沒有偷偷套用配平致動器。

## HITL

`hitl/scenarios/carrier_release.json` 仍是地面初始條件。`HilCarrierConfig` 是選用 ABI 擴充（capability 512），舊有 struct 大小不變。狀態 `model_truth.carrier_phase`：0 關閉、1 等待真實 armed、2 攜帶、3 已釋放。未收到 armed 前模型／感測器時間繼續走，但攜帶時鐘停在 0，沒有位移。失去 armed 確認時不可繼續。

`./runbridge carrier-run --scenario hitl/scenarios/carrier_release.json --release deploy/01_sdfix_cubeorangeplus --out <全新資料夾>`：只用已授權且核對過的 owner 通訊；驗證 FC、參數、runtime 身分 → 啟動地面感測器 → 正常切 Stabilized／解鎖（不 force arm）→ 攜帶／釋放 → 觀察 40 s → 停模型 → 標準 recover → 核對參數／新 log。

命令被拒絕、遙測過期、非預期模式／解鎖變化、時序晚到、USB 錯誤，都停止並留證據；沒有自動重試。QGC 操桿介入會中止自動測試並恢復。模型停止不等於恢復，必須檢查 recover 的最終狀態。

## 驗證與判讀

- `plant/core/validate_carrier.py`：有限差分檢查速度／加速度、軌跡段落連續、無效輸入拒絕、非零致動器的釋放連續性，以及釋放後自由動力學恢復。
- `hitl/tests/validate_carrier_abi.py`：未知和未解鎖不能移動；實際 armed 後才計時；釋放條件、模型／GPS 時間不中斷；disarm fault；空中初始化不能用此模式。
- `runbridge update-model` 在既有檢查外執行以上兩項，仍保留舊路線檢查。
- 使用者尚未訂定飛行品質通過門檻。只列最低高度、姿態／空速範圍、尾段趨勢、接手與時序記錄；`process COMPLETE` 不代表穩定／定高／任務成功。SITL 是真值；HITL 是 FC EKF，另存模型真值，不混在同一條曲線。
- 真正五邊任務需完整導航／任務接線；落地模型仍有根 README／plant README 記載的簡化。後續另外完成，不把姿態測試當五邊／降落已通過。

## 2026-10-21 Cube Orange+ 重測紀錄

修正版模型 `skysurfer-9b41ce5b3aa4eae4bc560b8c` 完成一次真實 Cube Orange+ HITL 空投，釋放後觀察 40 s。流程 COMPLETE、標準恢復 PASS；不是飛行品質評級，也沒有執行五邊或降落。未修改額外飛控參數，結束確認未解鎖、SYS_HITL=1、沒有待恢復；模型錯拍 0。

以下數值取自 FC ULog 的 EKF 估測，不是 plant 真值：

| 項目 | 紀錄 |
|---|---|
| 首次觀察到釋放的高度（相對 EKF 原點） | 100.27 m |
| 釋放後最低／結束高度 | 81.37 / 81.40 m |
| 釋放後水平路徑長度 | 約 658.2 m |
| 滾轉角範圍 | -0.51° 至 +3.85° |
| 俯仰角範圍 | -6.73° 至 +3.27° |
| 尾段五秒平均滾轉／俯仰 | +0.01° / +3.21° |
| 末段航向 | 約 +27°；原因尚待分析 |

釋放標記以 owner 首次回報 carrier_phase=3 的時間，對齊同次 agent-wire 記錄的 ATTITUDE 開機時鐘；最近封包相差約 1.3 ms，標記仍有狀態輪詢與傳輸延遲，不能當成精確分離時刻。FC 記錄 355 比測試前最新記錄 354 新，兩次下載 SHA256 都是 `3d24a1a178f89529f8ffe23ebf5fbe609a0062abe9b217971b4fd4735dcdce49`，嚴格解析通過。完整 log／圖表留在本機 `outputs/carrier-hitl-20261021-02`，不隨原始碼上傳。

離線發布六項檢查全部 PASS，含新增十秒定速差壓卡值回歸與固定種子重現。SITL 對照使用此修正前的相同動力學：最低高度約 82.05 m、俯仰 -6.64° 至 +3.26°；SITL 為真值，HITL 為 EKF，未訂定差異驗收門檻，不能由這組數字宣稱兩者等效。

第二階段的原生 AUTO 五邊與盤旋降落已完成 SITL 預演，尚未上 Cube；它是另一個 `carrier-circuit-run` 指令，詳見 [missions README](../docs/missions/README.md)。此處第一階段的 40 s 觀察結果與範圍維持不變。

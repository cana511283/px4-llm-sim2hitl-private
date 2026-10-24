# 母機空投後的原生 PX4 五邊任務

使用者選定 PX4 原生 AUTO 任務：現有 Cube Orange+ 韌體不變，攜帶／水平釋放沿用第一階段。此版本是純模擬的北向、平地、無風情境，不是任何實際機場的飛行計畫。

`carrier_circuit.py` 是 SITL 與 HITL 共用的航點來源；`--out <目錄>` 只輸出 JSON／QGC `.plan`，不連飛控。

## 流程

地面正常解鎖 → 等待 10 s → 母機爬升加速 40 s → 100 m／30 節攜帶 10 s → 水平釋放 → Stabilized 觀察 40 s → 切 AUTO_MISSION → 五邊 → 盤旋降到 40 m → 最後進場／接地／減速停住。

五邊是先完整飛過一次，之後另接降落盤旋，因此五邊的最後進場邊本身不接地。以初始地面位置為 N/E 原點：進入點 (1400,0)、離場邊終點 (2000,0)、側風邊 (2000,-400)、順風邊 (800,-400)、底邊 (800,0)、最後進場邊 (1300,0)，高度皆 100 m；降落盤旋中心 (800,-100)、逆時針半徑 100 m、退出高度 40 m；名義接地點 (1400,0)、高度 0。原生 PX4 會切彎與沿圓切線退出，不要求逐點走折線。

選用 `DO_CHANGE_SPEED`、`NAV_WAYPOINT`、`DO_LAND_START`、`NAV_LOITER_TO_ALT`、`NAV_LAND`，不另寫導引律。已對照鎖定版本的 `navigator/mission.cpp`、`mission_block.cpp`、`MissionFeasibility/FeasibilityChecker.cpp`、`fw_mode_manager/FixedWingModeManager.cpp`、`commander/ModeUtil/control_mode.cpp`。40 m／600 m 進場約 3.81°，低於預設 `FW_LND_ANG=5°`。

## 兩項暫時參數

- `FW_THR_TRIM=0.121`：模型在約 30 節的平飛配平油門。
- `FW_AIRSPD_TRIM=15.433333 m/s`：30 節目標空速。

不關閉空速、起飛前檢查或失效保護。HITL 用現有 `param-deploy` 交易部署兩項值；測試前讀回全部參數，用原數值建立獨立還原配方，結束後還原、再逐位元比對（排除飛控自己的飛行計數）。初次使用仍須人同意。

## SITL

`SIM2HITL_SITL_SUFFIX=_circuit_trial`，先 `sitl/build_sitl.py`，再 `sitl/run_sitl.py --scenario sitl/scenarios/carrier_native_circuit.json --out <新目錄> --retries 0`。

啟動原生 dataman、Navigator、fw_mode_manager 與固定翼控制器。mission_item 與 mission 使用 PX4 原生結構／DatamanClient 寫入模擬實例的資料管理器；位置與姿態使用模型真值，Commander 的模式／arming 仍由 stub 提供。100 s 時 stub 按 PX4 ModeUtil 對應旗標進入 AUTO_MISSION。接地 stub 需高度 <0.05 m 且地速 <1 m/s；它只是讓 Navigator 完成終止流程，不能證明實體 FC 的 land detector 或 Commander 會通過。

2026-10-21 離線結果：720 s，180,000 步完整跑完，無重試；任務序號 0..9 均有紀錄，Navigator 回報任務完成、landed。ULog 約每 20 ms 的真值位置樣本中，首次接地約解鎖後 478.36 s，水平速度 10.87 m/s、向下速度 0.61 m/s，距名義接地點約 39.03 m；最終停在接地點北側約 73.40 m。數字只是紀錄，未訂飛行品質通過門檻。圖、JSON、ULog 留在本機 `outputs/carrier-circuit-stage2`。

## HITL

關閉舊 owner 後重新啟動，載入包含新增腳本的 runtime 身分。保持原來已驗證的模型 release，沒有新韌體或新機體動力學。

```sh
./runbridge carrier-circuit-run --scenario hitl/scenarios/carrier_release.json \
  --release deploy/01_sdfix_cubeorangeplus --out <全新結果目錄>
```

`carrier_test.py --circuit` 重用原本身分、正常解鎖、時序、遙測、接管與標準恢復檢查；`native_circuit.py` 負責任務／暫時參數生命週期；`mission_protocol.py` 經 owner 的 agent UDP 路徑備份、上傳、讀回、還原航點，從不直接開串列埠。備份／上傳只允許確認未解鎖、HITL、模型已停、無待恢復的同一飛控。超時／拒絕不自動重試。

正常結束需在最後任務項目、收到新鮮的已落地狀態、地速 <1 m/s 且 local 高度 <2 m。這是流程終止條件，不是著陸品質門檻；若 FC 沒有回報，最多在 AUTO 段 900 s 後中止。切模式後保留中立遙控 keepalive；QGC 操桿仍可中止。最後停模型、標準 recover，再還原航點及參數；比對兩次下載的新 FC log 並嚴格解析。

模型只有重心一個彈簧／阻尼接地點，沒有起落架、機腹、撞擊或墜毀判定。此測試只能驗證模擬導航／接地流程，不能宣稱實機安全著陸。

### 首次 HIL 準備中止（2026-10-21）

`outputs/carrier-circuit-hitl-20261021-01` 尚未啟動模型或解鎖：暫時配方建立時排除了 `.px4`，而參數交易會核對該檔的 SHA，因此 trial／restore 都在開始交易前被拒絕。前後完整參數逐位元相同、FC 身分相同，無 pending recovery／deploy；agent wire 只有航點讀取協定和 heartbeat，沒有上傳 mission count、PARAM_SET 或 arm。已改為在 trial／restore 都保留完整 firmware 目錄（仍不刷機），新增配方包含韌體的回歸測試；用實際來源配方做離線 check，restore PASS，trial 僅差預期兩項值，local_mismatch 皆空。不得把此準備失敗視為已跑 HIL；再次測試須人同意、重啟載入新 runtime、使用新結果目錄。

### 第二次 HIL 準備中止（2026-10-21）

`outputs/carrier-circuit-hitl-20261021-02` 在兩項參數部署成功後、模型啟動前，被航點讀回檢查擋下：`NAV_LOITER_TO_ALT` 的 radius 錯填 param3。PX4 `mavlink_mission.cpp` 的 parse／format 都使用 param2，所以半徑讀回為 0。已修正共享任務及 SITL Dataman 轉換，使兩者遵循相同欄位語意；測試 fixture 不再只回送輸入，另模擬此指令未使用 param3 歸零的行為。讀回經緯度比對也改用 1e-7 度整數格點，避免相對浮點容差放過公尺級位置錯誤。原航點與 restore 配方均 PASS；前後所有參數 bits 相同，航點備份與還原 JSON 相同，沒有啟動模型或解鎖。舊 SITL 直接寫 Dataman，沒有走 MAVLink，不能驗證這項傳輸欄位；修正版重新預演，未自動重跑硬體。

補充驗證：修正後用鎖定版本實際 POSIX PX4 的 mavlink＋dataman（不啟動模型、Commander，不連 Cube）完成 10 項任務的真實 MAVLink 上傳／讀回與空白備份還原，LOITER_TO_ALT param2 半徑讀回 -100 m，檢查 PASS。相關 console、wire、JSON 在本機 `outputs/carrier-circuit-stage2/px4-mission-wire-check`；不是硬體 HIL。完整 720 s SITL 也重新完成。

### 第三次 HIL：完成航線，地形量測缺失中止降落（2026-10-21）

Cube Orange+ 實際完成地面正常解鎖、100 m／30 節母機釋放、40 s 穩定觀察、原生五邊、盤旋下降與最後進場。約解鎖後 450.96 s、EKF 高度 35.67 m，PX4 從 AUTO_MISSION 轉 HOLD，事件明確記錄 `Landing aborted: terrain measurement not found`；未接地，整體流程為 FAILED，不能宣稱降落通過。

4518 個 local position 樣本的 `dist_bottom_valid` 全為 false。`FW_LND_USETER=1`、`FW_LND_ABORT=3` 的地形保護正常生效；SITL 的真值 stub 提供有效對地高度，HIL 尚未模擬對地測距，因此 SITL 完成不能證明 HIL 可完成降落。下一步待確認模擬向下測距感測器方案，尚未實作，沒有關閉地形／降落中止檢查。

標準 recover、航點還原及兩項暫時參數還原均 PASS。飛控解除解鎖時自動將 ASPD_SCALE_1 由 1.0 微調成 1.0000187158584595，導致自動證據收集最後參數核對失敗；另行交易還原至 1.0 並讀回 PASS。最終未解鎖、SYS_HITL=1、無待恢復或參數交易。飛控自行更新的校正與飛行計數另記錄，不偽裝為完全逐位元相同。

飛控新 log 356（測試前最新 355）手動下載兩次 SHA-256 相同：`880ee3713b7ea5cd830da1778213e684df2f50dcb537f11e07e37bc987d26af5`，嚴格解析 PASS；時序 misses=0。原始失敗結果保留，本次摘要、清理核對與軌跡圖存於`../../experiments/20261021-cube-circuit/`；完整 wire、ULog、參數備份保留於工作區 outputs/carrier-circuit-hitl-20261021-03。

### 平面測距 HIL：完成降落流程（2026-10-22）

新增可選 `flat_rangefinder` 場景，機體重心向下、0.05–150 m、25 Hz，沿用原有平面，原生 C writer 送 DISTANCE_SENSOR。參數 FW_LND_USETER／FW_LND_ABORT 未停用，任務高度框架仍為 GLOBAL_RELATIVE_ALT_INT。新版模型 release `skysurfer-5e8343f64ea32411a30665ec` 的模型／封包 gates PASS，300.06 s 假 PTY 負載 PASS（零漏拍，7502 筆測距）。第一個離線負載錯用 100 m 零空速自由落下，接地撞擊觸發既有加速度 envelope，獨立重現並保留證據；第二次是 localhost 權限未開而未啟動。穩定地面第三次負載才完整通過，沒有放寬保護。

Cube 初次 preflight 因 stale heartbeat／serial errors 尚未啟動；人重插後連線恢復。實際飛行結果 `outputs/carrier-circuit-flat-range-hitl-20261022-01`：正常解鎖→母機釋放→40 s 觀察→五邊→盤旋下降→最後進場→落地／減速，process COMPLETE。新 FC log 357（之前最新356）雙下載 SHA-256 均為 `a30ea19a95e8f4400c51bf74ae0e9cdaea92c6e429956fb83da699caf9b63fbc`，嚴格解析通過。

ULog 中 distance_sensor 有497筆（記錄率不等於25 Hz傳送率）；local position 4968筆中4875筆 dist_bottom_valid，最後進場632筆中604筆有效，沒有 terrain_not_found 事件。FC 約解鎖後496.73 s判定落地，當時 EKF 水平速度約0.55 m/s。高度相對EKF原點約-0.67 m，不可當成模型穿地深度；EKF原點與實際平面有偏差。最後速度約0.47 m/s。時序misses=0。尚未事先訂飛行品質門檻，不能據此宣稱實機安全著陸。

標準recover、任務還原及暫時參數還原 PASS。ASPD_SCALE_1解除解鎖後自動由1.0改為1.0000311136245728，造成原自動證據收集最後check FAIL；另建交易還原至1.0並讀回PASS。原result.json不覆寫，補充證據見`../../experiments/20261022-flat-range-circuit/`。最終armed=false、SYS_HITL=1、無pending；其餘差異皆為配方列為FC-owned的校正／飛行計數。此結果支持「本次缺少地形量測導致中止」診斷，不證明未取得日誌的舊案例原因，也不是海拔／相對高度的對照試驗。

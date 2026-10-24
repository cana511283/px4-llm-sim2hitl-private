# 母機釋放後直接盤旋下降與降落：停用測距

2026-10-22，承接來源 9ea1a967。新分支 carrier-orbit-no-range-20261022；原五邊及測距情境仍保留。

## 本次範圍

沿用 100 m／30 節攜帶釋放；釋放後立即切原生 AUTO_MISSION，盤旋下降到 40 m，切線退出後進場至相對高度 0 m 的降落航點。不飛五邊、不做 40 秒觀察或其他任務。無風。

共享航點來源：carrier_orbit_landing.py。只有速度、降落開始、LOITER_TO_ALT、LAND 四項。盤旋中心 N800/E-100 m，逆時針半徑100 m；接地點 N1400/E0 m，名義下滑角3.81度。目標是一到兩圈；LOITER_TO_ALT 依實際下降與朝向條件退出，圈數尚未量測，不能保證或宣稱達成。

## 不用測距的方式

HITL 場景 carrier_release_no_range.json 明確 flat_rangefinder=false，不啟用新增的向下測距資料產生／傳送。無須移除 PX4 導航或降落控制器。暫時參數為 FW_THR_TRIM=0.121、FW_AIRSPD_TRIM=15.433333333333334、FW_LND_USETER=0；三項都由測前完整讀回建立精確還原配方。FW_LND_ABORT 不改。

已讀鎖定 PX4 v1.17.0（d6f12ad1）fw_mode_manager_params.c 與 FixedWingModeManager::getLandingTerrainAltitudeEstimate：USETER=0 時以降落航點海拔作地面基準，地形缺失／逾時中止位元不作用。此依賴降落點高度正確，不代表有真實測地能力。

SITL 新增可選 terrain_measurement=false，將 local dist_bottom_valid 與 global terrain_alt_valid 都停用；原情境預設仍提供有效真值。SITL 使用真值與 Commander／接地 stub，不能代表 FC EKF 或 land detector 通過。

## 執行

先按專案 AGENTS／README 完成身分、未解鎖、韌體部署、單一 owner 與恢復核對。不能透過換工作目錄繞過舊 owner 的待恢復狀態。

SITL 用獨立 SIM2HITL_SITL_SUFFIX=_orbit_no_range_20261022 建置，再執行：

```sh
python3 sitl/run_sitl.py --scenario sitl/scenarios/carrier_native_orbit_no_range.json --out <新結果目錄> --retries 0 --instance-base 80
```

完成部署及 owner 檢查後，HITL runner 為：

```sh
./runbridge carrier-circuit-run --mission-profile carrier_orbit_landing --scenario hitl/scenarios/carrier_release_no_range.json --release deploy/01_sdfix_cubeorangeplus --out <新結果目錄>
```

runner 沿用航點備份／上傳讀回／還原、參數交易、標準 recover、新 ULog 歸屬及雙下載檢查；沒有增加刷機或放寬 runtime 檢查。原 runner 曾因飛控自動更新 ASPD_SCALE_1 而在最後證據核對報錯；若再次出現須保留原結果並分開處理，不能當成所有還原通過。

## 驗證結果（2026-10-22）

程式建置通過，19項離線檢查通過。第一次SITL因Codex本機socket權限而未啟動，證據保留；新回合權限恢復後，第二次SITL完成480s、120000步，Navigator完成任務。

舊owner確認無待恢復後正常shutdown；人從本機終端啟動新owner。Cube實測一趟完成空投、盤旋下降、進場、落地及減速。SITL／HITL觀測盤旋皆約1.17圈。HITL log358中 FW_LND_USETER=0、FW_LND_ABORT=3；1851筆local position的dist_bottom_valid全為false，distance_sensor零筆，無降落中止。Commander在解鎖後185.16s回報Landing detected，Navigator隨後回報Mission finished, landed。

新FC log358（測前最新357）雙下載相同，嚴格解析通過；任務與參數還原、標準recover全通過，時序零漏拍，無非預期參數變化。最終armed=false、SYS_HITL=1、無pending；bridge仍開啟、模型未運行。

結果及原始未啟動證據在 ../../experiments/20261022-orbit-no-range/；本工作區 outputs/orbit-no-range-validation/report.md 是完整比較報告，outputs/orbit-no-range-hitl-20261022-01/ 是本次硬體原始證據。SITL用真值與接地stub；HITL用FC EKF與原生落地偵測，兩者取樣間隔分別約4ms及100ms。只能確認本次模擬流程完成，著陸品質未評定。

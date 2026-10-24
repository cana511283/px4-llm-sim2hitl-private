# 00_native：原生 PX4 基準

原生 PX4 v1.17.0（[px4_lock.json](../../px4/px4_lock.json)，原始碼不改）加上 FC 參數。它是 SITL 與 HITL 比對的基準、隨時可刷回的韌體，也是第一次接手時確認整套流程跑得通的參考。

## 參數

| 檔案 | 內容 | 來源 |
|---|---|---|
| [fc_params_20261013.params](fc_params_20261013.params) | QGC 匯出的 FC 全部參數（1,208 個），原樣保存、不手改 | FC 在 2026-10-13 21:35 的狀態：PX4 v1.17.0（`d6f12ad1`）、`SYS_AUTOSTART=2100`（Generic Standard Plane）、`SYS_HITL=0`；SHA-256 `ec3cd0d049eba223ce9aeba609f1368552f7938f5294875a3337dda99ce9b38b` |
| [hitl_overrides.params](hitl_overrides.params) | 在上一個檔上加的 HITL 設定：`SYS_HITL=1`、`HIL_ACT_FUNC1..5 = 201, 202, 203, 204, 101`、`HIL_ACT_REV=2`（右副翼反向）、`SDLOG_PROFILE=17`（預設＋高頻記錄，給 ULog 重播；只影響記錄） | 2026-10-16 由 FC 的 ULog 參數讀回確認。通道對應由 `controls[0..4]` = 左副翼、右副翼、升降舵、方向舵、油門（[AGENTS](../../AGENTS.md) 第 4 節）與機架 2100 的舵面配置推得；反向由 HITL wire log 推得：飛行中 controls[0] 與 controls[1] 每一筆都相同，而 plant 兩個副翼通道都是「正值 → 左滾」 |
| [params.json](params.json) | **部署參數**：專案寫到 FC、用來核對 FC 的參數（下一節） | 2026-10-16 FC 的完整讀回與 PX4 的機架腳本（檔內 `drafted_from`）；值照 FC 現值，使用者確認過 |

- **完整參數** = `fc_params_20261013.params` 再套 `hitl_overrides.params`。這兩個 `.params` 是取得基準時的原始資料；SITL（`sitl/run_sitl.py`）與 `analysis/response.py` 仍讀它們。
- **SITL**：輸出走 `pwm_out_sim` 的 `PWM_MAIN` 群組，不是 `HIL_ACT`；`sitl/run_sitl.py` 把 `HIL_ACT_FUNC*`／`HIL_ACT_REV` 換成 `PWM_MAIN_FUNC*`／`PWM_MAIN_REV`。
- **增益**：抽查的姿態與角速度增益 9 個（`FW_R_TC`、`FW_P_TC`、`FW_RR_P/I/FF`、`FW_PR_P/I/FF`、`FW_YR_P`）等於韌體預設值；其餘參數未逐項對照。不為了 HITL 調增益，飛不好先查模型（見 [README](../../README.md)「已知問題」）。
- **排除**：感測器校正（`CAL_*`）與硬體相關的參數只屬於實體 FC，SITL 不套用。

### 部署參數 params.json

這是 **HITL 配方**（`SYS_HITL=1`），只記跟韌體預設不同的：

| 區段 | 意思 | 寫到 FC？ |
|---|---|---|
| `airframe` | 機架 2100 腳本開機時設的值（34 個） | 不寫，只核對 |
| `boot` | STM32H7 板子架構預設（`EKF2_MULTI_IMU`、`SENS_IMU_MODE`、`MC_AT_EN`、`UAVCAN_ENABLE`）與 `rcS` 在 HITL 時設的 `GPS_1_CONFIG=0` | 不寫，只核對 |
| `verify_only` | `SYS_AUTOSTART` | 不寫，只核對 |
| `set` | 專案要寫的 11 個：HITL 8 個（`SYS_HITL`、`HIL_ACT_FUNC1..5`、`HIL_ACT_REV`、`SDLOG_PROFILE`）＋`MAV_1_CONFIG`、`MAV_1_FLOW_CTRL`、`SENS_EN_THERMAL`（FC 上原有、沒有開機腳本設、來源不明，照 FC 現值保留） | **只寫這些** |
| `fc_owned` | 每台 FC 自己與其安裝的參數（校正、空速管型號、操作者遙控器的 RC 校正與通道對應、板子安裝、電源模組、裝置 ID、計數器、學到的磁偏角）：11 條規則，解析出的 499 個名字存在檔內 | 值不寫不比，但一定要讀到 |

- 其他參數一律應等於 `firmware/parameters.json.xz` 的預設值。FC 上不要手動改參數：改了，核對會在 `not_default` 列出並判 FAIL；要改就改 `params.json` 再部署。**這只影響 FC**：SITL（`sitl/run_sitl.py`）讀的是本資料夾的 `fc_params_20261013.params` 與 `hitl_overrides.params`，分析（`analysis/response.py`）預設也讀前者，不會跟著改；SITL 與 HITL 比較前，先核對兩邊實際用的相關參數，分析的 `--params` 也要對應那次測試。`not_default` 只是報告，不是寫入清單。
- **讀回**（要先開 owner：`./runbridge bridge`）：`./runbridge param-readback --release deploy/<配方> --out DIR`。PX4 的參數清單只送「有在用」的參數，沒在用的仍可用名字讀寫；完整讀回 = 清單＋依韌體的參數名單用名字補讀（程式：`hitl/host/core.py` 的 `param_readback`）。
- **核對**：`python3 deploy/check_params.py --readback <param-readback 寫出的 params_*.json> [--release deploy/<配方>]`。結果 PASS（exit 0）、FAIL（3）、INCOMPLETE（5）。也核對身分：board 56（FC 回報的 `board_version`，是 USB 產品碼；6C 剛好等於 board id，別的板子不一定，見 [fc README](../../docs/fc/README.md)）、PX4 git hash 前 10 碼、NuttX hash 前 16 碼（配方有記時）、讀回用的參數名單與本機韌體檔的 SHA-256。
- **寫到 FC**：`./runbridge param-plan --release deploy/<配方> --out DIR` 先看要寫什麼（只讀）；`./runbridge param-deploy --release deploy/<配方> --out DIR` 只寫 `set` 裡和 FC 不同的值，有寫入就等 PX4 存檔、重開 FC、再讀回核對；失敗時只改回這次自己寫的值（程式：`hitl/host/core.py` 的 `param_deploy`）。`--release` 不加時是 `deploy/00_native`；要用 FC 上現在那個韌體的配方，身分不符會被擋下。一般用 `deploy-test` 一條龍，它會自己做這一步。
- **換 PX4 版本**：`python3 deploy/check_params.py --compare-firmware 舊.json.xz 新.json.xz` 列出新增、刪除、預設值或型別或重開機標記改變的、我們管的、被 `fc_owned` 規則新吞進的；新版的開機腳本也要一起看。
- **實機飛行**（`SYS_HITL=0`）要另一份配方：`rcS` 的 HITL 分支是用 `param set` 關掉 GPS，回到實機時不會自動恢復。現況見 README「已知問題」的 3.2 出場檢查。

## 韌體

[firmware/](firmware/) 放原生 PX4 v1.17.0（原始碼不改）建好的韌體，原樣保存在 git 裡（其他配方編出的韌體不進 git，見 `.gitignore`），是隨時可刷回的基準：

| 檔案 | 內容 |
|---|---|
| [firmware.json](firmware/firmware.json) | 紀錄：board、PX4 tag／commit、產物 SHA-256、內嵌身分（`embedded`）、flash 用量、刷機紀錄（`flashed`）。`builder`、`build` 與 `flashed` 裡的路徑是舊 macOS 機器上的紀錄（當時用 PX4 的 Docker image 建置），只供追溯；這個韌體不重建，不要照那些指令做 |
| [px4_fmu-v6c_default.px4](firmware/px4_fmu-v6c_default.px4) | 建置產物原樣保存（內含 `.bin` image、`parameters.xml`、`airframes.xml`）；SHA-256 記在 `firmware.json` |
| [parameters.json.xz](firmware/parameters.json.xz) | 這個韌體定義的全部參數（2,038 個，含名稱、型別、預設值、是否要重開機） |

- **刷回**：`./runbridge deploy-test --release deploy/00_native` 先看計畫（只讀：FC 上現在是什麼、會做什麼），由人選 `--flash`（SD 卡上有開機檔就先移除 → 燒錄這個 `.px4` → 核對身分 → 參數 → `hitl-run`）。這個配方沒有 `recipe`，所以 `build-firmware` 不編它，`--flash` 直接刷保存的檔。
- **怎麼知道 FC 上是它**：FC 回報 PX4 `d6f12ad1c4`、NuttX `fb2fadf6f599c140`（`firmware.json` 的 `embedded`）。`01_sdfix` 的 PX4 hash 相同、NuttX 是 `8d3ed2a2177267cb`。`deploy-test` 的計畫會列出 FC 符合哪個配方。
- **flash 用量** 97.97 %（1,926,128／1,966,080 bytes）：加客製模組時可能要從 board 設定拿掉用不到的模組。
- **參數名單**：QGC 匯出的 1,208 個與 FC 的 ULog 記下的參數都在名單內。名單的 `rebootRequired` 有 PX4 產生器的 bug：12 個（`ADSB_*` 6 個、`IMU_GYRO_*`／`IMU_DGYRO_CUTOFF` 6 個）XML 是 false、JSON 是 true，所以「要重開機」的 270 個是保守集合（XML 為 258 個）。
- **FC 硬體**：Pixhawk 6C（`px4_fmu-v6c`）。IMU0 是 BMI055（`CAL_ACC0_ID`／`CAL_GYRO0_ID` 的 devtype 0x41／0x42），另有 ICM-42688-P。ULog 的 `ver_hw` 是 `PX4_FMU_V6C`（子型號 `V6C000001`）。
- **SITL** 用同一個 PX4 commit，見 [sitl README](../../sitl/README.md)。

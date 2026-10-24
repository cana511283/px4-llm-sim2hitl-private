# 00_native_cubeorangeplus：Cube Orange+ 的原生 PX4 基準

CubePilot Cube Orange+ 的三個配方之一（和 6C 的 `00_native`、`01_sdfix`、`02_template` 對應）。飛控差異與這次的經驗見 [fc/cubeorangeplus.md](../../docs/fc/cubeorangeplus.md)。

| 配方 | 韌體 | 和 6C 的對應 |
|---|---|---|
| `00_native_cubeorangeplus`（本資料夾） | PX4 v1.17.0 的 `hil` 板子變體：預設板子設定只多編 `pwm_out_sim`（[patch](../../px4/patches/px4_cubeorangeplus_hil_variant_pwm_out_sim.patch)），PX4 的控制程式不改 | `00_native` |
| [`01_sdfix_cubeorangeplus`](../01_sdfix_cubeorangeplus/README.md) | 上一列＋NuttX 記憶卡修正 | `01_sdfix` |
| `02_template_cubeorangeplus` | 上一列＋範例模組 `guidance_template`＋開機檔＋`hitl.json`（起飛後動作與判定和 `02_template` 相同） | `02_template` |

交接時（2026-10-20 09:10 UTC）FC 上是 `01_sdfix_cubeorangeplus`，沒有開機檔。

- **為什麼原生也要 patch**：Cube Orange+ 的預設韌體沒有 `pwm_out_sim`，`SYS_HITL=1` 時沒有 HIL 輸出、不能做 HITL。6C 的原生韌體是官方原樣；Cube 的「原生」是「PX4 程式不改＋HITL 需要的模組」。
- **韌體**：有 `recipe`，`./runbridge build-firmware --release deploy/<配方>` 從鎖定的原始碼樹重建（編好的 `.px4` 不進 git）；make 目標 `cubepilot_cubeorangeplus_hil`。
- **怎麼知道 FC 上是它**：FC 回報 board_version 4184（USB 產品碼 0x1058；韌體的 board id 是 1063）、PX4 `c45268d056`、NuttX `fb2fadf6f599c140`（`01`／`02` 是 `8d3ed2a2177267cb`；`01` 與 `02` 相同，要靠刷機紀錄與開機檔區分）。PX4 hash 是板子變體 patch 的 commit，不是 v1.17.0 本身，所以 FC 回報的版本類型是 dev。

## 參數

三個配方的 `params.json` 相同（除了名稱、說明與參數名單的 SHA-256）：

| 區段 | 內容 |
|---|---|
| `airframe`、`verify_only` | 和 6C 相同：機架 2100 與 `rc.fw_defaults` 開機時設的 34 個、`SYS_AUTOSTART=2100` |
| `boot` | STM32H7 架構預設 4 個與 rcS 在 HITL 時的 `GPS_1_CONFIG=0`（同 6C），加上 Cube 的 `rc.board_defaults` 設的 `SENS_EN_THERMAL=0` |
| `set` | 6C 的 HITL 值 8 個：`SYS_HITL=1`、`HIL_ACT_FUNC1..5 = 201, 202, 203, 204, 101`、`HIL_ACT_REV=2`、`SDLOG_PROFILE=17`。6C 才有的 `MAV_1_CONFIG`、`MAV_1_FLOW_CTRL` 不放 |
| `fc_owned` | 6C 的 11 條規則，加 `^MAN_ARM_GESTURE$`（遙控器設了解鎖開關時 PX4 每次開機改成 0）；解析出 500 個名字 |

- 狀態是 AI 起草（`status.state = draft`）：使用者要求這輪不中斷跑完，值照 6C 的 HITL 設定與 Cube 的開機腳本；**待使用者確認**。
- [fc_params_20261020.params](fc_params_20261020.params)：接手前 FC 的完整讀回（QGC 格式，可在 QGC 載回），原樣保存；SHA-256 `1af2f6d6d2bf800d0f9c585786140abcd4560a981f2b8ee307adf45d0f74e44f`。那時 FC 是實機的設定：3 個舵面（`CA_SV_CS_COUNT=3`）、HIL 通道 201、202、203、101、遙控模式槽、`EKF2_ABL_LIM=0`。接手時改回配方的值，做法見 [fc README](../../docs/fc/README.md)「接手一塊用過的飛控」。

## 紀錄

證據在做測試那台機器的 `~/.cache/px4-llm-sim2hitl/`（不隨專案）。

| 時間（UTC） | 做了什麼 | 結果 | 證據 |
|---|---|---|---|
| 2026-10-20 08:36 | 從使用者自建的 hil 版燒錄成本配方（只燒錄） | PASS：一次成功，身分核對 PASS | `deploy_runs/run_20261020_083611Z_f8d97e75` |
| 2026-10-20 08:38–08:41 | 接手：一次性配方把 16 個殘留值改回 | 第一次 FAIL（`MAN_ARM_GESTURE` 被 PX4 改回 0，自動回復）；修正後 PASS，15 個寫入 | `onboard/deploy/deploy_20261020_083811Z_5bddb7b6`、`deploy_20261020_084049Z_714813ad` |
| 2026-10-20 08:41 | `deploy-test --flash`（燒錄、參數 0 個、`hitl-run`） | PASS：盤旋 60.16 s、2.78 圈、半徑 82.2 m；23,079 步 0 次晚到、最慢 0.35 ms；飛控記錄（7.0 MB）下載兩次相同、嚴格解析 PASS | `deploy_runs/run_20261020_084153Z_870d9582`、`hitl_runs/run_20261020_084238Z_96433390` |
| 2026-10-20 09:02 | `02_template_cubeorangeplus` 的 `deploy-test --flash`（燒錄、參數、開機檔、`hitl-run` 含範本判定） | PASS：OFFBOARD 61.0 s，航向 +29.15°、高度 +9.39 m（超越 18 %；高度基準是實測高度，因為 SD 卡的記錄清單沒有 `tecs_status`）、最大傾角 14.0°；41,069 步 0 次晚到；飛控記錄（12.6 MB）下載兩次相同、嚴格解析 PASS | `deploy_runs/run_20261020_090250Z_026ce466`、`hitl_runs/run_20261020_090346Z_e229e5c7` |

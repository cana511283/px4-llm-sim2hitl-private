# CubePilot Cube Orange+

2026-10-20，在一台 Cube Orange+（USB 直接接電腦）上把整輪走完後整理。換飛控的一般清單見 [fc README](README.md)。

結論：Cube Orange+ 可以跑完「編韌體 → 燒錄 → 身分 → 參數 → 開機檔 → HITL → 飛控記錄」，三個配方（原生、原生＋記憶卡修正、範本模組）的一條龍都 PASS，原生 PX4 的盤旋和 6C 的基準幾乎一樣。和 6C 不同、要先知道的有三件：**PX4 的預設韌體沒有 `pwm_out_sim`，不能做 HITL**（配方改用加了它的 `hil` 變體）；**FC 回報的板子身分是 USB 產品碼 4184，不是 board id 1063**；**出廠的 ArduPilot bootloader 叫 `CubeOrange+-BL`**，runtime 原本認不出它是 bootloader。

驗證環境：Cube Orange+（STM32H747、32 KiB FRAM、2 GB SD 卡、IO 處理器）、USB 供電（伺服軌 0.03 V，沒有電池）；Ubuntu 24.04.5 筆電（Dynabook，18 執行緒，沒有 MATLAB、沒有即時排程權限）；PX4 v1.17.0。

## 實跑紀錄

| 步驟 | 結果 |
|---|---|
| FC 原本的狀態 | 本機自建的 PX4 v1.17.0 `hil` 變體（`ver all`：`Build uri: localhost`、`Build variant: hil`，2026-07-26）；`SYS_HITL=1`、機架 2100，但機身改成 3 個舵面（`CA_SV_CS_COUNT=3`，單副翼＋升降舵＋方向舵）、HIL 通道 201／202／203／101；SD 卡有實機用的記錄清單與 9/27–10/15 的飛行記錄。原參數存在 [deploy/00_native_cubeorangeplus/fc_params_20261020.params](../../deploy/00_native_cubeorangeplus/fc_params_20261020.params) |
| 預設韌體（`cubepilot_cubeorangeplus_default`） | 編譯 PASS（98.10 %），但沒有 `pwm_out_sim`、參數名單沒有 `HIL_ACT_*`：不能用 |
| `hil` 變體（[patch](../../px4/patches/px4_cubeorangeplus_hil_variant_pwm_out_sim.patch)） | 三個配方編譯 PASS：原生 98.46 %、＋記憶卡修正 98.46 %、＋範本模組 98.58 %（上限 1,966,080 bytes）；參數 1,979 個（＝預設 1,962＋17 個 `HIL_ACT_*`）；PX4 hash `c45268d056`（patch commit，可重現）、NuttX 原生 `fb2fadf6…`／修正版 `8d3ed2a2…`；每次約 100–180 s |
| 燒錄 | `px_uploader.py` 經出廠的 ArduPilot bootloader，第一次就成功，約 30 s；序列埠一直是 `/dev/ttyACM0` |
| 身分 | 修正後 PASS：FC 回報 board_version 4184、PX4 與 NuttX hash 都等於配方 |
| 接手 | FC 上 16 個值和配方不同（機身 5 個、HIL 通道與記錄 4 個、遙控模式槽與 EKF 等 7 個）；第一次部署因 `MAN_ARM_GESTURE` 被 PX4 改回 0 而 FAIL、自動回復；查明原因後第二次 PASS（15 個寫入，重開後讀回正確） |
| 原生一條龍（`00_native_cubeorangeplus`） | PASS：盤旋 60.16 s、2.78 圈、半徑 82.2 m、高度 99.8–113.2 m；23,079 步 0 次晚到，最慢 0.35 ms；序列埠 0 錯、最慢寫入 0.18 ms；飛控記錄第 23 筆（飛行前最新 22）7.0 MB，下載兩次相同、嚴格解析 PASS；恢復後未解鎖、`SYS_HITL=1` |
| 範本一條龍（`02_template_cubeorangeplus`） | PASS（燒錄、身分、參數、開機檔、`hitl-run` 八部分）：FC 接受模組在韌體內自己發的 OFFBOARD 訊號；OFFBOARD 61.0 s，航向 +29.15°（誤差 −0.85°、不超越、15.1 s 安定）、高度 +9.39 m（誤差 −0.61 m、超越 18 %、24.9 s 安定）、最大傾角 14.0°、空速 17.2–29.3 m/s，範本判定 PASS；盤旋 60.02 s、1.98 圈；41,069 步 0 次晚到、最慢 1.24 ms；飛控記錄第 24 筆 12.6 MB，下載兩次相同、嚴格解析 PASS。**判定的高度基準和 6C 不同**：這張 SD 卡的記錄清單沒有 `tecs_status`，判定程式改用階躍前的實測高度（6C 與 SITL 用 TECS 的高度目標），所以不能拿來和 6C 的「超越 33 %、FAIL」直接比；2026-10-20 起報告的 `altitude_base_source` 會寫明 |
| 原生＋修正一條龍（`01_sdfix_cubeorangeplus`，交接時 FC 上是它） | PASS：先刪掉 02 的開機檔再燒錄；盤旋 60.16 s、2.79 圈、半徑 82.1 m、高度 99.9–113.3 m；23,094 步 0 次晚到、最慢 0.29 ms；飛控記錄第 25 筆 7.0 MB，下載兩次相同、嚴格解析 PASS；結束後未解鎖、`SYS_HITL=1`、沒有待恢復 |

和 6C 的原生基準（`hitl/flight_test.json` 的 baseline，2026-10-17）並排：盤旋 60.16／60.06 s、圈數 2.78／2.78、半徑 82.2／82.3 m、最低 99.8／100.0 m、最高 113.2／113.2 m、最慢晚到 0.35／0.79 ms。兩邊 PX4 控制程式與模型相同，差的是 FC 的硬體與 EKF 的估測；沒有事先講好的判定，只列差異。

## 和 6C 不同的地方

每一項寫 Cube 怎麼樣、影響、怎麼確認；「例」是這次實際遇到的。

### 預設韌體不能做 HITL

- v1.17.0 的 `boards/cubepilot/cubeorangeplus/default.px4board` 沒有任何模擬模組：`pwm_out_sim` 沒編進去。6C 的 board 開了 `SIMULATOR_SIH`，順帶選了 `pwm_out_sim`。
- 影響：`SYS_HITL=1` 開機時 rcS 的 `pwm_out_sim start -m hil` 失敗，沒有 HIL 輸出，參數名單也沒有 `HIL_ACT_*`。
- 做法：[px4/patches/px4_cubeorangeplus_hil_variant_pwm_out_sim.patch](../../px4/patches/px4_cubeorangeplus_hil_variant_pwm_out_sim.patch) 加一個 `hil.px4board`（只有 `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y`），make 目標 `cubepilot_cubeorangeplus_hil`。PX4 的控制程式不變；多 7 KB。
- 確認：建置目錄的 `boardconfig` 有 `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y`；FC 上 `pwm_out_sim status` 有 `Param prefix: HIL_ACT`；`ver all` 是 `Build variant: hil`。
- 例：FC 原本跑的就是使用者自己編的 `hil` 變體，所以一開始讀回時 FC 有 `HIL_ACT_*`、用預設目標編出來的參數名單卻沒有，讀回判 INCOMPLETE（17 個 `not_in_list`）。

### 板子身分是 USB 產品碼

- PX4 的 `AUTOPILOT_VERSION.board_version` = `CONFIG_CDCACM_PRODUCTID` = 0x1058 = 4184；韌體與 bootloader 的 board id 是 1063。
- 影響：原本的核對拿 board id 比，Cube 一定判身分 FAIL；而 Cube 的原生韌體和 6C 的原生韌體 PX4、NuttX hash 相同，「FC 上是哪個配方」會把 Cube 誤認成 6C 的 `00_native`。
- 做法：`px4/build_firmware.py` 記下 `embedded.board_version`；`deploy/check_params.py` 與 `deploy-test` 的配方比對都比它（舊紀錄沒有時用 board_id，6C 兩者相同）。

### ArduPilot 的 bootloader

- 進 bootloader 時 USB 名稱是 `CubePilot CubeOrange+-BL`，VID/PID 和 PX4 韌體一樣（`2dae:1058`），USB 序號不同（韌體是 `0`）。
- 影響：runtime 原本只認 `bootloader` 與 `PX4 BL`，把它當成一般韌體。重開機時是另一層保護（USB 序號對不上已記住的韌體身分）讓 owner 沒有連上去；但 owner 剛好在 FC 停在 bootloader 時才啟動，就會連上去送 MAVLink。已修：`hitl/host/serial_helpers.py` 的 `BOOTLOADER_LABEL` 認得 `-BL` 結尾，runtime 與 `deploy-test` 共用。
- 例：參數部署的每次重開，bootloader 出現約 4.8 s；燒錄時約 30 s。`px_uploader.py` 讀不到版本（`Bootloader version: unknown`），燒錄正常。

### IO 處理器、安全開關、伺服軌

- 有 px4io（`cubepilot_io-v2`，韌體在 PX4 的 ROMFS，開機時核對 CRC，必要時更新；這次 CRC `0x0458fffe` 沒變）。HITL 時 px4io 照樣啟動；`PWM_MAIN_FUNC*` 都是 0，IO 的輸出停在 disarmed 值。
- px4io 一啟動就宣告安全開關存在；`CBRK_IO_SAFETY` 預設 22027，不用按。
- 確認動力隔離：`px4io status` 的 `voltage_v` 是伺服軌電壓，這次 0.033 V（沒供電）。HITL 沒有 `battery_status`，不能拿它判斷有沒有接電池。

### 開機腳本與參數

- `rc.board_defaults` 設 `SENS_EN_THERMAL 0`（關 IMU 加熱；6C 上同值但沒有腳本設，放在 `set`）、電源模組分壓（屬 `fc_owned`）；STM32H7 的架構預設和 6C 相同。配方的 `boot` 因此是 6 個。
- `set` 是 6C 的 HITL 值（`SYS_HITL`、`HIL_ACT_FUNC1..5`、`HIL_ACT_REV`、`SDLOG_PROFILE`），不含 6C 才有的 `MAV_1_CONFIG`、`MAV_1_FLOW_CTRL`。
- `fc_owned` 多一條 `^MAN_ARM_GESTURE$`：這台的遙控器設了解鎖開關（`RC_MAP_ARM_SW=5`），PX4 每次開機把 `MAN_ARM_GESTURE` 改成 0。自動飛手用 MAVLink 指令解鎖，不受影響。
- 飛行後 PX4 會改 `CAL_ACC*_OFF`、`CAL_BARO*_OFF`（屬 `fc_owned`，核對照常 PASS）。

### 記憶卡、FRAM、記錄

- SD 卡在 SDMMC1（STM32H7、開 D-cache），和 6C 同一個 NuttX 驅動，記憶卡修正（`01_sdfix_cubeorangeplus` 起）照樣適用。
- FRAM 32 KiB，`/fs/mtd_params` 1024 blocks。
- 從 ArduPilot 換過來留下的 `param_import_fail.*`：見 [fc README](README.md)「從 ArduPilot 換過來的飛控」。使用者要求刪除，2026-10-20 已刪（備份在 cache 的 `manual/ardupilot_leftovers/`）。
- 這張卡的 `etc/logging/logger_topics.txt`（實機的慣量飛行測試清單，32 個主題）取代了 `SDLOG_PROFILE=17`：飛控記錄 7.0 MB、27 種格式（6C 是 10.9 MB），`tecs_status` 不在清單裡。沒有動它；要和 6C 的記錄逐項比，先決定要不要換掉這份清單。

### 版本類型

- 板子變體的 patch commit 在 PX4 主 repo，FC 回報的版本從 1.17.0 release（`17891583`）變成 1.17.0 dev（`17891328`）。身分核對比 hash，不受影響。

## 怎麼確認

| 要確認的事 | 指令 | 這次的結果 |
|---|---|---|
| FC 上是什麼韌體 | MAVLink Console：`ver all` | `Build variant: hil`、PX4 `c45268d0…`、NuttX `fb2fadf6…` 或 `8d3ed2a2…` |
| HIL 輸出模組 | `pwm_out_sim status` | `Param prefix: HIL_ACT`，5 路 201、202、203、204、101 |
| bootloader 的 USB 名稱 | 燒錄時 `ls /dev/serial/by-id/` | `usb-CubePilot_CubeOrange+-BL_<序號>-if00` |
| 伺服軌有沒有電 | `px4io status` 的 `voltage_v` | 0.033 V |
| 參數存放 | `mtd status`、`param status` | `/fs/mtd_params` 32 KiB；988／1979 used |
| SD 卡的記錄清單 | `ls /fs/microsd/etc/logging` | 有 `logger_topics.txt` |

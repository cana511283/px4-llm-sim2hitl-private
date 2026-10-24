# fc：不同飛控的理解

專案在 Pixhawk 6C 上建立；2026-10-20 第一次換到另一塊飛控（CubePilot Cube Orange+）把整輪走完。本資料夾寫「換一塊飛控時，哪些地方會不同、怎麼確認」，每塊飛控一個檔（目前是 [cubeorangeplus.md](cubeorangeplus.md)；6C 的細節在 [deploy/00_native](../../deploy/00_native/README.md)）。這是理解，不是安裝腳本：換到新的飛控或新的 PX4 版本時，照下面的清單重新查原始碼與實機，查到和本文不同的現象先留證據，再提議更新。

## 飛控差異放在哪一層

| 層 | 內容 | 換飛控時 |
|---|---|---|
| 跟飛控無關（共用一份） | 模型與 plant、SITL（不碰飛控）、情境、判定、`hitl-run` 的流程；配方裡機架 2100 的參數（`airframe`）與 HITL 的通道對應（`set`） | 不用改 |
| 每塊飛控各一份：部署配方 | `deploy/<層級>_<名稱>[_<板子>]`：make 目標、patch（含板子設定）、韌體與它的參數名單、FC 回報的身分（`embedded`）、開機腳本設的值（`boot`）、燒錄紀錄。6C 沒有板子字尾（`00_native`、`01_sdfix`、`02_template`）；Cube Orange+ 是 `*_cubeorangeplus` | 照下一節建一組 |
| 每一台 FC 自己的 | 校正、遙控器設定等（`fc_owned`）；以及它在別處用過留下的參數與 SD 卡內容 | 照「接手一塊用過的飛控」 |

FC 只有一個序列埠可接：`deploy-test` 的計畫會列出 FC 現在符合哪個配方（比對板子、PX4 與 NuttX 的 hash），對不上就是「unknown firmware」。

## 換一塊新飛控要查的事

依 2026-10-20 的經驗整理；每一項寫為什麼、怎麼查。

1. **預設韌體能不能做 HITL**：HITL（`SYS_HITL=1`）的舵面輸出靠 `pwm_out_sim`。看鎖定版本的 `boards/<廠商>/<板子>/default.px4board` 有沒有 `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y`，或會選到它的 `CONFIG_MODULES_SIMULATION_SIMULATOR_SIH=y`（6C 是後者）。沒有的話開機時 `pwm_out_sim start -m hil` 失敗：沒有 `HIL_ACT_*` 參數、FC 不送 HIL_ACTUATOR_CONTROLS、根本飛不了。做法：在 `px4/patches/` 加一個只多編 `pwm_out_sim` 的板子變體（PX4 的 `<label>.px4board` 疊在 `default.px4board` 上），配方的 make 目標改成 `<板子>_hil`。FC 上 `ver all` 會顯示 `Build variant: hil`。
2. **flash 空間**：`firmware.json` 的 `flash_used_percent`。加模組前先看還剩多少。
3. **FC 回報的板子身分**：`AUTOPILOT_VERSION.board_version` 是 PX4 的 `px4_board_version()`，等於 USB 產品碼（`CONFIG_CDCACM_PRODUCTID`），**不一定等於韌體的 board id**（6C 兩者都是 56；Cube Orange+ 是 0x1058 = 4184，board id 是 1063）。`px4/build_firmware.py` 從建置的 NuttX 設定記下它（`embedded.board_version`），身分核對（`deploy/check_params.py`）與「FC 上是哪個配方」都比這個。
4. **USB 名稱**：韌體與 bootloader 各叫什麼，燒錄時另開一個視窗每 0.5 s 記一次 `ls /dev/serial/by-id/`。runtime 靠名稱認出 bootloader（`hitl/host/serial_helpers.py` 的 `BOOTLOADER_LABEL`），認不出來就補上：FC 停在 bootloader 時 owner 不可以連上去送 MAVLink（bootloader 的燒錄協定裡有抹除指令）。
5. **開機腳本設的參數**：板子的 `init/rc.board_defaults`、晶片的 `platforms/nuttx/init/<晶片>/rc.board_arch_defaults`、`rcS` 的 HITL 分支，寫進配方的 `boot`（只核對、不寫）。韌體的參數名單每塊板子不同（6C 2,038 個、Cube Orange+ 的 hil 版 1,979 個），名單裡沒有的參數不能放進配方。
6. **IO 處理器與安全開關**：有 px4io 的板子（Cube）不論是否 HITL 都會啟動 px4io，它一啟動就宣告安全開關存在。v1.17 的 `CBRK_IO_SAFETY` 預設 22027（不用按），所以照預設沒事；有人把它改成 0 時，只接 USB、沒接安全開關的 HITL 會因「Press safety button first」解鎖不了。
7. **記憶卡**：STM32H7 的板子要套 NuttX 的 SD 卡讀取修正（[AGENTS](../../AGENTS.md) 第 1 段）；看 defconfig 有沒有 `CONFIG_STM32H7_SDMMC1`／`SDMMC2` 與 `CONFIG_ARMV7M_DCACHE`。修正後 FC 回報的 NuttX hash 跟板子無關（6C 與 Cube 都是 `8d3ed2a2…`）。
8. **參數存放**：nsh 的 `mtd status` 要有 `/fs/mtd_params`。例外的例子：fmu-v3 的副廠板用 16 KiB 的 FRAM，PX4 1.15 起切不出 32 KiB 的參數區，參數存不住。
9. **PX4 主 repo 的 patch 會讓版本變成 dev**：patch commit 在 PX4 主 repo（不是子模組）時，`git describe` 變成 `v1.17.0-1-g…`，FC 回報的版本類型從 release 變成 dev（`flight_sw_version` 最低位元組 255 → 0）。身分核對比的是 hash，不受影響；QGC 會顯示 dev 版。

## 接手一塊用過的飛控

`param-deploy` 照設計只寫配方的 `set`。FC 上別的值（在 QGC 改過的機身舵面、遙控模式槽、EKF 設定……）會讓部署被擋下（`airframe`／`boot` 的值不同：not written by deploy），或讓 `hitl-run` 的參數核對 FAIL（`not_default`）；工具裡沒有把它們改回來的動作。2026-10-20 在 Cube Orange+ 上的做法：

1. 完整讀回並存檔，QGC 可以載回（`param-readback`；例：[deploy/00_native_cubeorangeplus/fc_params_20261020.params](../../deploy/00_native_cubeorangeplus/fc_params_20261020.params)）。
2. 燒錄配方的韌體（`./runbridge flash --release deploy/<配方>`），讓身分對上（參數部署會先核對身分）。
3. 在 cache 做一份一次性的配方：正式配方，加上把殘留的名字放進 `set`（機身值改回機架腳本的值、其他改回韌體預設值；放進 `set` 的機身名字要從 `airframe` 拿掉），用 `./runbridge param-deploy --release <那份配方>` 寫入：同樣有交易、讀回與失敗回復。
4. `./runbridge param-plan --release deploy/<配方>` 要 0 個寫入、核對 PASS，再照常 `deploy-test --flash`。

PX4 自己會改回去的值要先查清楚：`RC_MAP_ARM_SW > 0`（遙控器設了解鎖開關）時，PX4 的 `manual_control` 每次開機把 `MAN_ARM_GESTURE` 設成 0，寫 1 會在重開後被改回去、部署因此 FAIL 並自動回復。這種跟著操作者遙控器走的值放進 `fc_owned`（Cube 的配方已經加了 `^MAN_ARM_GESTURE$`）。

SD 卡也要看：別處用過的卡可能帶著自己的記錄清單 `etc/logging/logger_topics.txt`，它會取代 `SDLOG_PROFILE`（記錄的主題與大小都不同）。runtime 的 `sd-put`／`sd-rm` 只動 `etc/extras.txt`，其他檔案要由人決定、在 QGC 的 MAVLink Console 處理。

## 從 ArduPilot 換過來的飛控

CubePilot 的板子出廠多半是 ArduPilot（韌體與 bootloader）。這次遇到的：

- **FRAM 裡是 ArduPilot 的參數區**（開頭 `50 41 06 00`：AP_Param 的 "PA"、版本 6）。PX4 第一次開機把這 4 個 byte 當成 BSON 長度（409,936），參數匯入失敗、響錯誤音，把整片 FRAM 倒成 SD 卡上的 `param_import_fail.bson`（32 KiB），失敗紀錄在 `param_import_fail.txt`。PX4 之後存參數就會蓋掉 FRAM，不用另外清；SD 卡上的兩個檔留證據後可以刪。
- **bootloader 是 ArduPilot 的**：USB 名稱是 `<板子>-BL`（例 `CubeOrange+-BL`），USB VID/PID 和 PX4 韌體相同、序號不同；`px_uploader.py` 讀不到它的版本（`Bootloader version: unknown`），但燒錄 PX4 完全正常。要換成 PX4 的 bootloader（`SYS_BL_UPDATE`）是另一件事：寫入中斷會讓板子開不了機，由人決定。

## 驗證過的飛控

每格只有 PASS、FAIL 或沒測。

| 飛控 | 配方 | 韌體編譯 | 一條龍（`deploy-test --flash`＋`hitl-run`） | 細節 |
|---|---|---|---|---|
| Pixhawk 6C（`px4_fmu-v6c_default`） | `00_native`、`01_sdfix`、`02_template` | PASS（2026-10-18） | PASS（2026-10-18；02 的範本判定 FAIL：高度衝過頭，見根目錄 README） | [deploy/00_native](../../deploy/00_native/README.md) |
| Cube Orange+（`cubepilot_cubeorangeplus_hil`） | `00_native_cubeorangeplus`、`01_sdfix_cubeorangeplus`、`02_template_cubeorangeplus` | PASS（2026-10-20） | PASS（2026-10-20，三個配方；02 的範本判定 PASS，但高度基準和 6C 不同，見細節） | [cubeorangeplus.md](cubeorangeplus.md) |

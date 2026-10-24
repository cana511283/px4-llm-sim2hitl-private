# px4-llm-sim2hitl

[檔案分類與維護規則](docs/ORGANIZATION.md)：後續更新均依此執行。

[技術文件](docs/README.md) · [實驗紀錄](experiments/README.md) · 分析工具保留於 `analysis/`。

**繁體中文** | [English](README.en.md)

（英文版是翻譯；兩者不一致時以本中文版為準）

飛控開發套件：MATLAB/Simulink 飛機模型 → 轉成 C → SITL（精簡 PX4）→ 客製 PX4 韌體 → 實體飛控（Pixhawk）HITL。用 AI 開發時，AI 的工作規則在 [AGENTS.md](AGENTS.md)。

![流程總覽：需求與規格 → SITL 電腦模擬 → HITL 實體飛控；人選下一步，AI 做技術](images/overview.svg)

**範例飛機是 SkySurfer**（固定翼）：`model/`、`hitl/aircraft/skysurfer/`、模型版本名 `skysurfer-…` 與 `SkySurfer_Core` 這些是飛機本身。套件叫 `px4-llm-sim2hitl`：資料夾用全名（Linux 是 `~/.cache/px4-llm-sim2hitl`、`~/.local/share/px4-llm-sim2hitl/venv`、`~/.config/px4-llm-sim2hitl`；這台機器實際的位置用 `./runbridge where` 查），環境變數用短名 `SIM2HITL_*`（環境變數不能有減號）。換成自己的飛機見第 1.1 段。

> 狀態：從舊的 macOS 專案搬到 Ubuntu 已完成，整套流程跑通（見最後一節）。2026-10-20 換到第二塊飛控 Cube Orange+ 也走完一輪；換飛控要查的事見 [fc/](docs/fc/README.md)。

## 怎麼開始

**把這個網址貼給你的 AI 就好。** 它會讀這份 README 和 [AGENTS](AGENTS.md)，接下來你只要說想做什麼。

- **只想了解**：任何能讀網頁的 AI 都可以，問它「這是什麼、能幫我做什麼」。
- **要動手開發**：用能在你電腦上執行指令的 AI（例如 Claude Code、Codex），一樣貼網址，請它下載到你電腦上一個固定的資料夾（之後不要搬；要搬先照 AGENTS 恢復）。
  - 第一次：說「照 README 的環境準備，幫我把這台電腦設好」。它會一項一項查，要 sudo 或權限時請你貼一次。**完整流程只在 Ubuntu 24.04（x86-64）驗證過**；macOS（Apple Silicon）驗證到 SITL（見「驗證矩陣」與 [platforms/macos.md](docs/platforms/macos.md)）；其他系統沒測過，AI 會照官方文件試，但可能要多花時間踩坑。
  - 之後：直接說你想做什麼，例如「我想換成我的飛機」「我想做一個新的導引算法」。它會從第 1 段「需求與規格」開始跟你談。
- **AI 要夠力**：要能讀完很長的文件、自己寫程式、自己跑指令並看懂結果。模型太小或推理開太低，容易跳過文件自己猜。測試過可以用：GPT-6.1 sol（推理 medium）。

開場白範例：「請把 https://github.com/rhyne1012/px4-llm-sim2hitl 下載到 ~/Developer，先讀 README 和 AGENTS，然後跟我談第 1 段：我想……」

## 這套專案的精神

**這不是一套自動化工具，也不應該變成一套想包辦所有開發情境的自動化；它是讓你的 AI 一開始就懂這個流程的交接。** 從飛機模型、PX4 原始碼、SITL、編譯燒錄到實體飛控，這條路怎麼走、哪裡有坑、什麼已經驗證過、怎樣才算對，都寫在文件（這份 README、[AGENTS.md](AGENTS.md)、各資料夾的 README）和現成的檢查裡。AI 讀完就接得上，不必每次從頭摸索整個流程，可以把時間和 token 花在真正要做的事：幫你除錯、幫你開發。

打個比方：AI 每開一次新對話，都像第一天上班的新同事。這包是前人留下的交接：系統怎麼運作、哪裡有坑、哪些已經驗證過、做事的規矩，還有一條實際走通過的基本路線（模型 → SITL → 燒錄 → HITL）。交接不會替他把工作做完，也不會列出他將來遇到的每一種狀況；它讓他第一天就能把心思放在你的問題上，遇到新狀況也知道從哪裡下手。處理完的新經驗和跟你談定的規則寫回交接，下一個新同事就更快上手。

- **先有理解，才有工具**：指令和腳本只是把已經想通的步驟固定下來。文件寫的是「為什麼這樣做、坑在哪裡」，例如記憶卡讀取的錯誤、地面模型不會讓飛機轉動、風什麼時候開始吹。這樣 AI 遇到問題才知道去哪裡查，也不會為了讓事情成功就繞過安全檢查硬做。
- **目的**：把這包交給別人，讓他和他的 AI 能從改飛機模型、開發飛控，一路做到編譯、燒錄與 HITL。
- **簡單、輕量、同一份來源**：每件事只放一個地方；能沿用現有程式與檢查就沿用；不為了可能用不到的情況預先加東西。
- **AI 講白話**：技術細節由 AI 查、AI 處理；只有目標、選擇與成功條件問人。
- **第 1 段最重要**：AI 先讀鎖定版本的 PX4 原始碼，把可行的接法用白話攤開和人討論，由人選；再建議先做「最小打通」，讓整條路先通、記錄下來，之後的修改交給自動化。
- **第 3 段一條龍**：編譯 → 燒錄 → 身分核對 → 參數 → 開機檔 → HITL → 飛控記錄，一個指令跑完；開始前先告訴人 FC 上現在是什麼，然後一律重新燒錄（只改外部模組時，FC 回報的版本編號可能不變，分不出新舊）。
- **自動化跟著開發慢慢長出來**：工具是理解的副產品。同一件事做了兩三次，AI 就建議把它變成專案裡的指令或腳本，由人決定；越用越省 token。
- **環境不寫死、驗收標準寫死**：路徑、編譯器、MATLAB、PX4 原始碼樹自動找、可指定，換平台時請 AI 查官方文件；量尺（版本鎖、檢查容差、4 ms、負載門檻）不隨環境改。
- **不分權限等級**：每段結束時問「下一步 1／2／3」，段內不逐項詢問；但 runtime 的安全檢查（FC 身分、未解鎖、讀回、恢復）不可繞過，見 AGENTS 第 3 節。
- **提醒，不擋**：非預設的飛行參數照跑並記錄；看起來不對的地方告訴人，由人決定。
- **SITL 與 HITL 用同一把尺**：同一份情境、同一支判定程式、同一種記錄（ULog）；兩邊的差異本身就是要找的資訊。

## 流程

1. **需求與規格**：先談清楚要做什麼、怎樣算成功。
   - 1.1 飛機模型與配平（換飛機或改模型時）
   - 1.2 算法、部署與驗收
2. **SITL（電腦模擬）**：反覆模擬，結果存成 `trace.csv`＋ULog（MATLAB 可直接讀）。
3. **HITL（自動編譯、燒錄、測試）**
   - 3.1 手動 HITL（`./runbridge`＋QGC）
   - 3.2 出場檢查：QGC 關掉 HITL 後確認舵面有輸出（尚未完成）

## 環境準備（第一次）

完整流程只在 **Ubuntu 24.04 x86-64** 驗證過（各平台驗證到哪一段見「驗證矩陣」；macOS 的平台差異見 [platforms/macos.md](docs/platforms/macos.md)）。下面每一項都寫了「為什麼要」與「驗證機（Ubuntu）的做法」；做法只是例子，換平台或版本時，**請 AI 先查該項的官方文件**確認正確做法再裝。

| 項目 | 為什麼要 | Ubuntu 24.04 的做法（例子） | 怎麼確認 |
|---|---|---|---|
| Python 環境 | runtime、測試、SITL、分析 | 安裝 [uv](https://docs.astral.sh/uv/)，再 `./runbridge --setup`（venv 的位置：`./runbridge where venv`） | `./runbridge --check-python` |
| MATLAB | 只有改飛機模型（第 1.1 段產碼）才需要：R2026b＋Simulink＋Simulink Coder＋Embedded Coder＋Aerospace Blockset。不需要 Java。沒有 MATLAB 的機器照樣能發布模型 release（第二段改用專案自己的兩個最小標頭檔，見 [plant README](plant/README.md)「沒有 MATLAB 的機器」） | MathWorks 安裝程式；建議接上 MATLAB MCP，讓 AI 能直接操作 | `matlab` 在 PATH，或設 `SIM2HITL_MATLAB_ROOT` |
| C 編譯器 | plant 與 HIL 函式庫 | 系統內建 gcc（`CC` 可指定別的） | `cc --version` |
| PX4 原始碼與工具鏈 | SITL 與編韌體；版本鎖在 [px4/px4_lock.json](px4/px4_lock.json) | 下載鎖定的 tag 到專案外，`python3 px4/check_px4_tree.py --path <樹> --remember`；工具鏈用 PX4 官方腳本 `bash <樹>/Tools/setup/ubuntu.sh --no-sim-tools`（不裝 Gazebo；會用 sudo）。官方腳本把 PX4 的 Python 套件用 `--break-system-packages` 裝進使用者的 Python；不想動它時，另建一個 venv 放在 PX4 樹外（`pip install -r <樹>/Tools/setup/requirements.txt "setuptools<70"`），編譯時把它的 `bin` 放在 PATH 最前面（2026-10-20 第二台 Ubuntu 的做法；樹裡放 `.venv` 會讓乾淨度核對 FAIL）。樹已經被別的工作改過時，另複製一份乾淨的給本專案，不要動原樹。官方說明：PX4 文件 “Ubuntu Development Environment” | `python3 px4/check_px4_tree.py` 為 PASS |
| **即時排程權限** | 4 ms 迴圈的 worker 執行緒要用即時排程（Linux：SCHED_FIFO，優先權 2）。沒有權限也能跑，但時間餘裕變小：驗證機 600 s 負載最慢晚到 3.7 ms（門檻 4 ms），有權限時 0.47 ms | 桌面程式由 systemd 使用者服務啟動，`/etc/security/limits.d` 管不到；改用 systemd，兩個檔都要：`/etc/systemd/system/user@<UID>.service.d/rtprio.conf` 寫入 `[Service]` 與 `LimitRTPRIO=10`（使用者管理程式本身的上限），以及 `/etc/systemd/user.conf.d/rtprio.conf` 寫入 `[Manager]` 與 `DefaultLimitRTPRIO=10`（它啟動桌面程式時套用的值；只設第一個，實測重開機後桌面程式仍是 0），`sudo systemctl daemon-reload`，再重新登入。暫時只給一個程序：`sudo prlimit --pid <PID> --rtprio=10:10`。官方說明：systemd.exec(5) 的 `LimitRTPRIO=`、sched(7) | `ulimit -r` ≥ 2；`./runbridge status` 的 `scheduler.time_constraint_result` 為 0 |
| 序列埠 | 連 Pixhawk（USB CDC-ACM） | 使用者加入 `dialout` 群組；停用 ModemManager（它會試探新插上的序列埠）：`sudo systemctl disable --now ModemManager` | `ls -l /dev/ttyACM*`、`groups` |
| QGC | 人看飛控、手動操作（3.1） | 官方 GitHub 發布的 AppImage（驗證機：v5.1.5，`~/Applications/`，核對官方公布的 SHA-256）；系統套件照 QGC 官方 Linux 說明（驗證機補裝 `libfuse2t64 gstreamer1.0-plugins-bad gstreamer1.0-libav libxcb-cursor0`）。**關掉 USB 自動連線**，只用 UDP 14550 連 runtime，否則會和 owner 搶飛控序列埠：設定檔 `~/.config/QGroundControl/QGroundControl.ini` 的 `[AutoConnect]` 把 `autoConnectPixhawk`、`autoConnectSiKRadio`、`autoConnectRTKGPS`、`autoConnectLibrePilot`、`autoConnectZeroConf` 設 `false`、`autoConnectUDP=true`（QGC 第一次啟動建好設定檔、關閉後再改；或在 QGC 的 Application Settings → Comm Links 取消勾選） | QGC 開著時 `lsof -p <QGC PID>` 只見 UDP 14550、不見 `/dev/ttyACM*` |

**同一台機器放兩份**（例如套件和自己的 fork）：runtime 的狀態跟著資料夾路徑，本來就分開；下面這些預設兩份共用，要分開就設環境變數。暫存資料夾（`SIM2HITL_CACHE`）；SITL 編譯資料夾（PX4 樹裡的 `build/px4_sitl_default_sim2hitl`，`SIM2HITL_SITL_SUFFIX`）——不分開的話，一邊重編，另一邊的 SITL 就跑到這一邊的機體，不會報錯；同時跑 SITL 時的 PX4 編號（`run_sitl.py --instance-base`）；韌體建置樹（`<PX4 樹>-build`，`SIM2HITL_PX4_BUILD_TREE`；共用時不要兩邊同時編）。FC 只有一個：同一時間只能一份開 owner，換之前先在原處 `recover` 並關 owner。

## 常用指令

```sh
./runbridge --help                    # 所有動作
./runbridge where                     # 這台機器實際的位置（venv 的 python、cache、PX4 樹、模型 release）；where python 只印一項
./runbridge update-model --generate   # 第一段：MATLAB 產碼 → model/generated（要 MATLAB）
./runbridge update-model              # 第二段：編譯、四道檢查、發布模型 release（不需要 MATLAB 連線）
./runbridge bridge                    # 開 owner（持有模型與序列埠）；QGC 用 UDP 連
./runbridge status                    # owner 狀態
./runbridge model-start              # 3.1：模型在地面開始運行（要先確認沒有待恢復、status 的 missing_conditions 是空的）；--scenario <場景> 換場景
./runbridge takeoff --altitude 100 --after template_offboard   # 3.1：模型在地面運行時自動起飛 → Hold → OFFBOARD（範本，FC 要是 02_template）→ Hold（QGC 的選單切不到固定翼的 Offboard）
./runbridge deploy-test --release deploy/<配方>           # 第 3 段：只看計畫（FC 上現在是什麼、會做什麼）
./runbridge deploy-test --release deploy/<配方> --flash   # 平常用這個：必要時編韌體 → 燒錄 → 參數 → 開機檔 → hitl-run
./runbridge deploy-test --release deploy/<配方> --reuse   # 只在人明確要求時：沿用 FC 上的韌體，參數 → 開機檔 → hitl-run
```

手動 HITL（3.1）的順序：`bridge` 開 owner → `status` 確認恢復完成、啟動條件齊全 → `model-start` → 確認模型在地面運行 → `takeoff`（之後可以用 QGC 的搖桿接手，要點哪裡見 [hitl README](hitl/README.md)「用 QGC 搖桿手動飛」）。結束後 `model-stop`，再照 [hitl README](hitl/README.md) 的恢復規則 `recover`（在空中結束的要加 `--hil-force-disarm`），最後 `shutdown` 關 owner。

## 已知問題

- **模型與 PX4 參數不吻合（空速偏高）**：PX4 的 `FW_THR_TRIM=0.6`，這個飛機模型在 15 m/s 平飛只需要約 0.115 的油門。已經觀察到：原生盤旋與 SITL 的空速都在 20–30 m/s（目標 15 m/s）；HITL 中 TECS 要爬高時空速衝高、高度衝過頭，範本的高度判定因此 FAIL。油門配平的差距是可能的原因之一，但兩者的因果與影響多大還沒確認。照 [AGENTS](AGENTS.md) 的原則先查模型（推力、阻力、重量），再決定模型要修，還是這架飛機的 PX4 參數本來就要配合實機。
- **3.2 出場檢查還沒做**：`deploy/00_native` 的基準參數沒有指定實體舵機輸出（`PWM_MAIN_FUNC*`、`PWM_AUX_FUNC*` 全為 0）、`GPS_1_CONFIG=0`、空速計驅動未開；關掉 HITL 不會有任何輸出。需要實機的接線與參數。
- **一般使用者的即時權限**：見「環境準備」的即時排程權限；沒有時能跑，但時間餘裕小。

## 驗證矩陣

只寫哪一段、哪個平台、哪天驗證過；每格只有 PASS、FAIL 或沒測，沒測的不算支援。

| 階段 | Linux（Ubuntu 24.04 x86-64） | macOS（27.0.1 arm64） |
|---|---|---|
| 原始碼樹與環境核對 | PASS（2026-10-18；2026-10-20 第二台） | PASS（2026-10-20） |
| 產碼（MATLAB R2026b） | PASS（2026-10-20，關掉指令集擴充後重跑） | PASS（2026-10-20） |
| 模型發布（四道檢查、60 秒節拍） | PASS（2026-10-20；同日第二台沒有 MATLAB，用專案的最小標頭檔也 PASS） | PASS（2026-10-20） |
| 配平 | PASS（2026-10-20，和上一版每一位相同） | PASS（2026-10-20，每一位相同） |
| SITL 參考情境與範本 | PASS（2026-10-20；第二台也 PASS） | PASS（2026-10-20，指標相同） |
| SITL 多實例 | PASS（2026-10-18，1000 次） | PASS（2026-10-20，第一批有 1 個卡住，見 macOS 文件） |
| 離線 runtime 測試 | PASS（2026-10-20；第二台 15 個檔全過） | PASS（2026-10-20，194 個） |
| 韌體編譯 | PASS（2026-10-18，6C；2026-10-20，Cube Orange+） | 沒測 |
| 實機迴路（HITL） | PASS（2026-10-18，6C；2026-10-20 6C 沒有即時權限、機器忙時時序 FAIL，見下表；2026-10-20 Cube Orange+ 三個配方 PASS，沒有即時權限、機器閒置） | 沒測 |

飛控（Pixhawk 6C、Cube Orange+）各驗證到哪裡，見 [fc README](docs/fc/README.md)「驗證過的飛控」。

## 驗證紀錄（2026-10-18，Ubuntu 24.04）

| 項目 | 結果 |
|---|---|
| 模型 release `skysurfer-6d1cf7a3…`（改名後；改名前 `f53eee94…`） | 四道檢查 PASS；核心與 C 真值對參考資料的誤差和改名前一位不差；對 macOS 錄的參考資料，除了靜止時沒有定義的攻角／側滑角，差 ≤ 6.2e-15 |
| 離線測試 | 14 個檔全部 OK（2026-10-18，改名後）。`test_runbridge_fake` 的恢復測試原本在機器很忙時會誤判（等狀態的輪詢趕不上假飛控 2.3 s 的重開），改成整個恢復期間都送 QGC 流量後，28 核全滿也 PASS |
| 600 s 負載（不開視窗） | PASS；有即時權限時最慢晚到 0.47 ms |
| SITL | 編譯 PASS；參考情境 `stabilized_steps` 的指標和 macOS 紀錄相同；速度：1000 次（每次模擬 60 s）實測 140.8 s，加大批次約 55 s；範本 `guidance_template` PASS |
| ULog 與 MATLAB | SITL 與 HITL 的 ULog 都能用 MATLAB `ulogreader` 畫（`analysis/matlab/ulog_overview_plot.m`，不需要 Java），也能轉 CSV（`analysis/ulog_export.py`） |
| HITL（實體 FC） | 00_native 基準 `hitl-run` PASS（2026-10-17 UTC）：盤旋 60 s、2.78 圈；23,039 步 0 次晚到、最慢 0.79 ms；USB 寫入最慢 0.26 ms；FC 量到感測器 250.009 Hz |
| 第 3 段一條龍 | `deploy/01_sdfix`（原生＋記憶卡修正）用 `deploy-test` 燒錄並跑完：FC 回報 NuttX `8d3ed2a2…`（可預期的那個）；寫參數 0 個；`hitl-run` 七部分全 PASS，盤旋 60.13 s、2.78 圈，23,104 步 0 次晚到、最慢 0.19 ms；飛控記錄（10.9 MB）下載兩次相同、嚴格解析 PASS |
| 範本 `02_template`（HITL，2026-10-18） | 一條龍全 PASS（重編、燒錄、身分、參數、開機檔）；**FC 接受模組在韌體內自己發的 OFFBOARD 訊號**。動作改成進 OFFBOARD 15 s 後轉 30°、35 s 後爬 10 m、共 60 s，盤旋高度判定改以開始盤旋的高度 ±15 m 之後：飛行 PASS；航跡角 +29.1°（不超越、15.2 s 安定）、最大傾角 14°；**高度衝過頭 33%**（SITL 0%）、OFFBOARD 結束前未回到 ±0.5 m，判定 FAIL。同時空速從 21 衝到 28 m/s：TECS 加能量時油門過多（`FW_THR_TRIM=0.6` 對模型的約 0.115），屬於模型與 PX4 參數不吻合，待查 |
| 3.1 手動 HITL（2026-10-18，FC 是 `02_template`） | QGC 只連 UDP；QGC 的模式選單切不到固定翼的 Offboard，改用 `./runbridge takeoff --altitude 100 --after template_offboard`。結果和自動測試幾乎相同：航跡角 +29.0°（15.4 s 安定）、最大傾角 14.2°、高度衝過頭 31% → 範本判定 FAIL（同上，模型與參數不吻合）；0 次晚到；停模型、`recover` 後未解鎖、`SYS_HITL=1`、無待恢復 |
| 交接時的 FC | 2026-10-18 07:22 UTC 用 `deploy-test --flash` 從 `02_template` 刷回 `01_sdfix`：全 PASS（盤旋 60.0 s、2.79 圈、0 次晚到），開機檔已移除；紀錄見 [deploy/01_sdfix](deploy/01_sdfix/README.md) |
| 改名 `px4-llm-sim2hitl`（2026-10-18） | 套件不再借用飛機名：資料夾全名、環境變數 `SIM2HITL_*`、PX4 模組 `plant_sim`／`guidance_template`、plant 函式字頭 `plant_`；紀錄不寫本機路徑。MATLAB 重新產碼只差時間戳記與暫存路徑；新模型版本四道檢查 PASS；離線測試全過；SITL 兩個情境和改名前相同（範本 PASS）；`02_template` 重編後 NuttX `8d3ed2a2…` 不變；FC（01）`hitl-run` 七部分全 PASS：盤旋 60.07 s、2.77 圈、0 次晚到、最慢 0.20 ms，記錄下載兩次相同、嚴格解析 PASS |
| 風（2026-10-19） | 模型的風開成外部接口（定常風，或風切＋陣風＋亂流；預設離地後才吹），SITL 與 HITL 同一個 `wind` 格式（`plant/wind.py`），說明在 [plant README](plant/README.md)「風」、AI 對使用者的說明清單在 AGENTS。原始 slx：Dryden 亂流取消註解（原本因地面空速 0 除以零而關掉）、空速輸入加 5 m/s 下限、雜訊間隔 0.1 s → 4 ms。無風時結果和之前逐位元相同；三組 Simulink 對照（含全部風都開）原始模型與核心副本逐位元相同；四道檢查 PASS（`skysurfer-1c2b65fe…`，HIL 函式庫的 wind case 和 Simulink 一致）；離線測試全過；SITL 參考情境與範本和之前相同，`wind_steady`、`wind_low_altitude` 照預期 |
| 有風實機 HITL（2026-10-19，FC 是 `01_sdfix`） | 場景 `hitl/scenarios/wind_steady_5.json`（地面開始＋定常風 5 m/s 從北方吹來，離地 5 m 後 5 s 漸進）；手動流程：`recover` → `model-start --scenario` → `takeoff --altitude 100` → 盤旋 60 s → `model-stop` → `recover --hil-force-disarm`。流程：盤旋 60 s、1.87 圈、0 次晚到（最慢 1.13 ms）、序列埠 0 錯；恢復後未解鎖、`SYS_HITL=1`、無待恢復；飛控記錄（第 53 筆，比飛行前最新的 52 新）下載兩次相同、嚴格解析 PASS。風：PX4 的 EKF（約每秒一筆）在爬升中約 20 s 估到 4.7 m/s，100 m 以上平均 5.01 m/s 從 0°（真值 5.00 m/s 從 0°）；空速計選擇器自己的風估計約 40 s 後到 5 m/s 附近。和前一次無風（2026-10-18，01）並排：圈數 1.87／2.77、半徑 78／83 m、高度 98–102／100–113 m、盤旋空速平均 17.3／24.3 m/s（有風這次後段約 15.5）、飛控記錄在解鎖瞬間有一段 24 ms 空缺／無；只列差異，沒有事先講好的判定 |
| 可攜產碼（2026-10-20） | 產碼關掉指令集擴充（MATLAB 對 Intel 預設用 SSE2，arm64 編不過；每一格其實就是一般的雙精度運算）。Simulink 參考結果逐位元相同；新 release `skysurfer-75a7ddff…` 四道檢查 PASS，核心與 C 真值對參考資料的每個誤差都和上一版一位不差；配平與 20 s 放手每一位相同；SITL 參考情境與範本和之前相同。HITL（`01_sdfix`，沒有即時權限）`hitl-run`：盤旋 60.1 s、2.78 圈、半徑 82 m，身分、參數前後、飛控紀錄、資料 PASS；時序 FAIL（23,297 步裡 11 步晚到，最慢 16 ms；當時 P 核心被別的計算佔滿） |
| HITL 換模式（2026-10-20，FC 是 `01_sdfix`） | 驗證「飛起來以後 PX4 換模式，飛手不放手」：進 Hold 後從 QGC 那條路依序要求 Stabilized、Hold、Position、Hold，各約 30 s。飛控紀錄：模式照要求換，期間沒有 failsafe、沒有遙控斷訊、沒有 RTL；飛手在 Stabilized 平飛定高，其他模式送中立搖桿。紀錄是這趟的、沒有改參數、恢復完成 |
| 第二台 Ubuntu、沒有 MATLAB（2026-10-20） | 新裝的 Ubuntu 24.04.5 筆電（18 執行緒，沒有 MATLAB、沒有即時排程權限）。uv＋venv；PX4 原樹被別的工作改過，另複製一份乾淨的 v1.17.0；PX4 的 Python 另建 venv 放 PATH。`update-model` 原本因找不到 MATLAB 停下（生成碼要 `rtw_solver.h`、`rtw_continuous.h`），改用專案的最小標頭檔（`plant/core/rtw/`）後四道檢查 PASS（`skysurfer-3cbb58ff…`），60 秒節拍 0 次晚到；對錄製參考資料最大誤差 6.2e-15（靜止時的攻角除外），和用 MATLAB 標頭檔的版本相同。SITL 參考情境的指標和紀錄差 ≤ 1 步（4 ms）／0.005°，範本 PASS；離線測試 15 個檔全過 |
| 換飛控：Cube Orange+（2026-10-20，同一台筆電） | 三個配方（`*_cubeorangeplus`）一條龍全 PASS，交接時 FC 上是 `01_sdfix_cubeorangeplus`。原生：盤旋 60.16 s、2.78 圈、半徑 82.2 m、高度 99.8–113.2 m（6C 基準 60.06 s、2.78 圈、82.3 m、100.0–113.2 m），23,079 步 0 次晚到、最慢 0.35 ms（沒有即時權限、機器閒置）。範本：FC 接受模組自己發的 OFFBOARD；航向 +29.15°、高度 +9.39 m、超越 18 %，判定 PASS，但這張 SD 卡的記錄清單沒有 `tecs_status`，高度基準是實測高度（6C 用 TECS 目標），不能直接比。和 6C 不同而要改的：預設韌體沒有 `pwm_out_sim`（改用 `hil` 板子變體 patch）、FC 回報的 board_version 是 USB 產品碼 4184（不是 board id 1063）、出廠 ArduPilot bootloader 的 USB 名稱 `CubeOrange+-BL` 原本認不出來、用過的 FC 有 16 個殘留參數（以一次性配方改回）。細節見 [fc/cubeorangeplus.md](docs/fc/cubeorangeplus.md) |
| 乾淨目錄試跑 | 2026-10-18（改名後）：從 git 拉到新資料夾、獨立快取、不連 MATLAB：`update-model` 建出**相同的 build-id** `skysurfer-6d1cf7a3…`、四道檢查 PASS；離線測試全過；**從零重編 `01_sdfix`**：PX4 `d6f12ad1…`、NuttX `8d3ed2a2…`、韌體大小都和 FC 上的相同（`.px4` 含建置時間，所以檔案 SHA 不同）；紀錄裡沒有本機路徑 |
| 收尾 | 核心精神已寫進本 README（「這套專案的精神」）；乾淨目錄試跑完成；上 GitHub：https://github.com/rhyne1012/px4-llm-sim2hitl（2026-10-18） |

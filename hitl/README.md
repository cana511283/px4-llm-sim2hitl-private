# hitl：HITL runtime

載入驗收過的模型 library，以 4 ms 步進執行，經 USB MAVLink 和實體 PX4 做 HITL。入口與常用指令見[根目錄 README](../README.md)（`./runbridge --help` 列出所有動作），安全規則見 [AGENTS](../AGENTS.md) 第 3、4 節。本文只寫 runtime 的位置、規則與測試。

## 環境

只在 Ubuntu 24.04 x86-64 驗證過，其他平台沒有驗證（換平台時照根目錄 README「環境準備」查官方文件）。

| 項目 | 內容 |
|---|---|
| 入口 | 根目錄的 bash 腳本 `./runbridge`：`update-model` 交給 `hitl/update.py`，`where`（這台機器實際的位置）交給 `hitl/where.py`，其他動作交給 `hitl/main.py` |
| Python | `./runbridge --setup`（要 uv）依 `pyproject.toml`／`uv.lock` 建 venv，位置見 `./runbridge where venv`（Linux 是 `~/.local/share/px4-llm-sim2hitl/venv`）；`SIM2HITL_VENV` 改位置，`SIM2HITL_PYTHON` 指定直譯器。套件：pyserial 3.5、pymavlink 2.4.49、numpy 2.0.2 |
| 檢查 | `./runbridge --check-python` 只檢查、不安裝；找不到 Python 時退出碼 4 |
| 模型 release | `./runbridge update-model --generate`（要 MATLAB）→ `./runbridge update-model`（編譯、四道檢查、發布）。release 在 `hitl/releases/skysurfer/<平台>/<build-id>/`（平台見 `./runbridge where platform`，例如 `linux-x64`、`macos-arm64`），用哪一個由 `hitl/active/<平台>.json` 指定；兩者都在本機建、不進 git。release 含模型與 worker（`libskysurfer`、`libhil_worker`、`libskysurfer_plant`；Linux 是 `.so`、macOS 是 `.dylib`），只依賴系統 C 函式庫，跑 runtime 不需要 MATLAB。`./runbridge update-model --rollback` 切回前一個 |
| FC 序列埠 | `/dev/ttyACM*`（使用者要在 `dialout` 群組）；`--port` 預設 `auto`，依 USB 廠商碼與裝置名稱挑出 FC。FC 在 bootloader 時不連上去：名稱規則在 `host/serial_helpers.py` 的 `BOOTLOADER_LABEL`（PX4 的 `PX4 BL …`、CubePilot 出廠 ArduPilot bootloader 的 `CubeOrange+-BL`，它和韌體同一個 USB VID/PID） |
| QGC 與自動飛手 | 只走本機 UDP：owner 從 127.0.0.1:14575 送到 QGC 的 14550；自動飛手是 14576 → 14560 |

- 模型 ABI：`interface/hil_model_api.h` 是權威；ABI 版本 65537（0x00010001），C 自然對齊、不 pack；Initial／Input／Output／Info 為 120／176／680／280 bytes。SkySurfer 的實作是 `aircraft/skysurfer/adapter.c`。
- `./runbridge update-model --fc-only` 發布只有 worker、沒有模型的 release，給不能編模型的機器用：owner 照樣能連 FC（`status`、參數讀回、`fetch-log`、`sd-*`、`recover`），需要模型的動作會被拒絕（`missing_conditions` 出現 `model release (FC-only on this platform)`）。平常用不到。

| 位置 | 內容 |
|---|---|
| `host/` | owner（`core.py`）、本機 IPC（`ipc.py`）、`hitl-run`、`deploy-test`、參數與記憶卡傳輸 |
| `native/posix/` | 4 ms worker（`worker.c`）與序列埠寫入、wire 編碼（`io.c`） |
| `aircraft/skysurfer/` | 模型 ABI 實作 `adapter.c`、`profile.json`、場景檢查 `scenario.py` |
| `autotakeoff/` | 自動飛手（起飛、爬升、平飛），見 [autotakeoff README](autotakeoff/README.md)；數字在 `aircraft/<飛機>/profile.json` 的 `automation` |
| `commands/` | 起飛後動作腳本，見 [commands README](commands/README.md) |
| `scenarios/ground.json` | 預設場景（地面靜止） |
| `flight_test.json` | `test-takeoff-hold`／`hitl-run` 的預設飛行與判定 |
| `update.py`、`matlab/mcp_build.py` | 模型 release 的兩段都由 `update.py` 做；第一段經 MATLAB MCP 產碼時用 `mcp_build.py` |
| `poc/source_columns.json` | 模型 58 個 raw 欄位的名稱 |
| `tests/` | 離線測試、負載測試、library 驗證 |

## 狀態資料夾

cache 下的 `runtime/<hash>/`（這台機器的位置：`./runbridge where runtime_state`），`<hash>` 是 `hitl/` 資料夾絕對路徑的 SHA-256 前 12 碼（`host/ipc.py` 的 `state_path`）。所以換資料夾就是另一份狀態：換之前先在原處恢復並關 owner（AGENTS 第 3 節）。cache 的位置依平台而定（`plant/core/toolchain.py`：Linux 是 `~/.cache/px4-llm-sim2hitl`，macOS 是 `~/Library/Caches/px4-llm-sim2hitl`），設了 `SIM2HITL_CACHE`（Linux 另有 `XDG_CACHE_HOME`）時跟著改。

| 檔案 | 內容 |
|---|---|
| `owner.lock` | 單一 owner 的鎖 |
| `host.json` | owner 的本機 IPC 埠與 token（權限 0600） |
| `recovery.json` | 待恢復記號；只由 runtime 寫，不可手動刪改 |
| `param_deploy.json` | 寫參數的交易記號；未完成時模型不能啟動 |
| `logs-<時間>/` | 每次 owner 的紀錄：`wire.bin`、`timing.csv`（每一步）、`io_events.jsonl`、`clock_sync.json`、`final_status.json`、`pilot/`（自動飛手） |

- IPC：127.0.0.1 的 TCP，請求與回覆各一行 JSON；請求上限 64 KiB、回覆 4 MiB。同時最多 16 個還沒結束的操作（再多回「Owner busy」），已結束的保留最近 32 個。`status` 的 `operations` 只有摘要；完整結果用 `operation` 請求帶 `request_id` 取，`host/ipc.py` 的 `request()` 在操作結束時會自動取。
- `hitl-run` 的證據另存在 cache（`./runbridge where cache`）的 `hitl_runs/`。

## 執行規則

- **步進**：ode4、4 ms，輸出取積分前的 t_k；單實例。worker 執行緒在 Linux 要求 SCHED_FIFO 優先權 2，要有即時權限（根目錄 README「即時排程權限」）；結果在 `./runbridge status` 的 `scheduler.time_constraint_result`（0 = 成功，否則是錯誤碼）。這是 soft real-time。
- **時序**：落後 ≥ 20 ms 就 fault；落後較少時立刻接著跑，不跳步。每一步記在 `timing.csv`；落後 ≥ 4 ms 算一次晚到（status 的 `misses`）。
- **controls**：只採用有限且在 [-1, 1] 的值；收到前為 0，之後保持最後一筆。超過 100 ms 沒更新時 status 的 `controls_stale` 為 true，≥ 2 s fault；模型開始後 5 s 內沒有有效 controls 也 fault。模型只用 `controls[0..4]`（左副翼、右副翼、升降舵、方向舵、油門）。
- **啟動模型的條件**：FC heartbeat 新鮮（≤ 3 s）、讀回 `SYS_HITL=1`、確認未解鎖、沒有待恢復、沒有未完成的寫參數。缺什麼看 status 的 `missing_conditions`。
- **恢復**：送過感測器後，停模型不等於 PX4 已恢復，要 `./runbridge recover`：停飛手與模型 → 一般 disarm → 重開 PX4 → 等同一台 FC 回來、未解鎖、重新讀回 `SYS_HITL=1`。任何一步失敗就保留待恢復。強制 disarm（`recover --hil-force-disarm`）只在一般 disarm 被明確拒絕、模型與飛手都已停止時才會用；`test-takeoff-hold` 收尾會加它，因為那時 PX4 還在空中，一般 disarm 通常被拒。
- **送給 FC 的感測器**：HIL_SENSOR 每步（250 Hz）、HIL_GPS 10 Hz、heartbeat 1 Hz，編碼在 `native/posix/io.c`（`host/packets.py` 是同樣規則的 Python 版，測試核對兩者一致）。模型輸出的單位：加速度 m/s²、角速度 rad/s、磁場 µT、氣壓 Pa；GPS 度／MSL m、NED m/s、航跡角度。送出時只換算一次：磁場 ×0.01 → gauss、氣壓 ×0.01 → hPa、經緯度 ×1e7、高度 m → mm、速度 m/s → cm/s、航跡角 → 0.01°。氣壓高度與溫度由絕對氣壓以 ISA 推出。
- **GPS 品質**是固定的模擬值，不是實際觀測：fix 3、衛星 10、eph／epv 30／50 cm（寫在 `native/posix/io.c` 與 `host/core.py`，`profile.json` 的 `gps_quality` 同值）。
- **場景**（`aircraft/skysurfer/scenario.py`）：可給離地高度 0–10000 m、TAS 0–200 m/s（加攻角／側滑 ±45°）或機體速度、Euler（roll ±180°、pitch ±80°、yaw ±360°）、緯經度。地面 MSL、geoid、初始角速度、亂數種子只接受 0，其他值拒絕。風用場景的 `wind_ned_m_s`（定常風）與 `wind` 物件（起風方式、陣風、風切、亂流；格式與提醒在 `plant/wind.py`），模型有風功能時由 `hil_model_configure_wind` 設定，提醒放在 `status` 的 `wind_reminders` 與每次 run 的紀錄。實體 FC 只能從地面靜止開始；非零高度或速度只能用 fake FC 離線驗證。
- **發散保護**（`adapter.c`）：加速度 > 1000 m/s²、角速度 > 100 rad/s、GPS 速度 > 200 m/s 或出現非有限值就 fault，要重設；不裁切、不歸零、不改飛機來掩蓋。

## 用 QGC 搖桿手動飛（3.1）

QGC 的搖桿（畫面上的虛擬搖桿，或 USB 遊戲手把）經 UDP 送 MANUAL_CONTROL，owner 原樣轉給 FC：HITL 沒有實體遙控器，**這個搖桿就是 PX4 的遙控**。自動飛手在飛時，QGC 的搖桿先交給飛手；搖桿動超過約 15 %（150/1000），飛手就放手、把控制交給 QGC（[autotakeoff README](autotakeoff/README.md)）。2026-10-20 在 Cube Orange+（`01_sdfix_cubeorangeplus`）上準備。

**一次性設定**

- QGC 只連 UDP：`~/.config/QGroundControl/QGroundControl.ini` 的 `[AutoConnect]`（根目錄 README「環境準備」）。改之前 QGC 要關著。
- 虛擬搖桿：QGC 左上 **Q 圖示 → Application Settings → General → Fly View** 區段，勾 **Virtual Joystick**；回到飛行畫面會出現兩個圓盤（mode 2：左邊上下是油門、左右是方向舵；右邊是俯仰與滾轉）。有「Auto-Center Throttle」時不要勾（固定翼的油門不回中）。
- USB 手把：插上後 **Q 圖示 → Vehicle Setup → Joystick**，在 Active joystick 選它，**Calibrate → Start** 照提示動桿，最後啟用。PX4 的 `COM_RC_IN_MODE` 不能是 0（預設 3：遙控或搖桿、先到先用）。

**一鍵版**：`hitl/qgc_session.sh`（Linux）把下面的「每次飛」串成一次：開 owner（沒開才開）→ 檢查 FC（未解鎖、`SYS_HITL=1`、沒有待恢復；有待恢復先做一次標準 `recover`；模型已經在跑就停下說明，不替它停：可能是別的 session 或視窗正在飛）→ 開 QGC（設定檔沒關 USB 自動連線就不開，並說明原因）→ 模型上地面（`--scenario` 可換場景，例 `hitl/scenarios/wind_steady_5.json` 在風裡飛）→ 問要不要自動起飛 → 你飛，視窗每 3 s 顯示高度、空速、模式、晚到次數 → 按 Enter、Ctrl-C 或關視窗都會停模型、`recover --hil-force-disarm`、檢查、留下這趟的紀錄、關 owner；收尾開始後再按 Ctrl-C 或關視窗，收尾照樣做完。任何一項沒準備好就停下說明，不繞過。選項見 `hitl/qgc_session.py --help`（`--altitude`、`--scenario`、`--no-takeoff`、`--no-qgc`、`--no-log`、`--keep-owner`）。要點兩下就跑，建一個桌面捷徑（`.desktop`，例：`Exec=gnome-terminal -- <專案>/hitl/qgc_session.sh`），放在 `~/.local/share/applications/` 或桌面；放桌面的要允許執行（右鍵 Allow Launching）。捷徑的路徑屬於這台機器，不進專案。

- **這趟的紀錄**：cache（`./runbridge where cache`）的 `manual_flights/flight_<時間>/session.json`：場景、有沒有自動起飛、模型版本、owner 的 `wire.bin`／`timing.csv` 在哪、飛控記錄（和 `hitl-run` 同樣的核對：編號比飛行前最新的新、下載兩次相同、嚴格解析）。沒解鎖就沒有新的飛控記錄，會寫明沒留。
- **一次只能一個**：第二個視窗會停掉第一個的模型、或在它下載飛控記錄時重開 FC（2026-10-20 實際發生：關掉的視窗還在收尾，又開了一個，第一趟的飛控記錄因此沒下載到）。現在用一把鎖（`flock`，程序結束就自動放開）：前一個還在收尾（約 1 分鐘）就等它，還在飛就說明後結束。
- **狀態列不能干擾時序**：在同一個程序裡走 owner 的本機 IPC 查狀態。2026-10-20 第一版每 2 s 開一個新的 `runbridge` 程序查，沒有即時權限時 4 ms 迴圈晚到 1 次（6.1 ms）；改成同程序、每 3 s 之後 0 次（最慢 2.1 ms）。

**人飛 HITL 能看什麼、不能當什麼**

- 能看：手感與模型可不可信（舵量、反應、空速）；PX4 的模式、解鎖與失效保護在實體 FC 上實際怎麼動（SITL 沒有 Commander，驗不到）；起飛降落與操作的練習；自動測試沒想到的情況。
- 不能當：通過／不通過的測試——人飛的是觀察，沒有事先講好的規則就只記數字與發生的事，由人判斷；也不是實機的手感（模型的簡易地面、空速偏高的已知問題）。
- 同一把尺：人飛的也是同一種飛控記錄（ULog）與 wire log，用 `analysis/` 的工具看。還沒做、但順著這個方向的小工具：把一趟人飛的搖桿輸入變成 SITL 情境，比較 SITL 與 HITL 對同一組輸入的反應。

**每次飛**

```sh
./runbridge bridge          # 開 owner（已開著就跳過）
./runbridge status          # fc.state = verified、ready_to_start_model = true、missing_conditions = []
```

1. 開 QGC（例：`~/Applications/QGroundControl-x86_64.AppImage`），等它經 UDP 連上飛機。
2. `./runbridge model-start`：模型在地面開始跑，PX4 開始收到感測器（Preflight Fail 的訊息會消失）。
3. 起飛，建議用自動起飛：`./runbridge takeoff --altitude 100`。滑跑、爬升到 100 m 後在 Stabilized 平飛。
4. 接手：動 QGC 的搖桿，飛手放手，之後由你飛。上方的模式選單：**Stabilized**（姿態增穩，放手回水平）、**Position**（固定翼：滾轉桿轉向、俯仰桿給爬升率、放手保持航向與高度，最好飛）、**Hold**（盤旋）。QGC 的選單切不到固定翼的 Offboard。
5. 結束（停在空中是正常的）：

```sh
./runbridge model-stop
./runbridge recover --hil-force-disarm    # 在空中結束要加；完成後 status 是未解鎖、SYS_HITL=1、沒有待恢復
./runbridge shutdown                      # 不再用就關 owner
```

**要知道的**

- QGC 停止送搖桿（關掉虛擬搖桿、關掉 QGC）超過 `COM_RC_LOSS_T`（0.5 s），PX4 的遙控斷訊保護接手：Hold 5 s 後 RTL。
- 想自己從地面起飛：模式切 Stabilized → 用 QGC 解鎖（點上方的飛行狀態 → Arm，滑動確認）→ 油門推過一半 → 空速約 12 m/s 時拉桿抬頭，爬升角約 12°。PX4 固定翼的 Takeoff 模式在這裡不能用（要跑道起飛設定或手擲偵測）。遙控器設了解鎖開關的 FC（`RC_MAP_ARM_SW > 0`，例如這次的 Cube）沒有搖桿手勢解鎖，一定要用 QGC 的 Arm。
- 不要用 QGC 改參數：配方的核對會判 FAIL（`not_default`），參數部署也會把別的寫入者當成錯誤。要改就改配方再部署。
- 空速偏高、爬高時衝過頭是已知的模型與參數不吻合（根目錄 README「已知問題」），不是搖桿的問題。

## 離線測試

用 fake FC、獨立的狀態資料夾與隨機本機埠，不碰序列埠、QGC 或真的待恢復記號。`PY` 是 venv 的 python（AGENTS 第 5 節）：

```sh
for f in hitl/tests/test_*.py deploy/test_*.py; do "$PY" -B "$f"; done
```

- 設 `FW_HIL_TEST_OUTPUT=<資料夾>` 時，`test_runbridge_fake.py` 會留下每個案例的 CLI、console、wire 與 status。
- 600 s 負載（不開視窗）：`"$PY" -B hitl/tests/run_load.py <新的空資料夾> --duration 600`。PASS 條件寫在 `run_load.py`：每步計算 p99 < 2 ms、最慢 < 4 ms、0 次晚到（≥ 4 ms）、每 10 s 的即時倍率在 0.99–1.01、沒有序列埠錯誤或掉 log。`--peer-mode process` 只用來診斷對照。
- `tests/validate_library.py` 由 `update-model` 呼叫，用 Simulink 參考結果與 ABI 驗證新建的 library。
- `tests/run_live_unarmed.py` 會連實體 PX4（不解鎖），屬實體測試；它的序列埠寫死成 macOS 的 `/dev/cu.usbmodem01`，在 Linux 不能直接用。

## 母機攜帶／釋放（第一階段，2026-10-21）

本機新增的實驗路線，規格與限制見 [carrier_release.md](carrier_release.md)。它仍從地面靜止、未解鎖開始；不放寬實體空中初始化的限制。母機是無風、朝北、水平姿態的運動約束，不是另一架有氣動模型的飛機。模型的 carrier 計時只在實體 FC 明確回報 armed 後開始；未知／未解鎖不啟動攜帶，開始後失去 armed 確認就 fault。沒有改 PX4 的解鎖檢查。

首次實體測試須先完成離線及 SITL，並依 AGENTS 取得首次新算法上 FC 的確認。此路線不沿用「起飛後盤旋」的驗收：`carrier-run` 有界執行、標準停止及恢復、參數前後核對、新飛控記錄歸屬核對／下載兩次／嚴格解析；飛行品質只報告，未約定門檻不能判穩定 PASS。

## 母機釋放後飛五邊與盤旋降落

第二階段的 `carrier-circuit-run` 使用原生 AUTO_MISSION，會暫時部署兩項配平參數及航點；先備份，結束後用原值還原，再核對新 log。首次新任務上 FC 前須人確認。規格、SITL 結果與限制見 [missions README](../missions/README.md)。原來的 `carrier-run` 仍只做釋放後 40 s 姿態觀察。

平面向下測距（選用，25 Hz）見 [地面幾何與測距](../plant/terrain/README.md)，範例 `scenarios/carrier_release_flat_range.json`。新版 release 才可啟用；2026-10-22 已完成 Cube EKF 接收及模擬降落流程驗證，詳見 [任務結果](../missions/README.md)；未評定實機著陸品質。

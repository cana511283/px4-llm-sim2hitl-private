# SITL：精簡 PX4＋plant 模組

這是本套件的 SITL 做法。**不跑完整 PX4 SITL、不走 MAVLink、不啟動 EKF2**：用鎖定版本的 PX4 posix build，只啟動情境需要的 PX4 模組，把 plant 寫成 PX4 模組放在同一個程序裡 lockstep 執行，估測值直接用 plant 的真值。PX4 模組的程式原封不動；不要自己重寫 PX4 模組的接線。

下面的 Python 指令都用專案的 venv（`./runbridge --setup` 建的；配平與分析要 numpy）：

```sh
PY="$(./runbridge where python)"   # 這台機器的其他位置也用 ./runbridge where 查
```

## 架構

```
plant_sim（本資料夾的 PX4 out-of-tree 模組）
  ├─ 擁有 PX4 時鐘：每步 +4 ms，等所有 PX4 work queue 做完（lockstep）
  ├─ plant：libskysurfer_plant（.so／.dylib；核心＋C Truth＋C sensors，與 HITL 模型 release 同一份驗收過的程式）
  ├─ IMU：C sensors 的 accel／gyro → PX4Accelerometer／PX4Gyroscope → PX4 sensors（保留陀螺儀濾波）
  ├─ 取代 EKF：vehicle_attitude、vehicle_local_position、vehicle_global_position、airspeed_validated、
  │            vehicle_air_data、vehicle_land_detected 由 Truth 直接發布
  ├─ stub：vehicle_status、vehicle_control_mode、actuator_armed、manual_control_setpoint（情境的搖桿）
  │       （沒有 Commander：模式與控制旗標由情境直接決定，模組的 OFFBOARD 訊號不會被讀）
  └─ 讀 actuator_outputs_sim（pwm_out_sim，PWM_MAIN_FUNC1..5 = HIL ABI 的 controls[0..4]）送回 plant
PX4 模組（Stabilized）：sensors、pwm_out_sim、control_allocator、fw_rate_control、fw_att_control
```

**SITL 沒驗到的事**：因為沒有 Commander，解鎖檢查、OFFBOARD 怎麼進入與退出、失聯保護（例如 `COM_OF_LOSS_T`）在 SITL 都沒有驗證；SITL 的導引表現 PASS 不能延伸成這些也沒問題，這些要在實體 FC 的 HITL 量。外接電腦送來的 MAVLink 也沒有經過 PX4 的接收：SITL 的 Offboard 是模組直接發布設定值，所以「`SET_ATTITUDE_TARGET` 的遮罩怎麼解讀」這類事 SITL 也驗不到。例：PX4 v1.17 固定翼的角速度模式，偏航角速度標成「忽略」會變成 NaN，`fw_rate_control` 看到任何一軸不是有限值，三軸都只輸出配平值（`FixedwingRateControl.cpp`），等於放手。

目前有三種模式（`run_sitl.py` 的 `MODULES`；`plant_sim` 的模式旗標照 PX4 `commander/ModeUtil/control_mode.cpp`）：

- **Acro**：`fw_rate_control` 為止。只保持「角速度為零」、不保持姿態，FC 的 `TRIM_PITCH=0` 下沒有人操控會一路低頭，所以沒有放無人操控的 Acro 參考情境。
- **Stabilized**：再加 `fw_att_control`。
- **Offboard**：再加 `fw_att_control`、`fw_lat_lon_control`（PX4 自己的高度與速度控制）；導引由 `modules/` 的模組負責（例：`guidance_template`）。

其他模式要在 `MODULES` 加 PX4 模組，並在 `plant_sim` 加對應的 stub。

## 檔案

| 檔案 | 用途 |
|---|---|
| [build_sitl.py](build_sitl.py) | 在 PX4 原始碼樹編 SITL 版 PX4（加 `plant_sim` 與 `modules/` 的模組） |
| [run_sitl.py](run_sitl.py) | 讀情境、算配平、產生參數與啟動腳本、啟動一或多個 PX4 instance、收集結果 |
| [trim.py](trim.py) | 配平（平飛、協調轉彎）與丟出去測試；常態工具，問配平就跑它 |
| [replay.py](replay.py) | ULog 重播：PX4 `replay` 只發布控制器輸入，原生控制器重算輸出；由 `run_sitl.py --replay` 呼叫 |
| [scenarios/](scenarios/) | 情境檔：`stabilized_steps.json` 是原生 PX4 的參考情境；`stabilized_steps_hitl.json` 是它的 HITL 版（HITL 序列與 SITL 預演共用）；`template_offboard.json` 是 `guidance_template` 範例的情境 |
| [px4_external/src/CMakeLists.txt](px4_external/src/CMakeLists.txt) | PX4 `EXTERNAL_MODULES_LOCATION` 的模組清單 |
| [px4_external/src/plant_sim/](px4_external/src/plant_sim/) | `plant_sim` 模組（`PlantSim.cpp`、`PlantSim.hpp`、`CMakeLists.txt`） |
| [px4_external/src/guidance_template/](px4_external/src/guidance_template/) | 包裝：把 `modules/src/guidance_template/` 編進 SITL（新模組照它加一個，並列進上面的清單） |

## 建置

```sh
"$PY" -B sitl/build_sitl.py                      # plant library = 目前發布的模型 release 裡的那份
"$PY" -B sitl/build_sitl.py --plant-lib "$(./runbridge where cache)/build_<時間>/native/libskysurfer_plant.so"   # 例如還沒發布的試建 cache（macOS 是 .dylib）
"$PY" -B sitl/build_sitl.py --replay             # ULog 重播用的 px4_sitl_replay
```

- 先跑 `px4/check_px4_tree.py`（PASS 才編），再在 PX4 原始碼樹執行 `make px4_sitl_default EXTERNAL_MODULES_LOCATION=<專案>/sitl/px4_external BUILD_DIR_SUFFIX=_sim2hitl CMAKE_ARGS=-DPLANT_LIB=<plant library>`。增量編譯，log 在 cache（`./runbridge where cache`）的 `sitl_build_<時間>.log`。
- 輸出在 `<PX4 原始碼樹>/build/px4_sitl_default_sim2hitl/`（git 忽略的 build 目錄，`check_px4_tree.py` 仍 PASS）。不改 PX4 原始碼，不覆蓋 `build/px4_sitl_default`。字尾 `_sim2hitl` 可用環境變數 `SIM2HITL_SITL_SUFFIX` 換：同一台機器有第二份 checkout 共用這棵 PX4 樹時要換（根目錄 README「同一台機器放兩份」）。
- PX4 執行檔照編譯時寫進去的路徑載入 plant library：Linux 是 RUNPATH（預設是模型 release 的資料夾）；macOS 的 library 以 `@rpath/<檔名>` 命名（`plant/core/toolchain.py` 的 `install_name`），CMake 同樣給執行檔一條指到 release 資料夾的 rpath（少了 `@rpath`，macOS 會去載入編譯那份 library 的建置 cache）。編完 `build_sitl.py` 核對執行檔實際載入的檔案（JSON 的 `loads_plant_lib`；macOS 用 otool、Linux 用 ldd），不是指定的那一份就 FAIL。發布新的模型 release 或刪 cache 後重跑本指令，否則跑的還是舊的機體，而且不會報錯（`run_sitl.py` 跑之前也會核對，見下面）。
- `--replay`：`make px4_sitl_replay`，輸出在 `<PX4 原始碼樹>/build/px4_sitl_replay/`。這是 PX4 自己的 board（預設 SITL＋uORB 發布規則 `ORB_USE_PUBLISHER_RULES`），不含 `plant_sim`、不需要 plant library；第一次約幾分鐘。
- 外部模組比 PX4 的 `src/lib` 早設定，所以 `plant_sim` 的 PX4 函式庫用 `target_link_libraries` 依名稱連結，不用 `px4_add_module` 的 `DEPENDS`。

## 執行（離線模擬，不碰硬體）

```sh
"$PY" -B sitl/run_sitl.py --out "$(./runbridge where cache)/sitl_$(date +%Y%m%d_%H%M%S)" [--scenario sitl/scenarios/stabilized_steps.json] [--instances 1]
```

- `--out` 用一個新的資料夾。每個 instance 一個子資料夾：`sitl_rcS`（產生的啟動腳本）、`sitl_params.bson`、`px4.log`、`trace.csv`（每步：時間、姿態、角速度、空速、高度、搖桿、五路 controls，最後是地速 `gs_m_s` 與實際作用的風 `wind_n/e/d_m_s`）、`summary.json`（步數與要求的步數、怎麼結束的 `end`：`complete`／`diverged`（模型發散）／`stopped`、模擬秒數、實際秒數、真實時間倍數），預設另有 ULog（`log/<日期>/*.ulg`，`--no-ulog` 不記）。最後印一行 JSON（各 instance 與合計）。每個 instance 都跑完整個情境，結束碼才是 0：被 `--timeout` 砍掉、重試用完仍卡住（`stalled`）、PX4 不是在 summary 之後正常關閉、沒有 `summary.json`、或 `end` 不是 `complete`（還沒有這個欄位的舊 SITL 執行檔也算：重跑 `sitl/build_sitl.py`），會寫在該 instance 的 `problems`，`run.json` 的 `complete` 是 false，結束碼 2。
- **參數**：`deploy/00_native/` 的 FC 匯出＋HITL 設定，轉成 SITL：`HIL_ACT_FUNC*`／`HIL_ACT_REV` → `PWM_MAIN_FUNC*`／`PWM_MAIN_REV`；`SYS_HITL=0`；FC 的感測器校正與硬體參數不套用，`CAL_ACC0_ID`／`CAL_GYRO0_ID` 設成模擬 IMU。參數寫成 BSON（`sitl_params.bson`），啟動腳本用一次 `param import` 載入（逐一 `param set` 有約 580 次 client 呼叫，多個 instance 同時啟動時偶爾有一次卡住不返回）；SITL build 沒有的參數會在 `px4.log` 印 `ignoring unrecognised parameter`，屬正常。
- **起始**（`PlantSim.hpp` 註解）：(1) 用真實時間跑到 `pwm_out_sim` 開始輸出（所有 work queue 都已建立；更早就 lockstep 會 deadlock）；(2) lockstep、disarmed、plant 固定在初值 100 步（輸出模組只在 disarmed 時套用 `PWM_MAIN_FUNC`）；(3) arm，收到第一筆 armed 輸出後 plant 才開始推進；「armed 輸出」是時間戳不早於 armed stub 發布時間的那一筆（更早的是 disarmed 時算的）。步數＝`duration` / 4 ms 四捨五入。
- **情境檔**（`scenarios/*.json`）：`mode`（`stabilized`／`acro`／`offboard`）、`initial`（高度、空速、航向、經緯度）、`trim`、`settle_s`、`duration_s`、`steps`（`start_s`／`end_s` 從 settle 結束起算，`roll`／`pitch`／`yaw`／`throttle` 搖桿增量）、可選 `base`、`base_sticks`、`start`（要多啟動的 PX4 指令，和 `--start` 合併）、`wind`（風；格式與提醒在 `plant/wind.py`，說明在 [plant README](../plant/README.md)「風」；提醒印在畫面並寫進 `run.json` 的 `wind_reminders`；例子 `wind_steady.json`、`wind_low_altitude.json`）。`run_sitl.py` 把它轉成 `plant_sim` 讀的 `scenario.txt`（格式見 `PlantSim::load_scenario`）。
- **配平**（`trim.py`，見下一節）：空中起始用平飛配平的攻角＝俯仰與油門；油門搖桿＝2×油門−1；Stabilized 另加基準俯仰搖桿，讓設定值等於配平俯仰。SITL 起始只帶入高度、速度、姿態（`PlantInitialConditions` 不含角速度與致動器狀態），所以轉彎配平還不能當 SITL 起始。
- **settle_s**：PX4 的角速度積分器從 0 開始，而配平需要升降舵 0.4（FC 的 `TRIM_PITCH=0`），所以起始後俯仰先掉到約 −6°，約 40 s 回到配平（俯仰差 < 0.25°）；收斂後的升降舵 0.396 與離線配平一致。參考情境先 settle 40 s 再做階躍。量反應前先看 `response.py` 的 `pre_slope_per_s`（階躍前 1 s 的斜率）：不接近 0 就代表還沒穩，那一段的數字不能拿來量反應。
- **參考情境 `stabilized_steps`**：settle 40 s 後，3–6 s roll 搖桿 0.5、9–11 s pitch 0.3、14–16 s yaw 0.5。指標用 [analysis/response.py](../analysis/response.py) 算。參考值（Ubuntu 2026-10-18，與舊機器的紀錄相同）：roll 尾窗平均 +22.32°（設定 22.5°）、上升 0.316 s、安定 0.456 s；pitch 尾窗平均 −3.39°（絕對設定值 −5.25°）、上升 1.06 s、未安定；yaw 角速度峰值 7.81 °/s（0.22 s）。這些值也存在情境檔的 `reference`：`response.py` 在結果加上 `reference_comparison`（每個指標的紀錄值、這次的值與差），只列差異、不判定；和人講好容差後填進 `reference.tolerance`，超出時結束碼 3。
- **`base_sticks`**：絕對的基準桿量（`manual_control_setpoint` 值），取代配平算出的基準；這時 `trim` 只決定空中起始狀態。`base` 是疊加的偏移。
- **HITL 版 `stabilized_steps_hitl`**：同一組階躍與基準桿量（15 m/s 平飛配平，取到 MANUAL_CONTROL 的 1/1000：pitch −0.125、油門 −0.77），但 HITL 是從起飛後的巡航（約 25 m/s）進入，所以 settle 60 s。在 SITL 跑就是那趟 HITL 的預演（25 m/s 平飛配平起始）；舊機器上量到約 50 s 回到 15 m/s、高度掉約 16 m、階躍指標和參考情境差 ≤ 0.02°／0.02 s。HITL 由 `hitl/commands/stabilized_steps.py` 飛（起飛後動作 `--after stabilized_steps`）。
- **`--start CMD`**（可重複）：在該模式的 PX4 模組之後、`plant_sim` 之前多啟動一個指令，例如 `--start "guidance_template start"`（`modules/src/guidance_template` 的範例模組；SITL 由 `px4_external/src/guidance_template/` 的包裝編進來）。情境 `template_offboard.json` 已在 `start` 寫了它，所以 `--scenario sitl/scenarios/template_offboard.json` 不用再加；判定用 `"$PY" -B analysis/template_check.py <out>/instance_0/log/<日期>/<檔>.ulg`。要做 A/B 時，基準要明確指定一個提供同樣任務目標的原生模式與設定值來源；在 OFFBOARD 情境只拿掉這個模組，等於拿掉導引的來源，不等於原生 PX4。
- **跑的是哪一份機體**：啟動前看這個 SITL 執行檔實際會載入哪一份 plant library（`build_sitl.py` 的 `loaded_library`：macOS 用 otool、Linux 用 ldd），和目前發布的模型 release 不同就在畫面提醒（不擋：用 `--plant-lib` 刻意試建時本來就不同）；兩者都寫進 `run.json` 的 `plant_library`。
- **多個 instance**：`--instances N` 平行跑，啟動間隔 `--stagger`（預設 1 s）。PX4 的 instance 編號從 0 起（`/tmp/px4-sock-<編號>`），兩次執行同時跑時用 `--instance-base N` 錯開，否則會互相干擾。PX4 的 server 執行緒在多個同時啟動時可能還沒準備好，所以啟動腳本第一行先等 `px4-ver` 有回應。每個 instance 是獨立的程序群組：在 `--startup-timeout`（預設 60 s）內沒有建立 `trace.csv`（代表 `plant_sim` 沒起來）、`trace.csv` 超過 `--stall-timeout`（預設 60 s）沒有變長，或結束時沒有 `summary.json`，runner 把它連同整個程序群組結束再重啟（`--retries`，結果的 `retries` 欄）；只結束 PX4 的話，它的啟動腳本會繼續對同編號的下一個 PX4 下指令。重試用完仍卡住的記為 `stalled`，不等到 `--timeout`。失敗的那次整個資料夾改名留成 `instance_<i>_attempt<n>`（證據不蓋掉，結果的 `failed_attempts` 欄），新的一次在原來的 `instance_<i>` 從頭跑。程序群組收不到終端機的 Ctrl-C：runner 被 Ctrl-C、SIGTERM 或 SIGHUP 停掉時，先結束所有 instance 的群組再離開。`px4.log` 是區塊緩衝寫入，PX4 結束前讀不到完整內容，不能用來即時判斷。
- **實測速度（Ubuntu 24.04、i7-14700 28 執行緒，2026-10-18，`template_offboard` 情境模擬 60 s、`--no-ulog`）**：一次完整跑完（含 PX4 啟動與關閉）約 1.3 s；`--instances 25 --stagger 0.1` 跑 1000 次共 140.8 s、全部成功、0 次重試（其中 100 s 是啟動間隔）；`--instances 100 --stagger 0.02` 一批 5.5 s，換算 1000 次約 55 s（約 1,100 倍真實時間）。大量批次用 `--no-ulog`、較短的 `--stagger`；換電腦先量一批確認沒有重試。
- **速度欄位**：每個 instance 的 `realtime_factor` 是它自己的「模擬秒數／實際秒數」；`throughput_realtime_factor` 是所有 instance 的模擬秒數合計，除以第一個開始推進到最後一個結束的實際時間，只有在各 instance 確實同時在跑時才代表平行吞吐量（要用夠長的 `--duration`）。

## 配平與丟出去（常態工具）

使用者問「配平多少」時直接跑，不用窮舉：

```sh
"$PY" -B sitl/trim.py                 # 預設 15、20、25 m/s，100 m 平飛
"$PY" -B sitl/trim.py --airspeed 18 --bank 20 --height 150
```

- **解法**：plant 核心的平衡點（`plant_core_eval` = f(x, u)，與模擬同一段程式，[plant README](../plant/README.md)）。給空速、高度、坡度（0 = 平飛；非 0 = 協調轉彎，轉彎角速度 g·tanφ/V），解攻角、側滑、俯仰、副翼（兩路同一命令，同 HITL）、升降舵、方向舵、油門與 5 個致動器狀態，使機體加速度、角加速度、致動器導數與垂直速度為 0。協調轉彎只要求轉彎角速度與這些平衡條件，**沒有另外要求側滑為 0**（15 m/s、20° 時側滑 −0.60°）。阻尼 Newton，三個空速約 0.3 s。
- **舵面後有 rate limiter**（記住前一次輸出的離散狀態），所以每次求值把同一狀態寫入並步進 40 次，讓 limiter 到達平衡偏角，再取導數。
- **丟出去**（`--drop`，預設 20 s；0 = 不做）：從配平狀態（limiter 已到位）開始，舵面與油門固定在配平值、沒有任何控制，回報 20 s 後的空速、高度、俯仰、滾轉變化與轉彎角速度。配平正確時漂移只來自很慢的模態；配歪會明顯漂移。
- **輸出**：表格，SI 為主並附 kts／ft；`--json` 輸出全部欄位（含舵面偏角、狀態向量、轉彎半徑、負載因數）。
- **plant library**：預設是目前發布的模型 release 裡的那份（第一行會印出路徑）；不需要 PX4 或 SITL build。
- **目前的配平**（release `skysurfer-f53eee94…`，100 m／328 ft，無風；命令為正規化值，升降舵 + = 抬頭）：

  | 空速 | 坡度 | 攻角 | 俯仰 | 側滑 | 升降舵 | 副翼 | 方向舵 | 油門 | 轉彎角速度 |
  |---|---|---|---|---|---|---|---|---|---|
  | 15 m/s（29.2 kts） | 0° | 3.76° | 3.76° | 0.19° | 0.3975 | −0.0005 | −0.0074 | 0.1147 | — |
  | 20 m/s（38.9 kts） | 0° | 1.27° | 1.27° | 0.08° | 0.1540 | −0.0001 | −0.0020 | 0.1910 | — |
  | 25 m/s（48.6 kts） | 0° | 0.12° | 0.12° | 0.03° | 0.0770 | 0.0000 | 0.0001 | 0.2874 | — |
  | 15 m/s | 20° | 4.09° | 3.64° | −0.60° | 0.5011 | 0.0153 | 0.0757 | 0.1153 | 13.6 °/s |
  | 20 m/s | 20° | 1.46° | 1.45° | 0.25° | 0.1745 | 0.0037 | 0.0057 | 0.1920 | 10.2 °/s |
  | 25 m/s | 20° | 0.24° | 0.30° | 0.22° | 0.0902 | 0.0015 | −0.0002 | 0.2885 | 8.2 °/s |

  模型或氣動表改了之後重跑，數值以重跑結果為準。
- **待補**：飛行品質（線性化後的短週期、長週期、荷蘭滾、螺旋、滾轉模態）。`plant_core_eval` 已可做數值線性化。

## ULog 重播

證明 SITL 裡的 PX4 控制器和 FC 上的算出一樣的輸出：把 ULog 記下的**輸入**原樣餵給同一版本的 `fw_att_control`、`fw_rate_control`、`control_allocator`，重算**輸出**，逐筆和 ULog 記下的輸出比。不跑 sensors、EKF2、commander、plant，只驗控制器。**這台 Ubuntu 還沒建 `px4_sitl_replay`、沒跑過重播**；下面的數字是舊機器（2026-10-16）上量的。

```sh
"$PY" -B sitl/build_sitl.py --replay                                    # 第一次
"$PY" -B sitl/run_sitl.py --replay <flight.ulg> --out <目錄> [--speed 5] [--tolerance 1e-3]
"$PY" -B sitl/run_sitl.py --out <目錄> [--logger-topics <清單>]          # SITL 自己也記 ULog（預設），自我驗收用
```

- **做法**（`replay.py`）：PX4 的一般 `replay` 模組照 log 時間發布、設定 lockstep 時鐘；`<out>/orb_publisher.rules` 限定它只發布 `replay.INPUTS`（控制器訂閱的上游主題），所以 log 裡的輸出不會被重發。規則檔在 PX4 啟動時就讀，所以先寫好再啟動。參數用 log 裡的（`replay tryapplyparams`）。重算的輸出由 logger 記到 `<out>/log/`，比對結果在 `<out>/replay_report.json`（[analysis/replay_compare.py](../analysis/replay_compare.py)）。
- **配對**：有 `timestamp_sample` 的主題（力矩、推力、舵面）依它配對；沒有的（姿態與角速度設定值）用 `timestamp` 減重播偏移量後找 1 ms 內最近的一筆（FC 的 log 和重播的時間戳不會完全相同）。**結果依 log 的 PX4 模式分段**（報告的 `modes`：各段時間、缺的輸入、`covered`）：`covered` 要同時滿足 (1) 模式是 manual、acro、stabilized（`COVERED_MODES`；AUTO 模式的姿態設定值來自沒跑的位置控制器）且 (2) 那段裡 `REQUIRED_INPUTS`（姿態、角速度、搖桿、空速）都有樣本、`vehicle_control_mode`／`vehicle_status` 在段落開始前已有。每個輸出另報 `paired_fraction`（有配到的比例；沒配到的不比）。兩邊都整欄 NaN 才算沒用到的欄位；一邊 NaN、另一邊 Inf 算不一致。JSON 裡非有限值寫成字串 `"Infinity"`。報告每個欄位的完全相同比例、中位數、p99、最大差，以及超過 `--tolerance` 的樣本與時間。**不判 PASS／FAIL**，容差另外和使用者約定。
- **SITL 的 ULog**（預設開，`--no-ulog` 關）：logger 用 HITL 的記錄設定（`SDLOG_PROFILE=17`），在 `plant_sim` 之後啟動（logger 只收啟動時已存在的 optional 主題，例如 `airspeed_validated`），`-p vehicle_angular_velocity` 讓每個角速度樣本寫一次。記 ULog 時較慢（這台單一 instance 約 106 倍真實時間，不記約 146 倍）。`--logger-topics` 用 PX4 的自訂清單（每行「主題 間隔ms instance」）取代 profile。
- **自我驗收**（SITL 記的 ULog 重播回去，參考情境 60 s；舊機器）：
  - 輸入全速記錄（自訂清單，含空速）：力矩差中位數 1.4e-5、p99 3.8e-4；超過 1e-3 的只有 7 筆，都在搖桿階躍的邊緣（同一時間戳的搖桿與姿態，重播時發布順序和原本不同，差一個樣本）。
  - **`SDLOG_PROFILE=17`（空速只記 5 Hz）**：中位數 6.0e-4、p99 5.5e-3，約 21 % 的樣本超過 1e-3。這是 `FW_ARSP_SCALE_EN=1`（增益隨空速縮放）時空速在兩筆之間被保持造成的。要把 FC 重播的差壓到 1e-4 等級，FC 的 SD 卡要放自訂清單（`etc/logging/logger_topics.txt`）讓空速全速記錄。
  - 速度：5 倍和 1 倍結果相同（60 s log 約 28 s）；10 倍中位數略大（7.7e-5）。預設 5 倍。
- **自己把 PX4 的程式搬到別處、要逐位元對得上時**：PX4 用 `-freciprocal-math` 編（`cmake/px4_add_common_flags.cmake`），除以常數會被換成乘倒數，照著寫才會一位不差；控制分配的增益是 PX4 用 float 算反矩陣的結果，不一定剛好是 1（一個雙副翼機型實測副翼 ±1.00000024、馬達 0.99999994）。

## 已知限制與判讀

- **不是逐位元可重現**：PX4 模組分在幾個 work queue 執行緒，同一步內的先後不固定。同設定重跑 4 次，roll 最大差 0.001 rad；比對 SITL 結果要用容差，不要求 trace 相同。
- **右副翼反向是推得的**：HITL 的 wire log 顯示 controls[0] 與 controls[1] 在飛行中完全相同，而 plant 兩個副翼通道都是「正值 → 左滾」，所以 FC 必定反向了一路；`deploy/00_native/hitl_overrides.params` 用 `HIL_ACT_REV=2`（右副翼）表示；2026-10-16 由 FC 的 ULog 參數讀回確認。
- **空速偏高**：PX4 的 `FW_THR_TRIM=0.6` 和這個模型（15 m/s 平飛約 0.115 油門）不吻合，見 [README](../README.md)「已知問題」。

## 實驗：母機攜帶後釋放

`carrier_release_stabilized.json` 由地面零狀態起始，`carrier` 指定等待／爬升／水平攜帶時長、釋放高度與速度。共用 plant 的 `plant_carrier_step`，僅在顯式選用的情境啟用；不改原生情境。`trace.csv` 多一欄 `carrier_attached`。SITL 的 arming 仍是 stub，不能代表實體 FC 會正常解鎖。完整規格與第一版限制見 [carrier_release.md](../hitl/carrier_release.md)。

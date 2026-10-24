# PX4：版本鎖、原始碼樹、韌體建置

PX4 原始碼樹很大，**不放在專案裡**，放在專案外。本資料夾記錄用哪個版本、怎麼找到這台機器的原始碼樹、怎麼編 FC 韌體。

| 檔案 | 內容 |
|---|---|
| [px4_lock.json](px4_lock.json) | 版本鎖：repository、tag、commit、FC 韌體說明、本機位置設定檔在哪 |
| [check_px4_tree.py](check_px4_tree.py) | 核對這台機器的原始碼樹是否等於版本鎖（commit、沒有修改、submodule）；其他程式也用它找原始碼樹 |
| [build_firmware.py](build_firmware.py) | 編部署配方的 FC 韌體（`./runbridge build-firmware --release deploy/<配方>` 呼叫它） |
| [patches/](patches/) | 套在 PX4 上的修正：NuttX 的記憶卡讀取修正（原因見 [AGENTS](../AGENTS.md) 第 1 段）；Cube Orange+ 的 `hil` 板子變體（預設韌體沒有 HITL 要的 `pwm_out_sim`，見 [fc/cubeorangeplus.md](../docs/fc/cubeorangeplus.md)） |

目前鎖定 **v1.17.0**（commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`）。SITL、FC 韌體與 HITL 都用這一個版本。

## 取得原始碼樹（第一次）

- 使用者已經有這個版本的樹：直接用，照下一節設定位置。
- 沒有：下載**鎖定的 tag** 到**專案外**。放哪裡不清楚就問使用者。不要用 main 或其他版本。

```bash
git clone --branch v1.17.0 --recurse-submodules https://github.com/PX4/PX4-Autopilot.git <專案外的路徑>
python3 px4/check_px4_tree.py --path <專案外的路徑> --remember
```

- 工具鏈用 PX4 官方腳本：`bash <樹>/Tools/setup/ubuntu.sh --no-sim-tools`（見 README「環境準備」）。

## 這台機器的原始碼樹在哪裡

依序從這三處取得：

1. 指令參數 `--path`（只有 `check_px4_tree.py` 有）
2. 環境變數 `SIM2HITL_PX4_ROOT`
3. 本機設定檔（只有一行路徑）：Linux 是 `~/.config/px4-llm-sim2hitl/px4_root`（有設 `XDG_CONFIG_HOME` 時是 `$XDG_CONFIG_HOME/px4-llm-sim2hitl/px4_root`）。macOS 是 `~/Library/Application Support/px4-llm-sim2hitl/px4_root`（2026-10-20 在 macOS 用過）；程式裡另有 Windows `%LOCALAPPDATA%\px4-llm-sim2hitl\px4_root`，沒有驗證過。這台機器用哪一個：`./runbridge where px4_config`。

```bash
python3 px4/check_px4_tree.py
```

| 結果 | 意思 | AI 該做什麼 |
|---|---|---|
| `PASS`（exit 0） | 原始碼樹等於版本鎖 | 可以用 |
| `FAIL`（exit 1） | commit 不同、有未提交的修改或多出的檔案、或 submodule 不符 | 停下回報，不要自行 checkout、reset 或清理 |
| `UNKNOWN_LOCATION`（exit 2） | 這台機器還沒設定位置、路徑不存在（外接碟沒掛上），或那裡不是 git 樹 | 問使用者原始碼樹在哪裡（沒有就照上一節下載），再執行下一個指令 |

```bash
python3 px4/check_px4_tree.py --path <路徑> --remember
```

`--remember` 只在核對 PASS 後寫入本機設定檔。

## 規則

- 不直接改使用者的 PX4 原始碼樹。自己的模組放 `modules/src/`（SITL 的包裝在 `sitl/px4_external/`），用 PX4 的 `EXTERNAL_MODULES_LOCATION` 編進去；必須改 PX4 原有的檔案時，做成 patch 放 `px4/patches/`，由配方的 `firmware/firmware.json` 的 `recipe.patches` 指定。
- **配方的 patch 只套用在韌體**：`px4/build_firmware.py` 會套，SITL（`sitl/build_sitl.py`）直接編鎖定的原始碼樹，不讀配方。現有的記憶卡修正只和實體飛控有關；以後 patch 改到控制邏輯時，要先說清楚 SITL 怎麼用到同一份修改，否則兩邊測的不是同一份程式。
- 樹裡的 `build/` 是建置產物（SITL 編在 `build/px4_sitl_default_sim2hitl`，字尾可用 `SIM2HITL_SITL_SUFFIX` 換），git 忽略，不影響核對。

## 編 FC 韌體

```bash
./runbridge build-firmware --release deploy/<配方>
```

- 讀配方 `deploy/<配方>/firmware/firmware.json` 的 `recipe`：`make_target`、`patches`、`modules`（`modules/src/` 裡的資料夾名）。沒有 `recipe` 的配方不編：`deploy/00_native` 保存的是現成的原生韌體。
- 在原始碼樹**旁邊的建置樹**編譯：`<樹>-build`（或環境變數 `SIM2HITL_PX4_BUILD_TREE`）。第一次用 `rsync` 從原樹複製一份（不需要網路），之後每次編譯前重設到鎖定的 commit。使用者的原樹不動。**建置樹是可以整個清掉的專用副本**：每次編譯前會強制切回鎖定的 commit、把子模組重設並清掉所有未追蹤的檔案，所以 `SIM2HITL_PX4_BUILD_TREE` 不可以指到原始碼樹或任何放了自己東西的資料夾。
- **FC 回報的 `board_version` 是 USB 產品碼**（`CONFIG_CDCACM_PRODUCTID`），不一定等於韌體的 board id：6C 都是 56，Cube Orange+ 是 4184 對 1063。`build_firmware.py` 從建置的 NuttX 設定記下它，身分核對比它。
- patch 用 `git am` commit 在建置樹對應的 repo 裡，作者與日期取自 patch 檔，committer 固定、commit 日期等於作者日期，所以同樣的 patch 永遠得到同樣的 commit。FC 回報的 NuttX hash 因此可預期，`deploy-test` 與 `deploy/check_params.py` 用它核對 FC 上是哪個配方。
- patch 的 `repo` 是 `.`（PX4 主 repo，例如板子變體）時，`git describe` 變成 `v1.17.0-1-g…`，FC 回報的版本類型從 release 變成 dev；只改子模組（NuttX 修正）時不會。
- 產物（`.px4`、`parameters.json.xz`）寫到配方的 `firmware/`（不進 git，見 `.gitignore`）；身分（`embedded`：board id、FC 會回報的 `board_version`、PX4 與 NuttX 的 hash）、flash 用量與建置紀錄寫進 `firmware.json`，參數名單的 SHA-256 寫進配方的 `params.json`。建置 log 在 cache（`./runbridge where cache`）的 `firmware/<配方>/`。
- 通常不必單獨編：`./runbridge deploy-test --release deploy/<配方> --flash` 在沒有編好的檔、或 patch、模組改了的時候會自己編。

## 換 PX4 版本

SITL、韌體與 HITL 要同一個版本，所以是整套一起換。先和使用者談好再動手：

1. 問使用者新樹放哪裡（專案外）；舊樹留著，回退要用。下載新的 tag。
2. 對舊 tag 看差異：uORB 訊息（`msg/`）、固定翼模組（`src/modules/fw_*`）、專案用到的參數、開機腳本與機架（`ROMFS/`）、有配方的 board（`boards/px4/fmu-v6c/`、`boards/cubepilot/cubeorangeplus/`；Cube 的預設韌體是否已經有 `pwm_out_sim`）。兩版的參數名單用 `python3 deploy/check_params.py --compare-firmware 舊.json.xz 新.json.xz` 比。
3. 查新版是否已經包含 `px4/patches/` 的 NuttX 修正（AGENTS 第 1 段），Cube 的 `hil` 變體 patch 是否還套得上、還需不需要。
4. 改 `px4_lock.json` 的 `tag`、`commit`、`hitl_firmware.version`；`python3 px4/check_px4_tree.py --path <新樹> --remember` 要 PASS。
5. 重編 SITL、重跑情境；每個配方重編並用 `deploy-test` 測。`deploy/00_native` 的韌體是 v1.17.0 的成品、不會自己重建，新版的原生基準怎麼取得要和使用者討論。

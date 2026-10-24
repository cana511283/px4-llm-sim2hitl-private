# 檔案分類與維護規則

2026-10-24 起採用。人與 AI 後續新增、分析、備份及搬移檔案均遵守本規則。

## 分支用途

- `main` 目前是歷史備份與實驗證據入口。
- `carrier-orbit-no-range-20261022` 目前是開發來源分支，包含模型、程式、設定、文件及實驗摘要。
- 從對應來源建立工作分支；先檢查、修改、驗證並推送工作分支，再依使用者授權合併回對應來源。未授權時不合併。不要把開發來源整包覆蓋備份 main。
- 本規則在兩個來源分支各保留一份；修改共用分類規則時，同步更新兩份文件與 AGENTS 入口。

## 檔案放哪裡

| 內容 | 位置與原則 |
|---|---|
| 氣動、慣量、控制、ULog 等可重用分析程式 | `analysis/`；一次分析的輸出放該次實驗，不混入工具目錄 |
| 技術理解與研究方法 | `docs/`；飛控 `docs/fc/`、平台 `docs/platforms/`、任務 `docs/missions/` |
| 每次 SIL、SITL、HITL、飛試與離線分析結果 | `experiments/YYYYMMDD-主題/`；同日多回合加獨立後綴，不覆寫舊結果 |
| 可重用任務程式 | `missions/`；結果不再新增到 `missions/results/` |
| 模型與建置來源 | 保持 `model/`、`plant/`、`modules/`、`px4/`、`sitl/`、`hitl/`、`aircraft/` 現有位置與分工 |
| 飛控部署配方與運行設定 | 保持 `deploy/`、`hitl/aircraft/`、`hitl/scenarios/`、`sitl/scenarios/` 等現有位置；依 loader 與配方分工維護 |
| 根目錄 | README、AGENTS、runbridge、Python metadata 等入口；不再散放新的實驗 ZIP、圖表與結果 |

`src/`、`config/` 是後續可能的重構方向，尚未完成遷移；不得只為符合名稱就新增空目錄或搬動目前有路徑相依的來源。元件旁既有 README 可保留，用 docs 索引串接。

## 實驗與分析紀錄

- 每次紀錄交代目的、資料來源、source branch／commit、模型 release、場景／任務、參數與韌體身分（不適用的註明）、分析方法、結果、限制及證據位置。
- 區分「流程完成」「驗收通過」「未測／未評定」。失敗、中止、未啟動回合均保留原始證據，不能併成成功案例。
- 分析更新可以獨立提交，不需要同步修改主程式。重新分析用新報告並指向原始資料，不覆寫原始 runner、日誌或失敗結果。
- 歷史 JSON 的執行路徑描述當時環境，維持原文；另用現行索引說明目前位置。
- `experiments/legacy-20261021-22/` 是跨多版本的歷史集合，保留原結構；新實驗依日期與主題另建目錄。

## 封存與搬移檢查

- 不刪除原始資料、不重寫歷史；用 `git mv` 搬已追蹤檔。模型、參數、產碼不得夾帶行為修改。
- 搬移前查所有引用：Markdown、MATLAB callback／load／addpath、MAT／資料字典／模型參照、Python import／parents、CMake、設定、release manifest 與 runtime 狀態定位。
- 保持 SLX、初始化、preload、importer、MAT 的相依組合；生成 C 不手改。調整來源或產碼遵守開發分支 AGENTS 的驗證規則。
- runtime／程式位置若要搬動，先在原執行位置確認 recover 完成且 owner 已關閉；不要用新路徑繞過待恢復狀態。
- 搬移前後核對清單與 SHA-256／Git blob，更新文件、圖檔、資料連結與導航。Windows 換行轉換需分清工作檔與 Git 原始 bytes，不能把 CRLF 差異誤報成模型改動。
- ZIP 分卷作為完整集合保留；按 manifest 的次序重組，再核對整包 SHA-256。Git bundle、來源 ZIP、參數、wire、時序、ULog 與還原證據的關聯不可拆散。
- 原 manifest 是歷史證據，不因搬移就改成新的雜湊；發現原有不符應記錄差異。大型資料更換儲存策略前，先有可用替代位置及完整性核對，不套用泛用忽略規則使原始資料失去保存。
- 提交前檢查 `git diff --check`、受影響連結與相依設定。依實際改動執行必要驗證，沒跑的明寫未測；單純文件／證據分類不宣稱模型或飛行測試通過。
- 開發分支修改中文 README 時，同步英文內容與其記載的中文 README SHA-256；更新 docs 與 experiments 索引。提交說明交代分類範圍、驗證與限制。

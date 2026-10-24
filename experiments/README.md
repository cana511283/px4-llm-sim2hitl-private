# 實驗紀錄

本目錄保存已追蹤的歷史摘要、結果與失敗證據；分析工具仍在 `analysis/`，可重用任務程式仍在 `missions/`。

| 紀錄 | 範圍 |
|---|---|
| [20261021-cube-circuit](20261021-cube-circuit/postmortem.json) | 缺少地形量測，降落中止；保留失敗診斷 |
| [20261022-flat-range-circuit](20261022-flat-range-circuit/flight-analysis.json) | 平面測距版本完成模擬降落流程 |
| [20261022-orbit-no-range](20261022-orbit-no-range/analysis.json) | 停用測距盤旋降落；保留未啟動回合的證據 |

任務方法與結果限制見[任務文件](../docs/missions/README.md)及[盤旋降落文件](../docs/missions/orbit_no_range.md)。完整封存與 ULog 在儲存庫 main 的備份歷史，不在本開發分支完整重複保存。流程完成不等於已評定實機著陸品質。

新增分析另存新檔，並記錄資料來源、source commit、模型 release、參數／任務設定、方法與結果；不可覆寫原始失敗證據。歷史 JSON 裡的執行路徑保留原文，代表當時執行位置。

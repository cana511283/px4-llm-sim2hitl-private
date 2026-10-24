# SkySurfer / Cube Orange+ 專案備份

本分支保存歷史實驗與證據；可接續開發的程式在 [carrier-orbit-no-range-20261022](https://github.com/cana511283/px4-llm-sim2hitl-private/tree/carrier-orbit-no-range-20261022)。

| 實驗紀錄 | 內容 |
|---|---|
| [停用測距盤旋降落](experiments/20261022-orbit-no-range/report.md) | 2026-10-22、來源 96d4691、FC log 358；模擬流程完成，著陸品質未評定 |
| [2026-10-21～22 歷史備份](experiments/legacy-20261021-22/README.md) | 同時保留降落中止版本 49a8bfd 的 postmortem 與後續平面測距版本 9ea1a967、FC log 357；不是單一成功實驗 |

原封存、分卷、manifest、報告內容完整保留。legacy 目錄內 README 是整理前的歷史快照，其中 `20261022-orbit-no-range/` 連結應改從本頁進入。manifest 仍是原始封存證據，沒有重寫成新的雜湊。

分割 ZIP 請在 legacy 目錄依檔名順序合併，核對 manifest 的整包 SHA-256 後解壓。source ZIP 與 Git bundle 是歷史來源，不能當成目前工作目錄或已發布模型 release。

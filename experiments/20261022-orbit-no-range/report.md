# 停用測距後的空投盤旋降落驗證

2026-10-22；Cube Orange+，沿用昨天的模型 release skysurfer-5e8343f64ea32411a30665ec 與既有韌體。

**本次 HITL 已完成：空投 → 約 1.17 圈盤旋下降 → 下滑道 → 降落與減速。沒有五邊或 40 秒觀察段。**

## 測試設定及實際證據

- 100 m、30 節母機水平釋放，無風，釋放後立即要求 AUTO。
- 場景 flat_rangefinder=false：不啟用新增的向下測距產生／傳送；未移除 PX4 導航與降落控制器。
- 飛控日誌實際參數 FW_LND_USETER=0；FW_LND_ABORT=3 保留原值，在 USETER=0 時其地形中止條件不作用。降落使用航點高度作為地面基準。
- 本次 HIL 1851 筆 local position 的 dist_bottom_valid 全為 false；distance_sensor 記錄 0 筆。SITL 同樣沒有有效地形資料。
- HIL 降落中止狀態只有 0；Commander 記錄 Landing detected，Navigator 記錄 Mission finished, landed。

## 量測

| 項目 | SITL（模型真值／stub） | HITL（Cube EKF／原生落地偵測） |
|---|---:|---:|
| 盤旋角度累積換算 | 1.172 圈 | 1.172 圈 |
| AUTO，解鎖後 | 60.00 s | 61.00 s |
| 回報落地，解鎖後 | 181.76 s | 185.16 s |
| 回報落地附近的水平速度 | 0.99 m/s | 0.28 m/s |
| 位置記錄中位取樣間隔 | 0.004 s | 0.100 s |

圈數算法為任務序號2時、距盤旋中心70–130m內的位置極角累積，包含部分進出場弧段；不是宣稱精確飛完指定整數圈。HIL 的最早有效地面定位在解鎖後8.60s（仍在10s地面等待段），換算至場景原點約 N-0.026/E-0.071m，作為座標核對。SITL 的接地 stub 與 HIL 的落地偵測規則不同；兩者使用各自記錄到的解鎖時間，不能把落地時間差直接解讀為控制性能差異。

SITL 第一次重心穿越地面平面約176.17s；HIL 此處只報原生落地偵測，未另定接觸或撞擊品質門檻。HIL 最終 local 高度約-0.285m是相對EKF原點，不能據此推算模型穿地深度。

## 日誌、還原與保留狀態

- 新 FC log **358**，測前最新357；雙下載 SHA-256相同，嚴格解析通過，ULog無dropout／截斷。
- 航點與三項暫時參數還原並讀回通過；沒有非預期參數差異。FC-owned差異僅 CAL_BARO2_OFF、COM_FLIGHT_UUID、LND_FLIGHT_T_LO。這次沒有昨天的 ASPD_SCALE_1 核對錯誤。
- 時序零漏拍，原runner成功結束，沒有 evidence_error。
- 最終未解鎖、SYS_HITL=1、無待恢復／參數交易；bridge保持開啟，模型未運行。

日誌 SHA-256：`9617b7ac54d705df1e7ce707942ee80af2ea8dda6938246030094b8af352fe17`。完整原始證據在旁邊的 orbit-no-range-hitl-20261022-01 資料夾；本資料夾有摘要、雙方數字、航跡圖與航點。原始權限失敗及未啟動的 SITL 證據仍保留於 work/orbit-sitl-01，沒有覆寫。

**結論範圍：支持本次無風、平地、已知降落點高度的 HITL 能在停用地形量測且不送向下測距時完成降落流程。著陸品質未評定，不作實機安全著陸的判定。**

![航跡比較](trajectory.png)

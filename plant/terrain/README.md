# 平面地面幾何

`../include/plant_flat_ground.h` 提供無限水平面 NED D=0，與產碼模型既有的接地平面一致。它是接地力學模型的幾何查詢介面，不再施加另一份地面力。

- `plant_flat_ground_down_m(N,E)`：任何位置的地面 D 都為 0 m。
- `plant_flat_ground_agl_m(D)`：垂直離地高度 -D，保留接地模型微小穿透的負高度。
- `plant_flat_ground_range_m(D, ray_ned, min, max, &range)`：感測器射線與平面的斜距；考慮射線傾斜，並檢查量程、無交點、非法輸入與地下感測器。失敗回傳 0 且距離為 NAN。

輸入以 plant 地面為原點，不能直接拿 FC Home 相對高度代入。若未來場地有已知海拔 H，地面 MSL=H、飛機 MSL=H-D；H 必須與動力學、GPS、氣壓及任務一致。現在不新增可移動的地面高度，現有模型仍使用其原本基準，避免幾何與接地面分離。

HIL 可選介面：SkySurfer 場景加入 `"flat_rangefinder": true`，使用新版 release 的 C writer，依模型真值 AGL 與 roll／pitch 算機體向下射線（感測器假設位於重心）。25 Hz、量程 0.05–150 m、理想無雜訊／延遲的雷射模型；DISTANCE_SENSOR 的 orientation=25、covariance=255（未知）。有效品質=100；地下、未相交或超量程時 current_distance=15001 cm、quality=1（MAVLink 編碼的零品質）。不把無效距離夾到量程內。實機感測器不能直接假定具備此量程與性能。

編碼在 native/posix/io.c 的 4 ms C 路徑（每 10 步一筆），另有獨立 latest-slot，沿用停止／錯誤／恢復時清空與寫入屏障。預設關閉，不影響舊場景；無支援的舊 worker 會拒絕啟用。host/packets.py 為 fake/offline 的同語意參考，不會進入原生 4 ms 迴圈。ABI 結構與產碼核心保持原樣，選用時限定 SkySurfer 的既有 raw truth 位置，不適用其他飛機。

範例：`hitl/scenarios/carrier_release_flat_range.json`。此功能須重建並驗證完整模型 release；它不修改飛控參數。封包離線測試不能證明 Cube 的 EKF 已接受地形資料，需先核對 distance_sensor、EKF 融合與 dist_bottom_valid，包含缺訊／超量程保護，再經人同意執行新版 HIL。

離線檢查（在專案根目錄，輸出選擇工作目錄）：

```sh
cc -std=c11 -Wall -Wextra -Werror -I plant/include plant/terrain/test_flat_ground.c -lm -o /tmp/test_flat_ground
/tmp/test_flat_ground
```

檢查包含 100 m 垂直距離、3-4-5 斜距、射線尺度不變性、量程邊界、地下／平行／向上／非法射線。2026-10-22 PASS。

2026-10-22 Cube Orange+ 實測已確認接收 distance_sensor、產生有效 EKF 對地高度並完成原生航線降落流程；詳見 [任務結果](../../missions/README.md)。仍使用理想模擬測距，未代表實體感測器規格或實機著陸品質。

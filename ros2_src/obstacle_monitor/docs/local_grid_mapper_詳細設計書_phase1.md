# 日本語文書

# local_grid_mapper 詳細設計書（Phase1）

## 1. 目的
UTM-30LX（/scan）と Mid-360（/mid360/points2d）を base_link 座標系へ揃え、
既存 `mid360_points2d_extractor_node.py::_publish_scan_image()` と同等の描画仕様で
`/grid_viewer` に可視化画像を配信する。

## 2. ノードI/F
### Subscribe
- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/mid360/points2d` (`sensor_msgs/msg/PointCloud2`)

### Publish
- `/grid_viewer` (`sensor_msgs/msg/Image`, `bgr8`)

### パラメータ
- `target_frame` (string, default: `base_link`)
- `scan_topic` (string, default: `/scan`)
- `mid_topic` (string, default: `/mid360/points2d`)
- `viewer_topic` (string, default: `/grid_viewer`)
- `viewer_x_range_m` (double, default: `20.0`)
- `viewer_y_range_m` (double, default: `5.0`)
- `viewer_pixel_pitch` (int, default: `40`)
- `viewer_grid_interval_m` (double, default: `5.0`)
- `robot_width_m` (double, default: `0.8`)

## 3. TFの扱い
- tf2 を用いて `/scan` と `/mid360/points2d` を `target_frame`（base_link）へ変換する。
- 変換取得に失敗した場合は warn ログを出し、当該メッセージの処理をスキップする。

## 4. 既存描画の踏襲方針（案A/B）
### 案A: `_publish_scan_image()` のロジックをノード内に踏襲
- 既存実装の描画仕様（画像サイズ・スケール・原点・補助線・背景）を
  そのまま新ノード内に再実装し、点群の色分けのみ追加する。
- 差分が最小で、既存の見た目と齟齬が出にくい。

### 案B: 共通描画関数を抽出して共有
- `mid360_points2d_extractor` から共通描画関数を切り出し、
  `local_grid_mapper` からも利用する。
- 共有化のための追加変更が必要となり、影響範囲が広がる。

### 採用案
- **案Aを採用する。**
- 理由: 既存仕様の踏襲を最優先とし、差分を最小化するため。

## 5. 色分け仕様
- UTM-30LX（/scan）: **青** `(B, G, R) = (255, 0, 0)`
- Mid-360（/mid360/points2d）: **赤** `(B, G, R) = (0, 0, 255)`
- 点のサイズ、描画範囲、背景色は既存 `_publish_scan_image()` に合わせる。

## 6. 更新タイミング
- センサコールバックはデータ受信・変換・キャッシュ更新のみを行う。
- 描画および `/grid_viewer` への publish は **timer 駆動（10Hz）** で行う。
- 未受信側の点群は直近のキャッシュを使用する。

## 7. 例外処理
- tf 取得失敗時は warn ログを出力し、当該処理をスキップする。
- 片方の点群が未受信の場合でも、受信済み点群のみ描画する。

## 8. 並行実行と排他制御
- `MultiThreadedExecutor` を使用し、センサ受信とタイマ更新を並行実行する。
- `MutuallyExclusiveCallbackGroup` をセンサ・タイマの各コールバックに明示指定する。
- 共有データ（点群キャッシュ、header）へのアクセスは `threading.Lock` で保護し、
  コールバックグループの排他に依存しない整合性を確保する。

## 9. 実行例
```bash
ros2 launch obstacle_monitor local_grid_mapper.launch.py \
  scan_topic:=/scan mid_topic:=/mid360/points2d target_frame:=base_link
```

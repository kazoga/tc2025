# 日本語文書
# mid360_scan_extractor 詳細設計書 (phase1)

## 1. 目的
Livox MID360 由来の `/mid360/livox/lidar` (sensor_msgs/msg/PointCloud2) を購読し、
高さ方向と水平半径の範囲で点を抽出した上で擬似的な2D LiDARの LaserScan を出力する
ノード `mid360_scan_extractor` を obstacle_monitor パッケージに追加する。
また、LaserScan を可視化した画像を配信する。

## 2. 入出力仕様
- 入力トピック: `/mid360/livox/lidar` (sensor_msgs/msg/PointCloud2)
- 出力トピック: `/mid360/scan` (sensor_msgs/msg/LaserScan)
- 画像トピック: `/mid360/scan_viewer` (sensor_msgs/msg/Image)
- フレームID: `/mid360_frame` (入力と同一)
- QoS:
  - PointCloud2 / LaserScan: SensorDataQoS (`qos_profile_sensor_data`)
  - 画像: Reliable / Volatile / depth=1
- 更新タイミング: 入力受信ごとに即時処理して publish

## 3. パラメータ一覧
| 名称 | 型 | 既定値 | 役割 |
| --- | --- | --- | --- |
| `height_min_m` | double | 0.1 | 抽出する高さ範囲の下限 [m] |
| `height_max_m` | double | 1.0 | 抽出する高さ範囲の上限 [m] |
| `max_radius_m` | double | 20.0 | 水平方向に残す最大半径 [m] |
| `range_min_m` | double | 0.05 | LaserScan の `range_min` に設定し、これ未満の点は除外 |
| `angle_min_deg` | double | -180.0 | LaserScan の開始角度 [deg] |
| `angle_max_deg` | double | 180.0 | LaserScan の終了角度 [deg] |
| `angle_increment_deg` | double | 0.5 | LaserScan の角度分解能 [deg] |
| `frame_id` | string | `/mid360_frame` | 出力 LaserScan の frame_id |
| `input_topic` | string | `/mid360/livox/lidar` | PointCloud2 購読先 |
| `output_topic` | string | `/mid360/scan` | LaserScan 配信先 |
| `viewer_topic` | string | `/mid360/scan_viewer` | LaserScan 画像の配信先 |
| `viewer_x_range_m` | double | 20.0 | 画像化する前方距離範囲 [m] |
| `viewer_y_range_m` | double | 5.0 | 画像化する左右距離範囲 [m] |
| `viewer_pixel_pitch` | int | 40 | 画像化の解像度 [pix/m] |
| `viewer_grid_interval_m` | double | 5.0 | 進行方向の補助線間隔 [m] |
| `robot_width_m` | double | 0.8 | ロボット幅ライン描画用の幅 [m] |

## 4. 処理フロー
1. ノード初期化時に各パラメータを declare し、角度範囲の妥当性を検証。
   - `angle_max_deg <= angle_min_deg` または `angle_increment_deg <= 0` の場合は ValueError を送出。
   - `height_max_m < height_min_m` の場合は警告を出して交換し、上限=下限となるケースでは通過点のみを許容。
2. PointCloud2 購読時のコールバックで以下を実施。
   1. `sensor_msgs_py.point_cloud2.read_points()` で (x, y, z) を取得し NaN をスキップ。
   2. `height_min_m <= z <= height_max_m` かつ 水平距離 `r = sqrt(x^2 + y^2)` が
      `range_min_m <= r <= max_radius_m` の点のみ残す。
   3. 角度 `theta = atan2(y, x)` が [`angle_min`, `angle_max`] に入る点を角度ビンへ割り当て、
      ビンごとに最小距離を保持する。ビン数は `(angle_max - angle_min) / angle_increment + 1` とする。
   4. 未観測ビンは `inf` で初期化したままとし、LaserScan.ranges へそのまま格納する。
   5. LaserScan メッセージを組み立て、ヘッダ stamp は入力 PointCloud2 の stamp を利用、
      frame_id はパラメータを使用する。
   6. LaserScan の ranges を XY 点群へ変換し、画像化して publish する。
      - 表示範囲: x 0..`viewer_x_range_m`, y ±`viewer_y_range_m`
      - 解像度: `viewer_pixel_pitch` [pix/m]
      - ロボット幅ラインと進行方向の補助線を描画する。
3. ノード起動・異常検出時は info / warn ログを日本語で出力する。

## 5. ファイル構成
- `obstacle_monitor/mid360_scan_extractor_node.py` : 新規ノード実装
- `setup.py` : console_scripts へ `mid360_scan_extractor` を追加
- `docs/mid360_scan_extractor_詳細設計書_phase1.md` : 本書

## 6. テスト方針
- ROS 環境外では単体テストが困難なため、静的検証に留める。
- 実機/シミュレータで PointCloud2 を流し、`ros2 topic echo /mid360/scan` で
  高さ範囲フィルタと角度分解能が期待通りであることを確認する。

## 7. 追加ノード `mid360_points2d_extractor`
Livox MID360 の点群を高さ・水平半径でフィルタし、Z=0 に正規化した擬似2D PointCloud2 を配信するノードを追加する。

- 入力トピック: `/mid360/livox/lidar` (sensor_msgs/msg/PointCloud2)
- 出力トピック: `/mid360/points2d` (sensor_msgs/msg/PointCloud2)
- フレームID: `/mid360_frame` (入力と同一)
- QoS: SensorDataQoS (`qos_profile_sensor_data`) を購読・配信双方で使用
- 更新タイミング: 入力受信ごとに即時処理して publish

### 7.1 パラメータ
| 名称 | 型 | 既定値 | 役割 |
| --- | --- | --- | --- |
| `height_min_m` | double | 0.1 | 抽出する高さ範囲の下限 [m] |
| `height_max_m` | double | 1.0 | 抽出する高さ範囲の上限 [m] |
| `max_radius_m` | double | 20.0 | 水平方向に残す最大半径 [m] |
| `range_min_m` | double | 0.05 | これ未満の近距離点を除外する下限 [m] |
| `frame_id` | string | `/mid360_frame` | 出力 PointCloud2 の frame_id |
| `input_topic` | string | `/mid360/livox/lidar` | PointCloud2 購読先 |
| `output_topic` | string | `/mid360/points2d` | PointCloud2 配信先 |

### 7.2 処理フロー
1. ノード初期化時に高さ・半径・入出力トピックを declare し、高さ範囲が逆転している場合は交換する。
2. PointCloud2 受信時に以下を行う。
   1. `sensor_msgs_py.point_cloud2.read_points()` で (x, y, z) を取得し NaN をスキップする。
   2. `height_min_m <= z <= height_max_m` かつ `range_min_m <= sqrt(x^2 + y^2) <= max_radius_m` を満たす点のみ残す。
   3. 残した点の z を 0.0 に置き換えて新しい PointCloud2 を生成する。
   4. ヘッダは入力の stamp を用い、frame_id はパラメータの値に設定して publish する。

### 7.3 テスト方針
- ROS 環境外では単体テストが困難なため、静的検証に留める。
- 実機/シミュレータで PointCloud2 を流し、`ros2 topic echo /mid360/points2d` で高さ・半径フィルタおよび Z=0 正規化が反映されていることを確認する。

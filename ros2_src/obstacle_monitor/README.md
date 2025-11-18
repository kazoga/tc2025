# obstacle_monitor パッケージ README (phase2正式版)

## 概要
`obstacle_monitor` は `/scan`（LiDAR）から前方障害物を解析し、
`route_msgs/ObstacleAvoidanceHint` を `/obstacle_avoidance_hint` として配信するノードです。
Phase2 では legacy の避障ロジックを ROS2 へ移植し、`/sensor_viewer`
（`sensor_msgs/Image`）で LaserScan の可視化も提供します。動作確認用に
`laser_scan_simulator` ノードも同梱しています。

## 主な機能
- `/scan` の ±90° 以内の点群からロボット幅帯を抽出し、左右の回避オフセットを算出。
- `front_cone_half_deg`・`stop_dist_m` を用いた前方くさび判定で閉塞状況を検知。
- `/obstacle_avoidance_hint` に front_blocked・front_clearance_m・左右オフセットを配信。
- `/sensor_viewer` に LaserScanViewer 互換の bgr8 画像を配信し、
  `/amcl_pose`・`/active_target` を重畳して目標方向を可視化。
- `hint_range_m` が `max_obstacle_distance_m` 未満の場合は警告しつつ自動で補正。

## 起動方法
### launch を用いた起動
```bash
ros2 launch obstacle_monitor obstacle_monitor.launch.py \
  scan_topic:=/scan hint_topic:=/obstacle_avoidance_hint
```
- launch 引数で購読・配信トピック名を remap できます。
- `params/default.yaml` が既定パラメータを提供します。

### 実行ファイルを直接起動
```bash
ros2 run obstacle_monitor obstacle_monitor
```
- `laser_scan_simulator` を併用する場合は別ターミナルで
  `ros2 run obstacle_monitor laser_scan_simulator` を起動し、`/amcl_pose` を供給してください。

## 外部インタフェース
### Subscriber
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 入力。SensorDataQoS（BEST_EFFORT / VOLATILE / depth=1）。 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 現在姿勢。viewer で目標線を描画するために利用。 | RELIABLE / VOLATILE / depth=10 |
| `/active_target` | `geometry_msgs/PoseStamped` | 現在の目標位置。viewer の矢印描画に利用。 | RELIABLE / VOLATILE / depth=10 |

### Publisher
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/obstacle_avoidance_hint` | `route_msgs/ObstacleAvoidanceHint` | front_blocked・front_clearance_m・左右オフセット[m] を配信。 | BEST_EFFORT / VOLATILE / depth=1 |
| `/sensor_viewer` | `sensor_msgs/Image` | LaserScanViewer 相当のデバッグ画像（bgr8）。 | BEST_EFFORT / VOLATILE / depth=1 |

> サービス・アクションは提供しません。

## パラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `front_cone_half_deg` | double | `10.0` | 前方閉塞判定の半角 [deg]。
| `stop_dist_m` | double | `1.0` | くさび内で停止を指示する距離 [m]。
| `front_clearance_percentile` | double | `5.0` | 前方閉塞判定で使用する距離分位点[%]。
| `front_clearance_min_points` | int | `5` | 前方閉塞判定を行うために必要な最小点数。
| `max_obstacle_distance_m` | double | `1.5` | 避障対象とする最大距離 [m]。
| `hint_range_m` | double | `5.0` | front_clearance を通知する最大距離 [m]。
| `robot_width_m` | double | `0.8` | ロボットの全幅 [m]。
| `avoid_offset_min_m` | double | `0.75` | 左右オフセットの下限値 [m]。
| `viewer_map_range_m` | double | `8.0` | viewer の描画範囲（x:0..8, y:-4..4）。
| `viewer_pixel_pitch` | int | `100` | viewer のピクセル密度 [pix/m]。
| `viewer_window` | string | `"laserscan"` | OpenCV 表示ウィンドウ名（debug 用）。
| `viewer_resize_px` | int | `500` | 配信画像のリサイズサイズ [px]。

## 状態管理・処理フロー
1. `/scan` を受信すると NaN/Inf・後方データを除外し、前方 ±90° の点群へ変換する。
2. `max_obstacle_distance_m` 以下の点を左右に分割し、|y| 昇順で並べ替える。
3. legacy `calcAvoidanceOffset` に基づきギャップ検出・外縁推定で左右オフセットを算出し、
   `avoid_offset_min_m` の下限を適用する。
4. `front_cone_half_deg` 内での距離分位点を計算し、`stop_dist_m` 以下で閉塞と判定する。
5. 算出結果を `ObstacleAvoidanceHint` に格納し、`/sensor_viewer` へ可視化画像を出力する。
6. `/amcl_pose`・`/active_target` を利用して viewer 上に目標矢印を描画し、状況把握を補助する。

## 動作確認手順
1. `robot_simulator` などから `/amcl_pose`・`/active_target` を配信する。
2. `laser_scan_simulator` を起動し、LiDAR の疑似データを `/scan` に配信する。
3. `obstacle_monitor` を launch し、`/obstacle_avoidance_hint` の `front_blocked` や
   `left_offset_m` が障害物に応じて変化することを確認する。
4. `rqt_image_view` 等で `/sensor_viewer` を参照し、点群・目標矢印が描画されていることを確認する。

## デバッグのヒント
- `hint_range_m` < `max_obstacle_distance_m` の場合は WARN が出力され、内部で自動補正されます。
- viewer 画像が真っ白な場合は `/scan` の距離が範囲外になっていないか確認してください。
- `front_blocked` が常に `False` の場合は `front_cone_half_deg` と `stop_dist_m` の組み合わせを見直します。

## 将来拡張メモ
- `ObstacleAvoidanceHint` に左右独立の clearance を追加し、偏った障害物への指示精度向上を検討中。
- `laser_scan_simulator` の地図入力を launch から指定できるよう改善予定です。

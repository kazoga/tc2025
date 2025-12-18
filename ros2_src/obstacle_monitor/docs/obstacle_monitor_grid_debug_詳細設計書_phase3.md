# 日本語文書

## 1. 目的と背景
`obstacle_monitor_grid_debug` ノードは UTM-30LX の LaserScan (40Hz) と Mid-360 の擬似2D PointCloud2 (10Hz) を
`odom` 固定の rolling grid (40m × 40m, resolution=0.1m) に統合し、free/hit スコアの時間正規化蓄積と
減衰処理を行う。OpenCV での色分け可視化によりスコア分布の健全性を確認することを目的とする。

## 2. ノード構成概要
- ノード名: `obstacle_monitor_grid_debug`
- 言語: Python (rclpy)
- subscribe:
  - `/scan` (`sensor_msgs/LaserScan`, SensorDataQoS)
  - `/mid360/points2d` (`sensor_msgs/PointCloud2`, SensorDataQoS) ※ `mid360_points2d_extractor` に合わせる
  - `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`, depth=10)
- tf: `odom` を固定フレームとして、`laser` および `mid360_frame` からの変換を取得する
- timer:
  - 減衰処理: 5Hz 相当で free/hit を減衰
  - 可視化更新: 10Hz 目安（描画負荷を抑えつつ十分な確認頻度）

## 3. グリッド管理
- 実空間: 40m × 40m、解像度 0.1m → 400 × 400 セル。
- 座標: `odom` 座標系に平行な固定グリッド。回転は行わない。
- センター追従: `amcl_pose` の平行移動量でグリッド中心を更新し、セル配列は numpy.roll により
  シフトして再利用する（配列サイズを変えずに rolling させる）。
- セル構造（配列で保持）:
  - `free_score` (float32)
  - `hit_score` (float32)
  - `last_hit` (float64, sec) ※ rclcpp::Time 相当の秒表現

### 3.1 中心更新とシフト
1. `amcl_pose` 更新時に中心座標を取得。
2. 旧中心との差分 (dx, dy) をセル数へ換算し丸める。
3. シフトが発生する軸のみ `numpy.roll` で free/hit/last_hit を移動。
4. ロールインした新領域をゼロクリアし、中心・原点座標を更新。

## 4. センサ更新フロー

### 4.1 LaserScan (UTM-30LX)
1. 受信時刻 `now` と前回 Laser 受信時刻から Δt を算出。初回は既定 Δt（0.025s）を使用。
2. `tf2_ros.Buffer.lookup_transform('odom', msg.header.frame_id, msg.header.stamp)` で変換を取得。
3. 各ビームについて角度を `sensor_yaw + angle_min + i*angle_increment` で算出。
4. free: range_max までの終点を用いて Bresenham でセル列を生成し、終点を除くセルへ
   `free_score += utm_free_rate * Δt`。
5. hit: range が有限かつ範囲内の場合に終点セルへ `hit_score += utm_hit_rate * Δt`、
   `last_hit = stamp(sec)` を格納。

### 4.2 Mid-360 PointCloud2
1. Δt を前回 Mid 更新時刻から算出（初回は既定 Δt=0.1s）。
2. `odom` への tf を取得し、各点 (x, y, 0) を 2D 変換。
3. グリッド内のセルに対し `hit_score += mid_hit_rate * Δt` を加算し、`last_hit` を更新。

### 4.3 時間正規化・レート
- Δt をすべての加算に乗算する。
- レートは固定値。
  - `utm_free_rate = 1.0`
  - `utm_hit_rate = 1.0`
  - `mid_hit_rate = 4.0`

### 4.4 減衰
- タイマで 0.2s ごとに下式を適用。
  - `free_score *= 0.8`
  - `hit_score  *= 0.95`

## 5. インデックス計算と raytrace
- 原点をグリッド左下とし、`index = floor((coord - origin) / resolution)` で算出。
- 範囲外は無視。
- Bresenham 法で start/end セルを離散化し、終点を含まないセル列を free 更新に利用する。

## 6. 可視化
- `cv::Mat` 相当の OpenCV 画像を NumPy で生成（400×400、BGR）。
- 色分け規則:
  - `hit_score > free_score` → 赤 (0,0,255)
  - `free_score > hit_score` → 青 (255,0,0)
  - それ以外 → 黒
- ロボット中心を画像中央とし、amcl_pose の yaw を用いた三角形で方位を描画。
- `cv2.imshow()` と `cv2.waitKey(1)` で随時更新する。

## 7. スレッド・コールバック構成
- rclpy シングルスレッド実行を想定。
- サブスク 3本、タイマ 2本（減衰・描画）。
- グリッド更新と描画は同スレッドで直列に処理され、データ競合は発生しない。

## 8. パラメータ一覧（予定）
| 名称 | 既定値 | 用途 |
| --- | --- | --- |
| `grid_resolution_m` | 0.1 | セル分解能 |
| `grid_size_m` | 40.0 | 一辺長 [m] |
| `target_frame` | `odom` | グリッドの基準座標 |
| `scan_topic` | `/scan` | LaserScan 入力 |
| `mid_topic` | `/mid360/points2d` | Mid-360 点群入力 |
| `amcl_topic` | `/amcl_pose` | 中心追従用姿勢 |
| `utm_free_rate` | 1.0 | free スコア加算レート |
| `utm_hit_rate` | 1.0 | Laser hit スコア加算レート |
| `mid_hit_rate` | 4.0 | Mid hit スコア加算レート |
| `decay_period_s` | 0.2 | 減衰周期 |
| `visualize_period_s` | 0.1 | 描画周期 |
| `default_scan_dt_s` | 0.025 | Laser 初回 Δt |
| `default_mid_dt_s` | 0.1 | Mid 初回 Δt |
| `window_name` | `grid_debug` | OpenCV 表示名 |

## 9. 期待される挙動
- odom 基準でグリッドを維持し、姿勢回転に影響されずに点群が配置される。
- Laser の free と Mid の hit が色分けで視認でき、静止時にスコアが蓄積する。
- 長時間停車でも減衰によりスコアが飽和しない。
- tf が得られない場合は該当フレームをスキップし、warn ログで通知する。

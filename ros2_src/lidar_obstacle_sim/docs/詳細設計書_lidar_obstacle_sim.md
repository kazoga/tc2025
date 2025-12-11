# lidar_obstacle_sim 詳細設計書

# 1. パッケージ概要

## 1.1 目的

`lidar_obstacle_sim` は、以下の条件を満たす **LiDAR 障害物回避ロジック検証用シミュレータパッケージ**である。

* ROS2 Foxy + Gazebo classic 11 を前提としたシミュレーション環境
* 自ロボットに以下2種類の LiDAR を搭載:

  * 北陽電機 UTM-30 相当の **2D LiDAR**（`/scan` / LaserScan）
  * Livox Mid-360 風の **3D LiDAR**（`/mid360/livox/lidar` / PointCloud2）
* 道幅 5m の道路コース（直進 → 左折 → 直進 → 右折）上でロボットが走行する環境
* ランダムな位置・本数（1〜3本）のパイロン（コーン）が、進行方向に対して「横一列」に配置される障害物
* Gazebo の `/odom` を元に `/amcl_pose` を擬似生成し、既存の `amcl_pose` ベースのロジックと整合を取る
* 別スクリプトで、道路中心線に沿った waypoint CSV を生成し、route_planner 系との連携の下地にも利用可能

本パッケージは、**自己位置推定自体の検証ではなく、LiDARベース障害物回避ロジックの検証を主目的**とする。

---

# 2. 前提環境

## 2.1 ソフトウェア

* OS: Ubuntu 20.04 LTS
* ROS2: Foxy Fitzroy
* Gazebo: Gazebo11（classic）
* ビルドツール: `colcon`, `ament_cmake`

## 2.2 依存パッケージ（ROS2）

* `gazebo_ros_pkgs`

  * Gazebo と ROS2 を連携し、`gazebo_ros` プラグイン（`libgazebo_ros_ray_sensor.so`, `libgazebo_ros_diff_drive.so`, `libgazebo_ros_factory.so`）を使用する
* `rclpy`
* `geometry_msgs`
* `nav_msgs`
* `gazebo_msgs`

---

# 3. 機能一覧

本パッケージの提供機能を列挙する。

1. **Gazebo シミュレーション環境起動**

   * world ファイル（`road_course.world`）を用いて Gazebo を起動
   * 自ロボット（`simple_robot`）を spawn
  * 2D LiDAR `/scan` と 3D LiDAR `/mid360/livox/lidar` を Gazebo → ROS2 へ publish
2. **ランダムパイロン生成**

   * Gazebo の `/spawn_entity` サービスを利用し、パイロンモデルを 1〜3本ランダムに横一列で配置
3. **擬似 AMCL Pose 生成**

   * `nav_msgs/Odometry` 型の `/odom` を subscribe
   * `geometry_msgs/PoseWithCovarianceStamped` 型の `/amcl_pose` を生成して publish
4. **道路中心線に沿った Waypoint CSV 生成**

   * 直進→左折→直進→右折の折れ線を道路中心線とみなす
   * 「5m進むごと」＋「屈折角が 45° 以上の曲がり角」に Pose を配置
   * `waypoints.csv` として `x,y,z,qx,qy,qz,qw` を出力

---

# 4. ディレクトリ構成

ワークスペース `ros2_ws/src` 直下に `lidar_obstacle_sim` を配置する前提とする。

```text
lidar_obstacle_sim/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── sim_with_lidars.launch.py
├── worlds/
│   └── road_course.world
├── models/
│   ├── simple_robot/
│   │   └── model.sdf
│   └── pylon/
│       └── model.sdf
└── scripts/
    ├── spawn_random_pylons.py
    ├── fake_amcl_pose.py
    └── generate_waypoints.py
```

各ファイルの役割は以下の通り。

* `CMakeLists.txt` / `package.xml`
  → ROS2 パッケージとしてビルド・インストールするための定義
* `launch/sim_with_lidars.launch.py`
  → Gazebo とノード群を一括起動
* `worlds/road_course.world`
  → 道路とロボットを配置した Gazebo world 定義
* `models/simple_robot/model.sdf`
  → 自ロボットモデル定義（本体 + 2種類の LiDAR + diff_drive プラグイン）
* `models/pylon/model.sdf`
  → パイロンモデル定義（円柱形）
* `scripts/spawn_random_pylons.py`
  → `/spawn_entity` を使ってランダムパイロン列を spawn
* `scripts/fake_amcl_pose.py`
  → `/odom` を `/amcl_pose` に変換して publish
* `scripts/generate_waypoints.py`
  → 道路中心線から waypoint CSV を生成するユーティリティスクリプト

---

# 5. ビルド・インストール設計

## 5.1 package.xml

* フォーマット: `format="3"`
* パッケージ名: `lidar_obstacle_sim`
* バージョン: `0.0.1`
* ライセンス: `Apache-2.0`
* ビルドツール: `ament_cmake`
* 依存:

  * `buildtool_depend`: `ament_cmake`
  * `depend`: `rclpy`, `geometry_msgs`, `nav_msgs`, `gazebo_msgs`
  * `exec_depend`: `gazebo_ros_pkgs`

## 5.2 CMakeLists.txt

* C++ 標準: C++14
* 利用 find_package:

  * `ament_cmake`
  * `rclpy`
  * `geometry_msgs`
  * `nav_msgs`
  * `gazebo_msgs`
* `install(DIRECTORY ...)` により

  * `launch`, `worlds`, `models` を `share/${PROJECT_NAME}` にコピー
* `install(PROGRAMS ...)` により

  * `scripts/*.py` を `lib/${PROJECT_NAME}` に配置（実行ビット要）

Python パッケージとしての `setup.py`/`setup.cfg` は利用しない。
本パッケージは CMake ベースの ROS2 パッケージとして設計する。

---

# 6. World 設計（road_course.world）

## 6.1 世界座標系と重力

* SDF version: 1.6
* world 名: `road_course_world`
* 重力: `<gravity>0 0 -9.81</gravity>`
* 光源:

  * `<include><uri>model://sun</uri></include>`
* 地面:

  * `<include><uri>model://ground_plane</uri></include>`

## 6.2 道路モデル

### 共通仕様

* 道幅: 5m
* 厚さ: 0.1m
* マテリアル: グレー（ambient/diffuse = 0.3, 0.3, 0.3, 1）
* `<static>true</static>` とする（動かない）

### 道路セグメント

1. `road_segment_1`

   * 直進道路1（x方向）
   * `size`: `<box><size>20 5 0.1</size></box>`

     * 長さ: 20m
     * 幅: 5m
   * `pose`: `<pose>10 0 0 0 0 0</pose>`

     * world x軸方向に長さ20m → 中心が (10,0)
     * 実際の伸び方: x = 0〜20, y = -2.5〜2.5 程度

2. `road_segment_2`

   * 左折道路（y方向）
   * `size`: 同上（20×5×0.1）
   * `pose`: `<pose>20 10 0 0 0 1.5708</pose>`

     * yaw = 90° (=1.5708 rad) で y方向に伸びる
     * おおよそ x = 17.5〜22.5, y = 0〜20 程度

3. `road_segment_3`

   * 右折道路（再び x方向）
   * `size`: 同上
   * `pose`: `<pose>30 20 0 0 0 0</pose>`

     * x = 20〜40, y = 17.5〜22.5 程度

## 6.3 ロボット配置

* `<include><uri>model://simple_robot</uri></include>`
* `<name>simple_robot</name>`
* `<pose>0 0 0.6 0 0 0</pose>`

  * ロボット本体高さ 1.2m の中心を z=0.6 に置くことで地面に接地している想定。

world 座標系をそのまま `map` / `odom` と同一視し、
`generate_waypoints.py` で出力する `x,y` 座標もこの world 座標を前提とする。

---

# 7. ロボットモデル設計（simple_robot）

## 7.1 ロボット本体

* モデル名: `simple_robot`
* `<static>false</static>`
* `<pose>0 0 0.6 0 0 0</pose>`

### `base_link` の形状・物理パラメータ

* link 名: `base_link`
* box サイズ:

  * `<size>0.6 0.8 1.2</size>`
  * 幅 0.6m, 長さ 0.8m, 高さ 1.2m
* 慣性:

  * 質量: 50.0kg
  * 慣性モーメント: `ixx = iyy = izz = 5.0`（簡易値）

※ 正確な物理挙動は目的ではなく、あくまで LiDAR 入力生成が目的のため、数値は概算とする。

## 7.2 2D LiDAR (UTM-30 相当)

### リンク位置

* link 名: `utm30_link`
* `<pose>0.15 0 0.4 0 0 0</pose>`（base_link に対する相対）

  * 前方 0.15m
  * 高さ 0.4m

### センサ設定

* `<sensor name="utm30" type="ray">`
* 更新レート: `<update_rate>40</update_rate>`（40Hz とする）
* スキャン設定:

  * `<samples>1080</samples>`
  * `<resolution>1</resolution>`
  * `<min_angle>-2.35619</min_angle>`（約 -135°）
  * `<max_angle>2.35619</max_angle>`（約 +135°）
* 測距設定:

  * `<min>0.1</min>`
  * `<max>30.0</max>`
  * `<resolution>0.01</resolution>`

### Gazebo → ROS2 プラグイン

* プラグイン名: `utm30_plugin`
* ファイル: `libgazebo_ros_ray_sensor.so`
* `<ros>` 設定:

* `<namespace>/</namespace>`
* `<remapping>~/out:=/scan</remapping>`

    * Gazebo 内部の出力トピック `~/out` を ROS2 トピック `/scan` に remap
* 出力型:

* `<output_type>sensor_msgs/LaserScan</output_type>`
* フレーム名:

* `<frame_name>laser</frame_name>`

→ 結果として、ROS2 側で `/scan` (LaserScan) を受信可能かつ `/laser` フレームが
  実機同様に利用できる。

## 7.3 3D LiDAR (Mid-360 風)

### リンク位置

* link 名: `mid360_frame`
* `<pose>-0.425 0 1.005 0 0 0</pose>`

  * 実機の tf `/base_link -> /mid360_frame` と同じ位置・姿勢に合わせる

### センサ設定

* `<sensor name="mid360" type="ray">`
* 更新レート: `<update_rate>10</update_rate>`
* スキャン設定:

  * 水平:

    * `<samples>900</samples>`
    * `<resolution>1</resolution>`
    * `<min_angle>-3.14159</min_angle>`（-180°）
    * `<max_angle>3.14159</max_angle>`（+180°）
  * 垂直:

    * `<samples>24</samples>`
    * `<resolution>1</resolution>`
    * `<min_angle>-0.122173</min_angle>`（約 -7°）
    * `<max_angle>0.907571</max_angle>`（約 +52°）
* 測距設定:

  * `<min>0.2</min>`
  * `<max>40.0</max>`
  * `<resolution>0.02</resolution>`

### Gazebo → ROS2 プラグイン

* プラグイン名: `mid360_plugin`
* ファイル: `libgazebo_ros_ray_sensor.so`
* `<ros>` 設定:

* `<namespace>/</namespace>`
* `<remapping>~/out:=/mid360/livox/lidar</remapping>`
* 出力型:

* `<output_type>sensor_msgs/PointCloud2</output_type>`
* フレーム名:

* `<frame_name>mid360_frame</frame_name>`

→ `/mid360/livox/lidar` (PointCloud2) を ROS2 で受信可能かつ `/mid360_frame` tf を
  実機と同名で利用できる。

## 7.4 差動駆動プラグイン（簡易版）

目的は「/cmd_vel を受けて /odom を publish し、FakeAmclPose に渡すこと」であり、
ホイールモデルは厳密でなくてよい。

* プラグイン名: `diff_drive`
* ファイル: `libgazebo_ros_diff_drive.so`
* `<ros>` 設定:

  * `<namespace>/</namespace>`
  * `<remapping>cmd_vel:=cmd_vel</remapping>`
  * `<remapping>odom:=odom</remapping>`
* パラメータ:

  * `<update_rate>50</update_rate>`
  * `<left_wheel>base_link</left_wheel>`
  * `<right_wheel>base_link</right_wheel>`

    * 実際にはホイールリンクを指定すべきだが、本設計では簡易化のため base_link を指定
  * `<wheel_separation>0.6</wheel_separation>`
  * `<wheel_diameter>0.2</wheel_diameter>`
  * `<odom_frame>odom</odom_frame>`
  * `<base_frame>base_link</base_frame>`
  * `<publish_tf>true</publish_tf>`

※ 将来的にホイールを正しくモデル化する場合は、
`left_wheel_link`, `right_wheel_link` を定義し、それらへの joint として差し替える想定。

---

# 8. ノード設計

## 8.1 spawn_random_pylons.py

### 8.1.1 概要

Gazebo 上に 1〜3本のパイロンを「横一列」でランダム配置するノード。
起動時に一度だけ `/spawn_entity` サービスを呼び、固定障害物を生成する。

### 8.1.2 ノード名

* `random_pylon_spawner`

### 8.1.3 使用する型・サービス

* 使用サービス:

  * `gazebo_msgs/srv/SpawnEntity`（`/spawn_entity`）
* 使用メッセージ:

  * `geometry_msgs/msg/Pose`（スポーン時の初期姿勢）

### 8.1.4 パラメータ

| 名称           | 型      | 既定値   | 説明                      |
| ------------ | ------ | ----- | ----------------------- |
| `model_path` | string | （空文字） | パイロンモデルの SDF ファイルパス（必須） |
| `road_width` | double | 5.0   | 道路幅[m]。横一列配置の計算に使用      |
| `x_min`      | double | 5.0   | パイロン列の中心 x 座標の下限[m]     |
| `x_max`      | double | 35.0  | パイロン列の中心 x 座標の上限[m]     |
| `y_center`   | double | 0.0   | 道路中心線の y 座標             |

### 8.1.5 トピック・サービス I/F

* クライアント:

  * `/spawn_entity` (gazebo_msgs/srv/SpawnEntity)

### 8.1.6 内部処理フロー

1. ノード起動時に `SpawnEntity` クライアントを生成
2. `/spawn_entity` サービスが利用可能になるまで最大 10 秒待機

   * 利用不可の場合、ログ出力後 `rclpy.shutdown()` して終了
3. パイロン本数を `random.randint(1, 3)` で決定
4. パイロン列の中心 x 座標を `random.uniform(x_min, x_max)` で決定
5. `_compute_offsets(num, road_width, y_center)` で各パイロンの `(dx, y)` オフセットを求める

   * 道幅5mの中央を基準に、1本の場合は中央、2本の場合は左右、3本の場合は左・中央・右とする
6. 各パイロンにつき以下を行う:

   1. `pose.position.x = x_center`
   2. `pose.position.y = y_offset`
   3. `pose.position.z = 0.35`（高さ0.7mの円柱の中心になる）
   4. name は `pylon_0`, `pylon_1`, `pylon_2` のようにユニーク化
   5. モデルファイル（`model_path`）を読み込み、`SpawnEntity.Request` に `xml` として設定
   6. `/spawn_entity` を call_async → `spin_until_future_complete` で同期完了待ち
7. 成功／失敗を `get_logger().info` / `.error` で出力
8. その後は特にループせず、ノードは起動したまま待機（パイロンは静的オブジェクトとして残る）

### 8.1.7 エラーハンドリング

* `model_path` が未設定またはファイル不存在の場合:

  * エラーログを出力し、spawn を行わない
* `/spawn_entity` サービス未起動（10秒以内に応答なし）の場合:

  * エラーログを出力し、 `rclpy.shutdown()` してノード終了
* spawn 失敗（サービス呼び出し結果が None）の場合:

  * エラーログを出力

---

## 8.2 fake_amcl_pose.py

### 8.2.1 概要

Gazebo が publish する `/odom` (nav_msgs/Odometry) を subscribe し、
その pose を `map` 座標系と同一視した `/amcl_pose` (PoseWithCovarianceStamped) として publish する。

AMCL を走らせなくても `/amcl_pose` を利用する既存ロジックがそのまま動作するようにするのが目的。

### 8.2.2 ノード名

* `fake_amcl_pose`

### 8.2.3 使用メッセージ

* 入力:

  * `nav_msgs/msg/Odometry` (`/odom`)
* 出力:

  * `geometry_msgs/msg/PoseWithCovarianceStamped` (`/amcl_pose`)

### 8.2.4 パラメータ

| 名称           | 型      | 既定値          | 説明                                   |
| ------------ | ------ | ------------ | ------------------------------------ |
| `odom_topic` | string | `/odom`      | 入力する Odometry トピック名                  |
| `amcl_topic` | string | `/amcl_pose` | 出力する PoseWithCovarianceStamped トピック名 |

### 8.2.5 処理フロー

1. `odom_topic`, `amcl_topic` をパラメータから取得
2. `odom_topic` に対して `Subscription<Odometry>` を作成
3. `amcl_topic` に対して `Publisher<PoseWithCovarianceStamped>` を作成
4. `/odom` のコールバックで以下を行う:

   * 新規 `PoseWithCovarianceStamped` を生成
   * `header.stamp = msg.header.stamp`
   * `header.frame_id = 'map'` とする（map=odom とみなす簡易実装）
   * `pose.pose = msg.pose.pose`
   * `pose.covariance` は長さ36の0埋めリスト
   * `publish` する

### 8.2.6 注意事項

* TF においては、Gazebo の diff_drive プラグインが `/odom` → `/base_link` の TF を publish することを想定する。
* `/amcl_pose` の frame_id を `map` としているため、`map→odom` の静的 TF を Identity で publish するノードを追加し、実機同様に `map` を起点とした TF 木を維持する。

---

## 8.3 generate_waypoints.py

### 8.3.1 概要

道路中心線に沿った waypoint リストを生成するスタンドアロンスクリプト。
ROS ノードではなく、純粋な Python スクリプトとして動作する。

* 折れ線（道路中心線）:

  * P0 = (0,0)
  * P1 = (20,0)
  * P2 = (20,20)
  * P3 = (40,20)
* 条件:

  * セグメントごとに 5m 間隔でサンプリング
  * セグメントの折れ曲がり角が 45° 以上である場合、その曲がり角位置にも必ず waypoint を追加
* 出力:

  * `waypoints.csv`
  * 各行: `x,y,z,qx,qy,qz,qw`

### 8.3.2 内部データ構造

```python
@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float  # [rad]
```

### 8.3.3 処理フロー

1. 折れ線点列を定義:

   * `points = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (40.0, 20.0)]`
2. `step = 5.0` [m]、`angle_threshold = 45°` をラジアンに変換
3. `waypoints` の空リストを生成
4. 各セグメント `i` について以下を行う:

   * `segment_points(p0, p1, step)` を呼び、5m刻みで Pose2D を生成

     * p0→p1の長さ L から、 `n_steps = int(L // step)`、i=0..n_steps-1 で補間
     * yaw = atan2(dy, dx) を各点に付与
   * 生成された Pose2D を `waypoints` に append
   * さらに `i < len(points)-2` （まだ次のセグメントがある）場合:

     * p1→p_next の yaw を計算
     * yaw_curr = atan2(p1 - p0)
     * yaw_next = atan2(p_next - p1)
     * 差分 dyaw = 正規化された角度差（-π〜+π）
     * `abs(dyaw) >= angle_threshold` の場合、曲がり角 p1 に yaw_next を持つ Pose2D を追加
5. 最後の終点 P3 も waypoint に追加

   * yaw は直前の waypoint の yaw をそのまま使用
6. `waypoints.csv` を生成:

   * 1行目にヘッダ: `['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']`
   * 各 waypoint について:

     * z は 0.0
     * yaw → 四元数への変換:

       * `half = yaw / 2`
       * `qx = 0`
       * `qy = 0`
       * `qz = sin(half)`
       * `qw = cos(half)`
7. 件数を標準出力に表示

### 8.3.4 想定出力例（概略）

* x: 0,5,10,15,20, 20,25,30,35,40 付近
* y: 0,0,0,0,0, 0,5,10,15,20 付近
* 曲がり角 (20,0) と (20,20) において yaw が変更される

---

# 9. Launch 設計（sim_with_lidars.launch.py）

## 9.1 概要

`ros2 launch lidar_obstacle_sim sim_with_lidars.launch.py` で以下を同時起動する。

* Gazebo（world: `road_course.world`）
* `random_pylon_spawner` ノード（パイロン列生成）
* `fake_amcl_pose` ノード（/odom → /amcl_pose）

## 9.2 実行内容

1. `gazebo` プロセス起動

   * コマンド:

     * `gazebo --verbose <world_path> -s libgazebo_ros_factory.so`
   * `world_path` は `share/lidar_obstacle_sim/worlds/road_course.world`
2. `spawn_random_pylons` ノード起動

   * package: `lidar_obstacle_sim`
   * executable: `spawn_random_pylons.py`
   * parameters:

     * `model_path`: `share/lidar_obstacle_sim/models/pylon/model.sdf`
     * `road_width`: 5.0
     * `x_min`: 5.0
     * `x_max`: 35.0
     * `y_center`: 0.0
3. `fake_amcl_pose` ノード起動

   * package: `lidar_obstacle_sim`
   * executable: `fake_amcl_pose.py`
   * parameters:

     * `odom_topic`: `/odom`
     * `amcl_topic`: `/amcl_pose`

4. 静的 tf (`/map -> /odom`)

   * package: `tf2_ros`
   * executable: `static_transform_publisher`
   * arguments: `0 0 0 0 0 0 map odom`
   * 実機では `mcl_3dl` が `/map -> /odom` を publish しており、シミュレーションでも
     同フレーム構成を維持するため恒等変換として送出する。

5. 静的 tf (`/base_link -> /mid360_frame`)

   * package: `tf2_ros`
   * executable: `static_transform_publisher`
   * arguments: `-0.425 0 1.005 0 0 0 base_link mid360_frame`

6. 静的 tf (`/base_link -> /laser`)

   * package: `tf2_ros`
   * executable: `static_transform_publisher`
   * arguments: `0.075 0 0.49 0 0 0 base_link laser`

※ 今後、`obstacle_monitor`, `robot_navigator`, `route_follower` などをこの launch に追加する想定。

---

# 10. トピック一覧

本パッケージ単体で扱う主なトピックは以下の通り。

| トピック名            | 型                                             | 発行元                         | 説明                           |
| --------------------- | --------------------------------------------- | ---------------------------- | ------------------------------ |
| `/scan`               | `sensor_msgs/msg/LaserScan`                   | Gazebo (utm30_plugin)        | UTM-30 相当 2D LiDAR           |
| `/mid360/livox/lidar` | `sensor_msgs/msg/PointCloud2`                 | Gazebo (mid360_plugin)       | Mid-360 風 3D LiDAR            |
| `/odom`               | `nav_msgs/msg/Odometry`                       | Gazebo diff_drive プラグイン | ロボットのオドメトリ           |
| `/amcl_pose`          | `geometry_msgs/msg/PoseWithCovarianceStamped` | `fake_amcl_pose` ノード      | Gazebo 真値をもとにした擬似 AMCL |
| `/cmd_vel`            | `geometry_msgs/msg/Twist`                     | 上位ノード（未実装）          | ロボットへの速度指令           |
QoS は全てデフォルト（reliable, keep last, depth=10）を想定する。

---

# 11. 想定する拡張

本詳細設計書は「最小限の構成」であり、以下の拡張余地を残している。

1. **移動障害物（他ロボット）の追加**

   * `models/other_robot` を定義
   * 新たな spawn スクリプト（例: `spawn_moving_robot.py`）で、一定速度でコースを移動するロボットを追加
   * LiDAR で他ロボットを検知し、障害物回避ロジックをさらに現実に近づける

2. **差動駆動の物理モデル改善**

   * `simple_robot` にホイールリンクとジョイントを定義
   * `diff_drive` プラグインの left/right_wheel を正しい joint 名に変更

3. **LiDAR パターンの精緻化**

   * Mid-360 の視野角・スキャンパターンを実機により近い値に修正
   * ノイズモデルの追加

4. **道路形状の複雑化**

   * 交差点・曲率の異なるカーブ・Uターン等を SDF に追加
   * `generate_waypoints.py` をパラメトリックにして、複数コースを扱えるようにする


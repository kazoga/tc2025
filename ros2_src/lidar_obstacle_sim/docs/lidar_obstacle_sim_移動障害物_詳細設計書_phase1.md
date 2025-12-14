# 日本語文書
# lidar_obstacle_sim 移動障害物 詳細設計（対向直進・一定距離停止）

## 0. 目的と前提
ROS2 Foxy + Gazebo Classic 11 環境で、直線路上をロボットと逆方向に直進し、一定距離で停止する移動障害物を追加する。実装は既存 `lidar_obstacle_sim` にプラグインとモデルを新設し、既存 world/launch をベースに追加構成を用意する。

### 0.1 確定入力条件
* ROS2 ディストリビューション: Foxy
* Gazebo: Classic（既存実装に合わせる）
* ロボット位置トピック: `/amcl_pose` を購読（`geometry_msgs/msg/PoseWithCovarianceStamped` 想定）
* 障害物サイズ: 0.5 m 立方
* 初期速度: 0.5 m/s（ロボットと逆方向へ移動）
* 停止距離: 1.0 m（ロボットとの距離が以下になったら一度だけ停止）
* 走行開始距離: 15.0 m（ロボットとの距離が以下になったら走行開始）
* plugins/ 新設、models/ 追加を許可。既存 world/launch を基に新設。

---

## 1. ディレクトリ構成設計（追加分）
* `plugins/` を新設し、Gazebo ModelPlugin を配置。
  * `plugins/moving_obstacle_plugin.cpp`
* `models/moving_obstacle/` を追加。
  * `models/moving_obstacle/model.sdf`（Box + plugin 設定）
  * `models/moving_obstacle/model.config`
* 既存 world/launch をベースに、新たな構成を最小追加。
  * `worlds/road_course_moving_obstacle.world`（既存 road_course.world を継承し、移動障害物モデルを spawn）
  * `launch/sim_with_moving_obstacle.launch.py`（既存 sim_with_lidars.launch.py をベースに移動障害物を追加）
* 既存ファイルは破壊せず、新規追加で完結する方針。

---

## 2. 移動障害物 ModelPlugin 設計
### 2.1 クラス構成
* クラス名: `MovingObstaclePlugin`
* 基底: `gazebo::ModelPlugin`
* 主なメンバ
  * `gazebo::physics::ModelPtr model_` : 障害物モデル参照
  * `gazebo::event::ConnectionPtr update_connection_` : Update イベント接続
  * `rclcpp::Node::SharedPtr ros_node_` : ROS2 ノード
  * `rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_` : ロボット位置購読
  * `ignition::math::Vector3d velocity_` : 初期速度ベクトル（SDF パラメータ）
  * `double stop_distance_` : 停止距離（SDF パラメータ）
  * `double start_distance_` : 走行開始距離（SDF パラメータ）
  * `std::string robot_pose_topic_` : ロボット位置トピック名（SDF パラメータ）
  * `bool moving_started_` : 走行開始済みフラグ
  * `bool stopped_` : 停止済みフラグ
  * `ignition::math::Pose3d robot_pose_` : 最新のロボット位置

### 2.2 状態・遷移
* 初期状態: `waiting`（`moving_started_ = false`, `stopped_ = false`）。Spawn 直後は停止し、ロボット接近を待つ。
* 更新処理: `OnUpdate()` で毎フレーム呼ばれる。
  * ロボット位置が未受信: 停止したまま待機。
  * ロボット位置を保持: モデル位置との差分を計算し、水平距離が `start_distance_` 以下になった時点で初回のみ `moving_started_ = true` とし、`velocity_` を `SetLinearVel` で与えて走行開始。
  * `moving_started_` 後: 距離が `stop_distance_` 以下になったら停止に遷移する。
* 停止処理: `stopped_` を true にし、`model_->SetLinearVel({0,0,0})` を一度だけ呼ぶ。停止後は Update でも速度を与えない。

### 2.3 使用 API
* Gazebo Classic API
  * `gazebo::ModelPlugin::Load`
  * `gazebo::physics::Model::SetLinearVel`
  * `gazebo::event::Events::ConnectWorldUpdateBegin`
* ROS2 rclcpp（Foxy）
  * `rclcpp::init`, `rclcpp::Node`
  * Subscription with QoS keep last 10
* 距離計算: `ignition::math::Vector3d` を使用し、XY 平面距離のみで判定（高度差無視）。

---

## 3. パラメータ仕様（SDF <plugin> 要素）
| パラメータ | 型 | デフォルト | 説明 |
| --- | --- | --- | --- |
| `robot_pose_topic` | string | `/amcl_pose` | ロボット位置購読トピック名 |
| `velocity` | double | `-0.5` (x 方向 m/s) | 初期直進速度。ロボットと逆方向を想定し、World X 正方向に対して負方向へ設定。必要に応じて XYZ ベクトル3要素で拡張可。|
| `stop_distance` | double | `1.0` | ロボットとの水平距離が以下になったら停止。|
| `start_distance` | double | `15.0` | ロボットとの水平距離が以下になったら走行開始。初期状態は静止。|
| `use_sim_time` | bool | `true` | ROS2 ノードでシミュレーション時間を使用。|

備考: 速度は Box 配置方向に合わせて SDF 側で符号を選択できるようにし、プラグイン側はベクトルをそのまま適用する。

---

## 4. SDF モデル設計
* 形状: Box（0.5 m 立方）。質量・慣性はデフォルト（物理影響は SetLinearVel によるため最小限）。
* Collision/Visual: `size=0.5 0.5 0.5`、シンプルな色指定のみ。
  * Plugin 埋め込み: `model.sdf` の `<plugin name="moving_obstacle_plugin" filename="libmoving_obstacle_plugin.so">` 内でパラメータを指定。
  * 例
    ```xml
    <robot_pose_topic>/amcl_pose</robot_pose_topic>
    <velocity>-0.5 0 0</velocity>
    <stop_distance>1.0</stop_distance>
    <start_distance>15.0</start_distance>
    <use_sim_time>true</use_sim_time>
    ```
* モデル配置: `road_course_moving_obstacle.world` で道路上の適切な位置（ロボット進行方向に対向する向き）へ `<include>` で配置。

### 4.1 Spawn 位置と走行開始タイミング
* **Spawn 位置**: `road_course_moving_obstacle.world` 内の `<include>` で `<pose>x y z roll pitch yaw</pose>` を明示し、ロボットの初期経路上に対向する向きで配置する。ロボットの初期姿勢に合わせて、十分な距離（例: 30〜50 m 程度）の余裕を持った位置に置くことで、`start_distance` 判定までの待機が保証される。
* **複数配置**: 同一の直線路に対して、`<include>` を複数記述することで移動障害物を複数並べる。各障害物は独立した ModelPlugin インスタンスとして動作し、個別の `<pose>` で位置と進行方向を設定する。速度・開始距離・停止距離は各 `<plugin>` ブロックで明示することで、シナリオに応じた組み合わせが可能となる。
* **走行開始トリガー**: `/amcl_pose` を購読し、XY 平面距離が `start_distance` 以下になるまでは静止。閾値を下回った瞬間に初めて速度を与えて直進を開始する。
* **停止タイミング**: 走行開始後に距離が `stop_distance` 以下になったら速度を 0 にし、再始動しない。

---

## 5. ビルド・インストール設計
* `CMakeLists.txt`
  * `plugins/moving_obstacle_plugin.cpp` を追加し、`gazebo` / `rclcpp` / `geometry_msgs` をリンクした共有ライブラリ `libmoving_obstacle_plugin.so` を生成。
  * `ament_export_libraries` と `ament_export_include_directories` を適宜追加。
  * `install(TARGETS ... LIBRARY DESTINATION lib)` でプラグインをインストール。
  * モデル・world・launch も `share/${PROJECT_NAME}/` 配下にインストール（既存パターンに合わせる）。
  * `colcon build` 実行時にプラグインライブラリがワークスペースの `install/` 以下に配置されることを前提とし、`rclcpp` などの動的リンクが必要な依存ライブラリも `package.xml` の依存として宣言して `install` に展開されるようにする。`AMENT_PREFIX_PATH` と `GAZEBO_PLUGIN_PATH` が `install` 配下を指すことで、追加の手作業なしで `ros2 launch ...` からプラグインが解決できるようにする。
* `package.xml`
  * 既存依存関係に `rclcpp`, `geometry_msgs` をプラグイン用に明記（未記載なら追加）。
* Gazebo plugin path
  * `GAZEBO_PLUGIN_PATH` で `install/lib` を参照する前提。launch で `ament_index_python` を用い、環境変数設定の追加が必要なら最小限で行う。

---

## 6. 想定ユースケース
1. **移動障害物のみの環境（複数配置対応）**: 直線路に対向してくる Box 障害物を 1 個または複数配置し、個別の `<include>` で初期位置・進行方向を指定する。各障害物が `/amcl_pose` との距離判定で独立に走行開始・停止する挙動を確認する。
2. **100m 直線路対向シナリオ（複数配置拡張）**: 100 m など十分な直線路を確保し、間隔を空けて複数の移動障害物を配置する。ロボットが順次対向し、各障害物が設定距離で停止する連続シナリオを評価する。

---

## 7. 作業範囲と留意点
* 既存ファイルの破壊的変更を避け、追加ファイル中心で構成する。
* world/launch は既存内容をコピーし、移動障害物を追加する形で新設する。
* 実装では `SetLinearVel` を毎フレーム適用し、摩擦や質量の影響を排除する。停止後は速度指令を与えない。
* ロボット位置が未取得でも動作を継続するが、取得後に停止判定を行う。

---

## 8. 衝突位置の事前試算（速度・走行開始距離の組み合わせ）
ロボットが道路中心線上を 1.07 m/s で直進し、移動障害物はロボットとの水平距離が所定閾値以下になった時点で走行を開始する。移動開始後は指定速度でロボットに向けて直進し、距離 1 m で停止した直後にロボットが追突する前提とする。

* 前提位置: ロボットは x=1.4 m、移動障害物は x=40 m に spawn され、いずれも y=0 m。
* 計算方法: 走行開始時のロボット・障害物間距離を `start_distance`、障害物速度を `v_obs`、ロボット速度を `v_robot=1.07 m/s` とする。走行開始後に距離が 1 m になるまでの時間は `(start_distance-1) / (v_robot + v_obs)`、その間に障害物が前進する距離は `v_obs × (start_distance-1) / (v_robot + v_obs)` となる。停止後にロボットが残り 1 m を詰めて衝突するため、衝突時の障害物前進量はこの値で決まる。

| 走行開始距離 start_distance [m] / 障害物速度 v_obs [m/s] | 0.4 | 0.6 | 0.8 | 1.0 |
| --- | --- | --- | --- | --- |
| 15 | 3.81 | 5.03 | 5.99 | 6.76 |
| 20 | 5.17 | 6.83 | 8.13 | 9.18 |
| 25 | 6.53 | 8.62 | 10.27 | 11.59 |

備考: 単位はいずれも「spawn 位置（x=40 m）からロボット方向へどれだけ前進したか」を示す。障害物停止距離 1 m の前提を変える場合、`start_distance-1` の部分を新しい停止距離で置き換えて再計算する。

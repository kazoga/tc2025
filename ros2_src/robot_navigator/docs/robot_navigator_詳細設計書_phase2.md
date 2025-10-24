# robot_navigator_詳細設計書_phase2

## 0. 文書目的
本書は robot_navigator パッケージの `RobotNavigator` ノード実装を対象に、route_manager 系文書と同一の章立てで詳細設計を整理する。ノードの制御責務、入出力インタフェース、内部データ構造、状態遷移、主処理およびテスト観点を明確にし、他ノード（例：`robot_simulator_node`）との連携設計を横断的に理解できるようにする。

---

## 1. ノード概要

| 項目 | 内容 |
|------|------|
| ノード名 | robot_navigator |
| クラス構成 | `RobotNavigator`（Node 派生 + 制御ロジック + CSV ログユーティリティ） |
| 主機能 | `/active_target` と自己位置から `/cmd_vel` を生成し、障害物距離に応じた時間最適制御を行う移動コントローラ |
| 実装言語 | Python3（Google Python Style + 型ヒント） |
| 対象 ROS2 ディストリビューション | Foxy |

---

## 2. 責務とスコープ

### 2.1 責務
- `/active_target` で受信した目標姿勢へ追従するための線速度・角速度を算出する。
- `/amcl_pose` と `/odom` を利用して現在の位置・姿勢・速度を推定する。
- `/scan` または `/obstacle_avoidance_hint` を基に障害物までの距離を評価し、速度制限・停止を行う。
- `/direction_marker` を publish して現在の進行方向を可視化する。
- 制御サイクルごとのデバッグ情報を CSV に記録する。

### 2.2 スコープ外
- 経路生成や複数目標管理。目標は単一 Pose とし、route_manager 等の上位ノードが供給する前提とする。
- 実機の複雑なダイナミクス（スリップ、ジャーク制限など）の再現。加速度上限による一次近似のみを行う。
- 障害物回避経路の生成。障害物ヒントを受けて減速・停止するまでを責務とし、迂回は別モジュールで実施する。

---

## 3. 入出力インタフェース

### 3.1 購読トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/odom` | `nav_msgs/Odometry` | 現在の速度を取得し、角速度・線速度のフィードバックに用いる。 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 現在位置・姿勢を取得し、目標との距離・角度計算に利用する。 |
| `/active_target` | `geometry_msgs/PoseStamped` | route_manager から配信される追従目標 Pose。 |
| `/scan` | `sensor_msgs/LaserScan` | `obstacle_distance_mode=scan` のときに障害物距離を算出する。 |
| `/obstacle_avoidance_hint` | `route_msgs/ObstacleAvoidanceHint` | `obstacle_distance_mode=hint` のときに前方クリアランス情報を受け取る。 |

### 3.2 公開トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 時間最適制御で算出した移動指令。線速度・角速度を同時に publish する。 |
| `/direction_marker` | `visualization_msgs/Marker` | 進行方向を表す RViz 用矢印。制御ループと同周期で更新する。 |

### 3.3 サービス
- なし。

---

## 4. パラメータ

| 名称 | 型 | 既定値 | 説明 |
|-------------|----|---------|------|
| `max_vel` | double | 1.1 | 線速度の上限 [m/s]。 |
| `max_w` | double | 1.8 | 角速度の上限 [rad/s]。 |
| `max_acc_v` | double | 1.0 | 線速度の加速度上限。 |
| `max_acc_w` | double | 1.5 | 角速度の加速度上限。 |
| `pos_tol` | double | 0.5 | 目標位置到達判定の距離閾値。 |
| `ang_tol` | double | 0.25 | 目標姿勢到達判定の角度閾値 [rad]。 |
| `control_rate_hz` | double | 20.0 | 制御ループ周期 [Hz]。 |
| `robot_width` | double | 0.6 | 障害物判定に用いるロボット幅。 |
| `safety_distance` | double | 0.8 | 減速開始距離。 |
| `min_obstacle_distance` | double | 0.5 | 停止距離。 |
| `obst_max_dist` | double | 5.0 | LaserScan 解析時の最大対象距離。 |
| `obstacle_distance_mode` | string | `hint` | 障害物距離の入力ソース。`scan`/`hint` を許容。 |
| `marker_frame` | string | `map` | Marker publish 時の基準フレーム。 |
| `log_csv_path` | string | `~/control_log.csv` | CSV ログ出力先。 |

---

## 5. データ構造
- `DebugInfo` dataclass：CSV ログ 1 行分の情報（制御周期、姿勢誤差、障害物距離、出力コマンドなど）を保持する。
- `RobotNavigator` 内部状態：`current_pose`、`current_velocity`、`current_goal`、`obstacle_distance`、角速度 PID の積分項など。制御計算とログで利用する。
- 障害物距離バッファ：LaserScan／ヒント由来の距離を `_latest_obstacle_distance` として保持し、タイムアウト管理を行う。

---

## 6. 状態遷移仕様

| 状態 | 概要 | 遷移契機 |
|------|------|----------|
| `WAIT_INPUT` | 必須トピックが揃っていない待機状態。安全のため `/cmd_vel` は停止指令を出す。 | 起動直後、目標や姿勢を未受信のとき |
| `TRACKING` | 目標・姿勢・速度が揃って制御を実施する状態。障害物距離で線速度を制限する。 | 必須入力を揃えたとき自動遷移 |
| `ARRIVED` | `pos_tol`／`ang_tol` を満たした状態。内部 flag ではなく `compute_time_optimal_cmd_vel()` の出力で停止制御を行う。 | 目標誤差が閾値以下になったとき |

---

## 7. 主処理仕様
- `on_goal()`: 目標 Pose を受信し `current_goal` を更新。フレーム不一致時は WARN を出して無視、正常時はゴール到達ログを初期化する。
- `on_amcl_pose()`／`on_odom()`: 現在姿勢と速度を更新し、最新の時刻を記録する。Pose 受信で位置を、Odometry 受信で速度を保持する。
- `on_scan()`／`on_hint()`: 設定に応じて障害物距離を抽出し `_apply_obstacle_distance()` により正規化。異常値は除外し、タイムスタンプで新鮮度を管理する。
- `compute_time_optimal_cmd_vel()`: 角度誤差を PID で処理し角速度を決定。線速度は角度誤差でスケールし、障害物距離によって `min_obstacle_distance` で 0、`safety_distance` で線形減衰させる。加速度上限で前周期出力との差分を制限し、結果とデバッグ情報を返す。
- `on_timer()`: 制御周期で呼び出され、必要データが揃わなければ停止指令を publish。揃っていれば `compute_time_optimal_cmd_vel()` を呼び `/cmd_vel` と Marker を出力し、CSV に追記する。ヒント・Scan のタイムアウト監視や WARN ログの抑制（5 秒間隔）も実施する。
- `_log_goal_status()`: 1Hz の補助タイマで目標と現在位置の距離・角度誤差を INFO ログに出力する。

---

## 8. クラス構成
- `RobotNavigator` クラスが Node を継承し、購読・出版・タイマ・CSV ログ初期化を行う。角度正規化ユーティリティ `normalize_angle()` と四元数→ヨー変換 `quaternion_to_yaw()` を内部関数として持つ。
- 制御ロジックの中心は `compute_time_optimal_cmd_vel()` であり、テスト容易性のために引数依存関係を整理し副作用を最小化している。

---

## 9. QoS設計
- `/cmd_vel` publisher は RELIABLE／KEEP_LAST(10) を使用し、コマンド欠落を防止する。
- `/direction_marker` publisher は RELIABLE／KEEP_LAST(1) で最新矢印のみ保持する。
- `/odom` 購読は RELIABLE／KEEP_LAST(10)、`/amcl_pose` 購読は RELIABLE／KEEP_LAST(10) とし、姿勢情報の欠落を避ける。
- `/scan` 購読は `qos_profile_sensor_data` を使用し低遅延で受信する。ヒント購読時は BEST_EFFORT／KEEP_LAST(1) で最新情報に追従する。

---

## 10. エラー処理・ログ方針
- 必須トピック未受信時は WARN ログを 5 秒間隔で出しつつ停止指令を送出し、安全側に倒す。
- `obstacle_distance_mode` が不正値のときは WARN を出して `hint` にフォールバックする。
- CSV ログのオープンに失敗した場合は例外を捕捉し WARN を出して以降のログ書き込みを無効化する。
- `/cmd_vel` 出力は NaN/Inf を検出した場合に ERROR を出してゼロ出力へ置き換える。

---

## 11. Phase3 拡張想定
- `ObstacleAvoidanceHint` の信頼度を加味した速度制御、迂回経路候補との統合。
- 速度制限の滑らかさを高めるためのジャーク制限や曲率予測によるフィードフォワード制御の導入。
- `/direction_marker` 以外の診断トピック追加（制御状態、障害物距離、ヒント新鮮度など）。

---

## 12. テスト観点

| テスト種別 | 目的 | 検証項目 |
|-------------|------|----------|
| ユニットテスト | 制御ロジック検証 | `compute_time_optimal_cmd_vel()` の角度誤差→速度スケーリング、障害物距離による減速・停止判定、`normalize_angle()` の正規化 |
| コンポーネントテスト | ノード単体検証 | `/cmd_vel` 出力上限と加速度制限、CSV ログ生成、`obstacle_distance_mode` 切り替え時の入出力挙動 |
| 結合テスト | 他ノード連携 | route_manager からの目標追従、obstacle_monitor からのヒント連携による停止挙動、robot_simulator との閉ループ走行 |
| シナリオテスト | システム試験 | 連続目標の追従、障害物急接近時の緊急停止、ヒント未受信時のフェールセーフ |

---

## 13. まとめ
- RobotNavigator は AMCL／Odometry／障害物情報を統合し、時間最適志向の `/cmd_vel` を生成するコントローラである。
- 必須トピック監視、障害物に応じた速度制御、CSV ログなどの補助機能を備え、実機・シミュレータ双方との結合に利用できる。
- 本書により route_manager・route_follower 系設計書と一貫した粒度で仕様を参照できる。

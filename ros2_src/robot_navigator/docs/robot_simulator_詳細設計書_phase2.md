# robot_simulator_詳細設計書_phase2

## 0. 文書目的
本書は robot_navigator パッケージに含まれる `RobotSimulatorNode` の現行実装を対象とし、route_manager 系文書と同一章構成で詳細設計を整理する。シミュレータの入出力、パラメータ、状態遷移、障害物を考慮した挙動、および試験観点を明確化し、RobotNavigator との結合検証を効率化する。

---

## 1. ノード概要

| 項目 | 内容 |
|------|------|
| ノード名 | robot_simulator |
| クラス構成 | `RobotSimulatorNode`（Node 派生 + `SimulatorState` dataclass + ユーティリティ関数群） |
| 主機能 | `/cmd_vel` を積分して `/odom` と `/amcl_pose` を配信する軽量移動体シミュレータ。障害物距離とコマンド上限を考慮する。 |
| 実装言語 | Python3（Google Python Style + 型ヒント） |
| 対象 ROS2 ディストリビューション | Foxy |

---

## 2. 責務とスコープ

### 2.1 責務
- `/cmd_vel` を購読し、設定された上限値でクリップした上で内部状態を更新する。
- `/active_target` 初回受信で map→odom オフセットと初期姿勢を確定し、以降のログにも利用する。
- `/amcl_pose` を再 publish して RobotNavigator に閉ループの自己位置を提供し、必要に応じて `/tf` を配信する。
- `/odom` を一定周期で配信し、他ノードがシミュレータの推定速度を参照できるようにする。
- `/cmd_vel` タイムアウト時に停止し WARN を出力する。

### 2.2 スコープ外
- 高精度な物理モデル（ジャーク・タイヤスリップ等）。一次オイラー積分のみを行う。
- 経路生成や障害物回避の判断。RobotNavigator や obstacle_monitor が担うことを前提とする。
- センサ雑音や遅延の複雑なモデル化。ガウス雑音付加のみをサポートする。

---

## 3. 入出力インタフェース

### 3.1 購読トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/cmd_vel` | `geometry_msgs/Twist` | RobotNavigator からの速度指令。`enable_cmd_limit` が真なら上限でクリップする。 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | map→odom 変換の更新に利用する。外部 Localization から供給されることも想定。 |
| `/active_target` | `geometry_msgs/PoseStamped` | 初期化とログ出力に使用。初回受信で状態を `ACTIVE` に遷移させる。 |

### 3.2 公開トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/odom` | `nav_msgs/Odometry` | 内部状態を積分して得た現在速度・姿勢を配信する。周期は `timer_publish_odom_ms` で制御。 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 現在推定姿勢を配信。雑音パラメータによって微小ノイズを加える。 |
| `/tf` | `tf2_msgs/TFMessage` | `publish_tf` が真のとき map→base_link 変換を送出する。 |

### 3.3 サービス
- なし。

---

## 4. パラメータ

| 名称 | 型 | 既定値 | 説明 |
|-------------|----|---------|------|
| `cycle_hz` | double | 10.0 | 内部更新周期 [Hz]。状態積分タイマの周期を決定する。 |
| `max_linear_mps` | double | 2.0 | 線速度指令の上限 [m/s]。`enable_cmd_limit` が真のときに適用。 |
| `max_angular_rps` | double | 3.0 | 角速度指令の上限 [rad/s]。 |
| `enable_cmd_limit` | bool | True | `/cmd_vel` を上限でクリップするか。 |
| `pose_noise_std_m` | double | 0.0 | `/amcl_pose` に加える位置雑音の標準偏差 [m]。 |
| `yaw_noise_std_deg` | double | 0.0 | `/amcl_pose` に加えるヨー雑音の標準偏差 [deg]。 |
| `publish_tf` | bool | True | `/tf` を配信するか。 |
| `frame_map` | string | `map` | TF・Odometry の親フレーム名。 |
| `frame_odom` | string | `odom` | `/odom` メッセージのフレーム。 |
| `frame_base` | string | `base_link` | TF の子フレーム。 |
| `x0` | double | 0.0 | 初期 X 座標。 |
| `y0` | double | 0.0 | 初期 Y 座標。 |
| `yaw0_deg` | double | 0.0 | 初期ヨー角 [deg]。 |
| `timer_publish_odom_ms` | int | 100 | `/odom` publish 周期 [ms]。 |
| `cmd_timeout_sec` | double | 2.0 | `/cmd_vel` が途絶した際に停止へ移行するタイムアウト [s]。 |

---

## 5. データ構造
- `SimulatorState` dataclass：位置、速度、ヨー角、最終指令時刻などを保持する。
- map→odom 変換保持変数（`_map_origin_x`、`_map_origin_y`、`_map_origin_yaw`）で外部姿勢とのずれを表現する。
- `_last_published_amcl_stamp` や `_last_published_odom_stamp` により過剰な publish を抑制する。
- `_rng`：雑音生成用の NumPy Random Generator。姿勢ノイズ付加に使用する。

---

## 6. 状態遷移仕様

| 状態 | 概要 | 遷移契機 |
|------|------|----------|
| `IDLE` | `/active_target` 未受信状態。`/cmd_vel` を受けても速度適用せずログのみを出す。 | 起動直後 |
| `ACTIVE` | 初期姿勢確定後、`/cmd_vel` を積分して `/amcl_pose`・`/odom` を更新する通常動作。 | `/active_target` 初回受信 |
| `TIMEOUT_STOP` | 指令タイムアウトによる安全停止。速度をゼロにし WARN を出す。 | `cmd_timeout_sec` 超過 |

---

## 7. 主処理仕様
- `_on_cmd_vel()`: 速度指令を受信し上限でクリップ。NaN/Inf を検出した場合は ERROR ログを出して無視する。受信時刻を `SimulatorState` に記録する。
- `_on_active_target()`: 初回受信で map→odom オフセットと初期姿勢を確定し、以降の受信はログのみ。定期的な待機ログを抑制する。
- `_on_amcl_origin()`: 外部 `PoseWithCovarianceStamped` を用いて map→odom のオフセット更新を行う。シミュレータ内部のズレを補正する。
- `_on_timer_update()`: `cycle_hz` 周期で呼び出され、最新 `/cmd_vel` を積分して状態を更新。ノイズ付加後に `/amcl_pose` を publish し、`timer_publish_odom_ms` ごとに `/odom` と `/tf` を送出する。`cmd_timeout_sec` 超過時には速度をゼロにして `TIMEOUT_STOP` へ遷移する。

---

## 8. クラス構成
- `RobotSimulatorNode`：Node 派生クラス。購読／出版設定、タイマ生成、状態保持を担う。
- `SimulatorState`：内部状態を 1 箇所にまとめる dataclass。速度、姿勢、最後のコマンド時刻などを持つ。
- ユーティリティ関数 `yaw_to_quaternion()` と `normalize_angle()` をモジュール内関数として提供し、メッセージ生成や角度正規化に用いる。

---

## 9. QoS設計
- `/cmd_vel` 購読は RELIABLE／KEEP_LAST(10) を使用し指令欠落を防ぐ。
- `/amcl_pose` publisher は RELIABLE／KEEP_LAST(1)、`/odom` publisher は RELIABLE／KEEP_LAST(10) とし、最新姿勢・速度が確実に届くようにする。
- `/active_target` 購読は RELIABLE／KEEP_LAST(10)、外部 `/amcl_pose` 購読も同一設定とし、オフセット更新の欠落を防ぐ。
- `/tf` は QoS 設定不要だが、`publish_tf` が偽の環境では送出しない。

---

## 10. エラー処理・ログ方針
- map→odom 初期化未完了で `/cmd_vel` が来た場合は INFO ログで無視を通知する。
- NaN/Inf の `/cmd_vel` を受信した際は ERROR を出し適用しない。
- `/cmd_vel` タイムアウトは WARN ログで通知し、速度ゼロを publish する。
- CSV 等の外部リソースを持たないため、例外は主に TF 生成やメッセージ生成時の ValueError を想定し、発生時は ERROR ログを出す。

---

## 11. Phase3 拡張想定
- ジャーク制限や旋回時の車両モデルを導入してより現実的な挙動に近づける。
- 動的障害物の再現や `/scan` 擬似生成機能を追加し、obstacle_monitor 単体試験を容易にする。
- タイムアウト後の自動再起動や状態機械を診断トピックで公開し、監視性を高める。

---

## 12. テスト観点

| テスト種別 | 目的 | 検証項目 |
|-------------|------|----------|
| ユニットテスト | 数値積分の検証 | `_on_timer_update()` における速度積分、角度正規化、雑音付加の境界条件 |
| コンポーネントテスト | ノード単体検証 | `/cmd_vel` 上限クリップ、タイムアウト停止、`publish_tf` 切り替え時の TF 出力 |
| 結合テスト | RobotNavigator 連携 | 目標追従シナリオで `/amcl_pose` が Navigator に正しく反映されるか、タイムアウト時に停止が伝播するか |
| シナリオテスト | システム試験 | route_manager からの連続目標に対する追従、障害物ヒント連携時の減速確認 |

---

## 13. まとめ
- RobotSimulatorNode は `/cmd_vel` の積分と簡易な雑音付加により、RobotNavigator を含むシステムの結合試験を支援する。
- 上限クリップ、タイムアウト停止、TF 配信などの安全機構を備え、実機環境を模擬する足場として活用できる。
- 本書により robot_navigator パッケージ内のシミュレータ仕様を単独で把握でき、他ノード設計書と整合した粒度を維持する。

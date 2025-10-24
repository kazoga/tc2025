# robot_simulator パッケージ README (phase2正式版)

## 概要
`robot_simulator` は `/active_target`（`PoseStamped`）を追従して疑似走行を行い、
自己位置を `/amcl_pose`（`PoseWithCovarianceStamped`）として周期配信する試験用ノードです。
Phase2 では `/scan`（`LaserScan`）を併用して障害物距離を推定し、減速・停止や
その場旋回を含む挙動に拡張しました。`publish_tf=true` の場合は
`/tf` で `map→base_link` の Transform も配信します。

## 主な機能
- `/active_target` を購読し、初回受信時に目標の背後 `init_offset_m` [m] に初期化。
  以降は **等速直線運動**で追従し、距離が増加する極小解では停止する。
- `/scan` からロボット幅内の最近傍距離を算出し、`min_obstacle_distance` 未満で停止、
  `safety_distance` 未満では線速度を線形スケールして減速する。
- 障害物停止時は `rotation_speed_degps` で目標方向へその場旋回を継続し、
  障害物解消後に再び前進する。
- `noise_pos_std_m` / `noise_yaw_std_deg` により配信値へガウスノイズを付与可能。
- `log_debug=true` で障害物距離や停止判断を INFO/WARN ログへ出力する。

## 起動方法
### launch を用いた起動
```bash
ros2 launch robot_simulator robot_simulator.launch.py \
  speed_kmph:=5.0 timer_period_ms:=100 publish_tf:=false
```
- launch 引数は `speed_kmph` や `init_offset_m` など主要パラメータを直接上書き可能。
- `/scan` 入力が無い場合は障害物制御を行わず、純粋な等速追従となる。

### 実行ファイルを直接起動
```bash
ros2 run robot_simulator robot_simulator
```
- `ros2 param set` で障害物関連パラメータを動的に変更できます。

## 外部インタフェース
### Subscriber
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/active_target` | `geometry_msgs/PoseStamped` | 追従目標。`frame_id` 不一致は警告のみで処理継続。 | RELIABLE / VOLATILE / depth=10 |
| `/scan` | `sensor_msgs/LaserScan` | 前方障害物距離を計測。未接続時は障害物制御が無効。 | RELIABLE / VOLATILE / depth=10 |

### Publisher
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | シミュレータの自己位置。`timer_period_ms` ごとに更新。 | RELIABLE / VOLATILE / depth=10 |
| `/tf` | `tf2_msgs/TFMessage` | `publish_tf=true` の場合に `map→base_link` を配信。 | RELIABLE / VOLATILE / depth=10 |

> サービス・アクションは提供しません。

## パラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `speed_kmph` | double | `5.0` | 巡航速度 [km/h]。内部で m/s に変換。
| `timer_period_ms` | int | `20` | 更新周期 [ms]。旧 Phase1 の 100 ms から細分化可能。
| `frame_id` | string | `map` | `/amcl_pose`・`/tf` の親フレーム。
| `child_frame_id` | string | `base_link` | `/tf` の子フレーム。
| `publish_tf` | bool | `False` | `True` で `/tf` を配信。
| `init_offset_m` | double | `5.0` | 初期化時に目標背後へ下げる距離。
| `stop_radius_m` | double | `1.0` | 極小解停止の判定距離。
| `noise_pos_std_m` | double | `0.0` | 位置ノイズの標準偏差 [m]。
| `noise_yaw_std_deg` | double | `0.0` | ヨー角ノイズの標準偏差 [deg]。
| `log_debug` | bool | `False` | 障害物関連の詳細ログを出力。
| `robot_width` | double | `0.6` | 障害物帯の横幅判定に使用するロボット幅。
| `max_detection_distance` | double | `5.0` | `/scan` から検知する最大距離。
| `safety_distance` | double | `1.0` | 減速を開始する距離。
| `min_obstacle_distance` | double | `0.8` | 停止を指示する距離。
| `max_decel_mps2` | double | `1.0` | 減速時の最大減速度 [m/s^2]。
| `enable_obstacle_stop` | bool | `True` | 障害物制御を有効化。
| `rotation_speed_degps` | double | `30.0` | 停止時の回頭速度 [deg/s]。

## 状態管理・処理フロー
1. 初回の `/active_target` 受信で背後 `init_offset_m` に初期姿勢を設定し、`have_target` を
   有効化する。
2. タイマーごとに目標位置との距離・方向を算出し、等速直線で候補位置を生成する。
3. 極小解停止条件（距離≦`stop_radius_m` かつ次ステップで距離増加）を満たす場合は停止を継続。
4. `/scan` から得た最近傍距離が `min_obstacle_distance` 未満なら並進を止め、目標方向へ
   `rotation_speed_degps` で旋回する。`safety_distance` 未満では線速度をスケールする。
5. 新しい目標が来た場合は `target_changed` をリセットして再加速する。
6. 出力時にノイズ設定を反映し、`/amcl_pose` と `/tf`（有効時）を Publish する。

## 動作確認手順
1. `/active_target` を手動で Publish する。
   ```bash
   ros2 topic pub /active_target geometry_msgs/PoseStamped \
     "{header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" -1
   ```
2. `robot_simulator` を起動し、`/amcl_pose` が 20 ms 周期で更新されることを確認する。
3. `/scan` を模擬する場合は `obstacle_monitor` の `laser_scan_simulator` などで入力し、
   障害物距離に応じて速度ログや停止挙動が変化することを確認する。
4. `/tf` を利用する場合は `publish_tf:=true` を指定し、`ros2 topic echo /tf` で Transform を確認する。

## デバッグのヒント
- `log_debug=true` で停止判断や距離計測を WARN/INFO で出力する。
- 障害物距離が常に `None` の場合は `/scan` の角度範囲と `robot_width` が適切かを確認する。
- タイマー周期を短くすると CPU 負荷が増えるため、実環境に合わせて `timer_period_ms` を調整する。

## 将来拡張メモ
- `/active_target` の耐障害性向上のため、TRANSIENT_LOCAL QoS への戻しも検討中。
- `/scan` が無い環境向けに仮想障害物シナリオをパラメータ化する予定。

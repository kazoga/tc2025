# route_follower パッケージ README (phase2正式版)

## 概要
`route_follower` は `route_manager` から配信される `/active_route` を追従し、
現在のターゲット Pose を `/active_target` に Publish する経路追従ノードです。Phase2 では ROS2
ラッパ層 (`route_follower_node.py`) とロジック中核 (`follower_core.py`) を分離し、滞留検知と
`/report_stuck` サービス呼び出しを統合しています。

## 主な機能
- `/active_route` 受信時に `FollowerCore` へ適用し、`start_immediately` 設定に応じて自動開始。
- `/amcl_pose`・`/obstacle_avoidance_hint`・`/sig_recog`・`/manual_start` を購読し、追従状態や滞留判定に利用。
- `FollowerCore.tick()` の結果を `/active_target` と `/follower_state` へ出力し、滞留統計も Publish。
- 滞留検知時に `/report_stuck` を呼び出し、`decision_code` や `offset_hint` を反映して走行継続・スキップ・失敗を判定。
- 1/`resend_interval_sec` [Hz] でターゲット Pose を再送し、制御系へ冪等な指示を提供。

## 起動方法
```bash
ros2 launch route_follower route_follower.launch.py \
  arrival_threshold:=0.6 \
  control_rate_hz:=20.0 \
  resend_interval_sec:=1.0 \
  start_immediately:=true \
  target_frame:=map
```

- launch 引数はノードパラメータへそのまま反映される。
- `param_file` で YAML (`params/default.yaml`) を差し替えることも可能。

## 外部インタフェース
### 購読トピック
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/active_route` | `route_msgs/Route` | 追従対象ルート。`Route.version` と `start_index` を `FollowerCore` に転送。 | RELIABLE / TRANSIENT_LOCAL |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 現在姿勢。2D yaw を算出して `FollowerCore` へ投入。 | RELIABLE / VOLATILE |
| `/obstacle_avoidance_hint` | `route_msgs/ObstacleAvoidanceHint` | 障害物回避ヒント。最新値を統計化し滞留判定に利用。 | BEST_EFFORT / VOLATILE |
| `/manual_start` | `std_msgs/Bool` | TRUE 受信で手動再開。 | RELIABLE / VOLATILE |
| `/sig_recog` | `std_msgs/Int32` | 信号認識結果（1=GO, 2=NOGO 等）。 | RELIABLE / VOLATILE |

### 配信トピック
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/active_target` | `geometry_msgs/PoseStamped` | 現在向かうべきターゲット Pose。`target_frame` を frame_id に設定。 | RELIABLE / VOLATILE |
| `/follower_state` | `route_msgs/FollowerState` | 状態・route_version・滞留統計を Publish。 | RELIABLE / VOLATILE |

### サービスクライアント
| 名称 | 型 | 説明 |
|------|----|------|
| `/report_stuck` | `route_msgs/srv/ReportStuck` | 滞留判定時に呼び出し。現在 index やオフセット履歴を送信し、再計画方針を受け取る。 |

## パラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `arrival_threshold` | double | `0.6` | Waypoint 到達判定距離[m]。 |
| `control_rate_hz` | double | `20.0` | `FollowerCore.tick()` を呼ぶ周期[Hz]。0 以下の場合は 20Hz に補正。 |
| `resend_interval_sec` | double | `1.0` | `/active_target` 再送周期[s]。内部で `republish_target_hz = 1.0 / resend_interval_sec` を設定。 |
| `start_immediately` | bool | `true` | ルート受信直後に自動で RUNNING へ遷移するか。 |
| `target_frame` | string | `"map"` | Publish する Pose の frame_id。 |

## 状態管理・処理フロー
### Follower 状態遷移
`FollowerCore` は以下の状態を持ち、`/follower_state.state` に文字列として出力します。

| 状態 | 説明 |
|------|------|
| `IDLE` | ルート未適用。`start_immediately=false` で manual_start 待ち。 |
| `RUNNING` | 通常追従中。 |
| `WAITING_STOP` | line_stop に到達し手動再開待ち。`/manual_start` で RUNNING へ復帰。 |
| `STAGNATION_DETECTED` | 滞留を検知し、局所回避や待機準備を実施。 |
| `AVOIDING` | 回避サブゴール（横オフセットなど）を実行中。 |
| `WAITING_REROUTE` | `/report_stuck` 応答待ち。サービス結果に応じて再開または失敗判定。 |
| `FINISHED` | 最終 Waypoint へ到達。 |
| `ERROR` | frame_id 不一致など致命的エラー。 |

### `/report_stuck` 呼び出しフロー
1. `FollowerCore` が滞留を検知すると `WAITING_REROUTE` に遷移し、ノード側で `/report_stuck` を非同期呼び出しする。
2. リクエストには `route_version`・`current_index`・`current_wp_label`・現在 Pose・ヒント統計などを含める。
3. 応答が `DECISION_REPLAN` や `DECISION_SKIP` の場合は `offset_hint` や skip 指示を `FollowerCore` に反映する。
4. `DECISION_FAILED` の場合は `notify_reroute_failed()` を呼び出し `ERROR` へ遷移してオペレータ介入を要求する。

## 動作確認手順
1. `route_manager` を起動し、`/active_route` と `/report_stuck` が利用可能な状態にする。
2. 本ノードを起動後、`ros2 topic echo /follower_state` で状態遷移を監視する。
3. `/manual_start` に `std_msgs/Bool data:true` を Publish し、`WAITING_STOP` から再開できることを確認する。
4. `/report_stuck` を `ros2 service call` 等で模擬し、`decision_code` に応じたログと状態変化を確認する。

## デバッグのヒント
- `/active_route` の `header.frame_id` が `target_frame` と異なる場合は `ERROR` へ遷移する。空文字は自動補完される。
- `/obstacle_avoidance_hint` が届かない場合は滞留判定統計が十分に貯まらず、`front_blocked_majority` が False のままになる。
- `/report_stuck` が利用できない状態で滞留するとリトライ待ちとなり、ログに "not ready" が出力される。サービス起動順を確認する。

## 将来拡張メモ
- `FollowerCore` には信号判定・横オフセット試行回数などの内部パラメータが多数定義されており、必要に応じて ROS パラメータ化を検討している。
- `WAITING_REROUTE` 中のタイムアウト（`reroute_timeout_sec`）は Core 内部で管理しており、ノード側で警告を出す機能追加を予定。

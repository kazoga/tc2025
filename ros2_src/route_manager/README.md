# route_manager パッケージ README (phase2正式版)

## 概要
`route_manager` は `route_planner` が提供する `/get_route`・`/update_route` サービスを利用し、
経路データの取得・配信・状態管理を統括する経路管理ノードです。Phase2 では ROS2 ノード層
(`route_manager_node.py`) とコアロジック (`manager_core.py` / `manager_fsm.py`) を分割し、
非同期 FSM で経路要求と再計画を制御します。

## 主な機能
- 起動直後に `/get_route` を呼び出し、初期ルートと PNG 画像を取得。
- `/active_route` を **TRANSIENT_LOCAL** QoS で配信し、再起動後も最新ルートを取得可能にする。
- `/route_state`・`/manager_status` で FSM 状態・再計画判断・ルートバージョンを通知。
- `/mission_info` に start / goal / checkpoint の要求条件を Publish。
- `/report_stuck` で滞留報告を受け取り、`/update_route`・shift・skip を段階的に判断。
- 再計画結果を `ReportStuck.Response` の `decision_code`・`offset_hint`・`note` へ反映。

## 起動方法
### launch を用いた起動
```bash
ros2 launch route_manager route_manager.launch.py \
  start_label:=START \
  goal_label:=GOAL \
  checkpoint_labels:="['P1','P2']"
```

- `start_label` / `goal_label` はルートをスライスするラベル（未指定時は先頭〜末尾）。
- `checkpoint_labels` は YAML 既定値に追加で通過させたいラベルを配列で指定。
- 起動直後は `/get_route` 接続を待ち、接続後に FSM から初期要求が送出される。

## 外部インタフェース
### Publisher
| 名称 | 型 | 説明 | QoS |
|------|----|------|-----|
| `/active_route` | `route_msgs/Route` | 現在有効なルート。`Route.version` は major*100 + minor で管理。 | RELIABLE / TRANSIENT_LOCAL / depth=1 |
| `/route_state` | `route_msgs/RouteState` | 現在 index・ラベル・ステータスを通知。 | RELIABLE / VOLATILE / depth=10 |
| `/mission_info` | `route_msgs/MissionInfo` | 現在の start / goal / checkpoint を配信。 | RELIABLE / TRANSIENT_LOCAL / depth=1 |
| `/manager_status` | `route_msgs/ManagerStatus` | FSM 状態と再計画判断結果を通知。 | RELIABLE / VOLATILE / depth=10 |

### Service Client
| 名称 | 型 | 説明 |
|------|----|------|
| `/get_route` | `route_msgs/srv/GetRoute` | 初期ルート取得。成功時は `Route.version=1` で保存。 |
| `/update_route` | `route_msgs/srv/UpdateRoute` | 滞留時の再計画。成功時に部分ルートを差し替え、`Route.version` を進める。 |

### Service Server
| 名称 | 型 | 説明 |
|------|----|------|
| `/report_stuck` | `route_msgs/srv/ReportStuck` | `route_follower` からの滞留通報を受け付け、再計画方針と `offset_hint` を返す。 |

> 本ノードは購読トピックを持たず、滞留状況は `/report_stuck` 要求に含まれる情報を利用します。

## パラメータ
| 名称 | 型 | 既定値 | 概要 |
|------|----|--------|------|
| `start_label` | string | `""` | 初期ルート要求時の開始ラベル。空文字で先頭から開始。 |
| `goal_label` | string | `""` | 初期ルート要求時の終了ラベル。空文字で末尾まで。 |
| `checkpoint_labels` | string[] | `[]` | 追加で通過させたいチェックポイント一覧。 |
| `planner_timeout_sec` | double | `5.0` | `/update_route` 呼び出し時のサービス待機タイムアウト。 |
| `planner_retry_count` | int | `2` | `/update_route` が失敗した場合の再試行回数。 |
| `planner_connect_timeout_sec` | double | `10.0` | `/get_route` 接続待ちの打ち切り時間。 |
| `state_publish_rate_hz` | double | `1.0` | `/route_state` 再送タイマーの周期（再送実装補完用）。 |
| `image_encoding_check` | bool | `false` | 受信画像の encoding を検証するか（将来拡張用）。 |
| `report_stuck_timeout_sec` | double | `5.0` | `/report_stuck` 処理の許容待ち時間（拡張用）。 |
| `offset_step_max_m` | double | `1.0` | Shift 回避で適用する横方向オフセットの上限値[m]。 |

## 状態管理・処理フロー
### FSM 状態遷移
`manager_fsm.py` に実装された FSM は以下の状態遷移を持ちます。

| FSM 状態 | 説明 | `/route_state.status` へのマッピング |
|----------|------|---------------------------------------|
| `IDLE` | 起動直後・ルート未取得 | `STATUS_IDLE` |
| `REQUESTING` | `/get_route` 応答待ち | `STATUS_RUNNING` |
| `ACTIVE` | ルート配信中 | `STATUS_RUNNING` |
| `WAITING_REROUTE` | 滞留中で再計画待ち | `STATUS_HOLDING` |
| `UPDATING` | `/update_route` 応答待ち | `STATUS_UPDATING_ROUTE` |
| `COMPLETED` | ルート完走（follower から完了通報） | `STATUS_COMPLETED` |
| `ERROR` | サービス失敗・例外 | `STATUS_ERROR` |

`/manager_status` には `state`（上表）と `decision`（replan / shift / skip / failed）を出力します。

> Phase2 では走行開始タイミングの制御を `route_follower` の `start_immediately` パラメータに統一し、
> `route_manager` は起動直後に必ず初期ルートを要求します。`start_immediately=false` の場合は
> `/manual_start` メッセージを受信した時点で `route_follower` が走行を開始します。

### `/report_stuck` に対する再計画フロー
1. Core が滞留報告を受理し、ルートバージョン・現在 index・滞留理由を検証する。
2. `/update_route` を呼び出し、新ルートが得られればそのまま適用する。
3. `/update_route` が失敗した場合は `offset_step_max_m` を上限とした左右オフセット提示（shift）を試みる。
4. それでも走行不能な場合は次ウェイポイントのスキップ（skip）を試行し、部分ルートを再配信する。
5. すべて失敗した場合は `decision_code=FAILED` とし、`note` に原因を格納する。
6. `offset_hint` には follower が採用すべき横方向オフセット量（m）を設定する。

## 動作確認手順
1. `route_planner` を起動し、`/get_route` `/update_route` サービスが利用可能であることを確認する。
2. 上記コマンドで `route_manager` を起動し、初期ルート取得後に `/active_route` が Publish されることを確認する。
3. 滞留状況を模擬する場合は以下のように `/report_stuck` を呼び出す。
   ```bash
   ros2 service call /report_stuck route_msgs/srv/ReportStuck \
     "{route_version: 1, current_index: 5, current_wp_label: 'W5',
        current_pose_map: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}}},
        reason: 'front_blocked', avoid_trial_count: 2, last_hint_blocked: true,
        last_applied_offset_m: 0.5}"
   ```
   応答の `decision_code` や `offset_hint` を `/manager_status` と合わせて確認する。
4. `/route_state` や `/manager_status` を `ros2 topic echo` で監視し、FSM 状態遷移が意図どおりであることを確認する。

## デバッグのヒント
- `/manager_status` の `decision` が `skip` の場合、Core がローカルに経路をスライスして再配信している。
- `/update_route` が連続で失敗する場合は、`route_planner` 側の閉塞ノード指定や YAML 設定を確認する。
- `planner_connect_timeout_sec` を短く設定すると、`/get_route` サービスが未起動のまま一定時間経過した場合に `ERROR` へ遷移する。

## 将来拡張メモ
- `image_encoding_check` および `report_stuck_timeout_sec` は将来的な検証ロジック追加のためのパラメータ。
- `/route_state` の再送タイマ（`state_publish_rate_hz`）は今後再送実装を補完する予定。

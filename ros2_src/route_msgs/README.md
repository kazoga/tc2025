# route_msgs パッケージ README (phase3正式版)

## 概要
`route_msgs` は経路計画・走行系で共通利用するメッセージ／サービス型を定義する
インタフェース専用パッケージです。Phase3 ではルートバージョン管理、滞留報告の
理由コード化（`reason_code` / `reason_detail`）、経路封鎖判定用フラグなどのフィールドを拡張し、
`route_manager`・`route_planner`・`route_follower`・`obstacle_monitor`・`robot_navigator` など
複数ノード間のデータ交換を統一しました。

## 主な機能
- 経路データ (`Route`・`Waypoint`) と状態通知 (`RouteState`・`ManagerStatus`) のメッセージ定義。
- ミッション設定 (`MissionInfo`) やフォロワ状態 (`FollowerState`) の共有メッセージ定義。
- `/get_route`・`/update_route`・`/report_stuck` 向けサービス定義。
- 障害物回避支援情報 (`ObstacleAvoidanceHint`) の定義。

## 起動方法
本パッケージはノードを持たず、`colcon build` により他パッケージと同時にビルドされます。
インタフェースの確認には以下を利用してください。
```bash
ros2 interface show route_msgs/msg/Route
ros2 interface show route_msgs/srv/ReportStuck
```

## 外部インタフェース
本パッケージはトピックやサービスサーバを提供しませんが、以下のメッセージ／サービス型を
他パッケージが使用します。

### メッセージ定義
| 名称 | 概要 | 主な利用ノード |
|------|------|---------------|
| `Route` | 経路全体。`version`・`route_image`・`Waypoint[]` を保持。 | `route_manager`, `route_planner` |
| `Waypoint` | 個別経路点。`label`・`pose`・停止/スキップフラグを含む。 | `route_planner`, `route_follower` |
| `RouteState` | 進行中ルートの概要（現在 index・status）。 | `route_manager`, 監視 UI |
| `ManagerStatus` | FSM 状態と再計画判断 (`decision`) を通知。 | `route_manager`, `route_follower` |
| `MissionInfo` | start/goal/チェックポイントの情報。 | `route_manager`, ダッシュボード |
| `FollowerState` | フォロワの状態・滞留統計を報告。 | `route_follower`, ログ収集 |
| `ObstacleAvoidanceHint` | 前方閉塞・左右オフセットのヒント。 | `obstacle_monitor`, `robot_navigator`, `route_follower` |

### サービス定義
| 名称 | 概要 | リクエスト主要項目 | 主な利用ノード |
|------|------|------------------|---------------|
| `GetRoute` | 初期ルートを取得。応答で `Route` を返す。 | `mission_id` 相当フィールドは無し。 | `route_manager` → `route_planner` |
| `UpdateRoute` | 滞留時の部分ルート差し替え。 | `prev_index`・`next_index`・`route_version`・`reason`。 | `route_manager` ↔ `route_planner` |
| `ReportStuck` | 滞留報告と再計画結果返却。 | `route_version`・`current_index`・`reason_code`・`reason_detail`・`avoid_trial_count`。 | `route_follower` → `route_manager` |

## パラメータ
- 本パッケージが公開するパラメータはありません。

## 状態管理・処理フロー
- `Route.version` は `route_manager` が major/minor を 100 倍で符号化し、`/active_route` 配信の世代管理に利用します。
- `RouteState.status` は Phase3 の FSM (`STATUS_IDLE`～`STATUS_ERROR`) と 1 対 1 に対応します。
- `ReportStuck.Response.decision_code` は `route_manager` の再計画・シフト・スキップ判断に一致し、
  `offset_hint` が `route_follower` への横オフセット指示となります。
- `ObstacleAvoidanceHint.front_clearance_m` は `robot_navigator`・`route_follower` の減速／停止条件に利用されます。

## 動作確認手順
1. `colcon build` 後、`ros2 interface show route_msgs/msg/ManagerStatus` などで定義内容を確認する。
2. `ros2 interface proto route_msgs/msg/Route` を用いてテストデータを生成し、Publish/Service 呼び出しの型整合を確認する。
3. `route_manager`・`route_planner`・`route_follower` 間で通信ログを収集し、定義どおりのフィールドが送受信されているか検証する。

## デバッグのヒント
- `ros2 interface check` を用いて独自に生成した YAML が型に適合しているかを確認してください。
- メッセージ拡張時は `route_manager`・`route_planner` など関連パッケージの README と docs を同時に更新すること。
- `ObstacleAvoidanceHint` の `front_clearance_m` が `inf` になりやすい場合は、送信側で距離上限を再確認します。

## 将来拡張メモ
- Phase3 では `Route` の属性にレーン種別や速度プロファイルを追加予定です。
- `ReportStuck` に `recovery_command` を追加し、フォロワ側への直接指示を可能にする構想があります。

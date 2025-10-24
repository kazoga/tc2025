# route_follower_詳細設計書（Phase2 最終版・完全統合版）

## 0. 文書目的
本書は route_follower ノード（Phase2）の実装に整合する詳細設計を提供する。ROS2依存層と非ROSロジック層を分離した現行実装に基づき、I/O仕様・状態遷移・滞留検知および回避処理の挙動を明記する。

---

## 1. ノード概要

| 項目 | 内容 |
|------|------|
| ノード名 | route_follower |
| 構成 | `RouteFollowerNode`（ROS2 I/F） + `FollowerCore`（非ROSロジック） |
| 主機能 | 経路追従、滞留検知、障害物回避（L字）、停止制御、`/report_stuck` 要求 |
| 実装言語 | Python3（Google Python Style + 型ヒント） |
| 対象ROS2ディストリビューション | Foxy |

`RouteFollowerNode` はROSメッセージ変換・QoS設定・サービス呼び出しを担当し、周期タイマーで `FollowerCore.tick()` を呼び出す。`FollowerCore` は最新入力のスナップショットを取得して状態遷移・回避計画を決定する。

---

## 2. 責務とスコープ

- `/active_route` で受信した経路を `FollowerCore.Route` 構造体へ変換し、追従対象として保持する。
- `/amcl_pose`、`/obstacle_avoidance_hint`、`/manual_start`、`/sig_recog` から取得した情報を Core の mail box に蓄え、`tick()` で一貫性のあるスナップショットを用いて処理する。
- 滞留検知・L字回避シーケンス・WAITING_STOP／WAITING_REROUTE 状態管理を Core 側で実施し、Node 側で `/active_target`・`/follower_state` をpublishする。
- Core が WAITING_REROUTE 状態へ遷移した際に `RouteFollowerNode` が `/report_stuck` (route_msgs/srv/ReportStuck) を非同期で呼び出し、結果に応じて Core へ成功／失敗を通知する。

---

## 3. 入出力インタフェース

### 3.1 購読トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/active_route` | `route_msgs/Route` | 経路情報（TRANSIENT_LOCAL）。受信後 `FollowerCore.update_route()` を呼ぶ。 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 現在推定位置。Yawへ変換後 `update_pose()` を呼ぶ。 |
| `/obstacle_avoidance_hint` | `route_msgs/ObstacleAvoidanceHint` | 前方障害物ヒント。サンプルを `FollowerCore.update_hint()` へ渡す。 |
| `/manual_start` | `std_msgs/Bool` | STOP解除指示。True立ち上がり時に `update_control_inputs(manual_start=True)`。 |
| `/sig_recog` | `std_msgs/Int32` | 信号判定（1=GO, 2=NOGO）。最新値を `update_control_inputs(sig_recog=...)`。 |

### 3.2 公開トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/active_target` | `geometry_msgs/PoseStamped` | 追従中の目標Pose。Coreの出力Poseを必要時のみ再送。 |
| `/follower_state` | `route_msgs/FollowerState` | 状態情報。Coreの辞書をROSメッセージへ変換してpublish。 |

### 3.3 サービス

| 名称 | 型 | 説明 |
|------|----|------|
| `/report_stuck` | `route_msgs/srv/ReportStuck` | WAITING_REROUTE遷移時にNodeが呼び出す外部サービス。 |

---

## 4. パラメータ

### 4.1 ノードパラメータ

| 名称 | 型 | 既定値 | 説明 |
|------|----|---------|------|
| `arrival_threshold` | double | 0.6 | Waypoint到達判定距離[m]。Coreの `arrival_threshold` に反映。 |
| `control_rate_hz` | double | 20.0 | `tick()` 呼び出し周期[Hz]。0以下の場合は20Hzへフォールバック。 |
| `resend_interval_sec` | double | 1.0 | `/active_target` の保険再送間隔[s]。Coreの `republish_target_hz` に反映。 |
| `start_immediately` | bool | True | 新ルート適用時に手動開始を待たずRUNNINGへ入るか。 |
| `target_frame` | string | "map" | `/active_target` のframe_id。 |

### 4.2 Core内部パラメータ
`FollowerCore` 内で初期化される主要定数（ROSパラメータ化されていない）。

| 名称 | 既定値 | 用途 |
|------|--------|------|
| `window_sec` | 2.0 | 滞留検知窓幅[s] |
| `progress_epsilon_m` | 0.10 | 滞留判定距離閾値[m] |
| `min_speed_mps` | 0.05 | 滞留判定速度閾値[m/s] |
| `stagnation_duration_sec` | 15.0 | 滞留成立までの継続時間[s] |
| `stagnation_grace_sec` | 2.0 | ルート切替・回避後の猶予[s] |
| `avoid_min_offset_m` | 0.35 | L字横シフト最小値[m] |
| `avoid_max_offset_m` | 5.0 | L字横シフト最大値[m] |
| `avoid_forward_clearance_m` | 2.0 | L字前進距離[m] |
| `max_avoidance_attempts_per_wp` | 2 | waypoint毎の回避最大回数 |
| `reroute_timeout_sec` | 30.0 | WAITING_REROUTEのタイムアウト[s] |
| `hint_cache_window_sec` | 5.0 | Hintサンプル保持窓[s] |
| `hint_majority_true_ratio` | 0.8 | front_blocked多数決割合 |
| `hint_min_samples` | 5 | Hint評価に必要な最少サンプル数 |

---

## 5. データ構造

### 5.1 Core内モデル
- `Pose(x, y, yaw)`
- `Waypoint(label, pose, line_stop, signal_stop, left_open, right_open)`
- `Route(version, waypoints, start_index, start_waypoint_label)`
- `HintSample(t, front_blocked, left_offset, right_offset, front_clearance)`
- `FollowerOutput(target_pose: Optional[Pose], state: Optional[dict])`

### 5.2 `/follower_state` 出力項目
`FollowerCore._make_state_dict()` が生成する辞書を `RouteFollowerNode._handle_state_publish()` がメッセージへ変換する。

| フィールド | 説明 |
|------------|------|
| `route_version` | 現在適用中の経路バージョン。 |
| `state` | `FollowerStatus` 列挙の名称（`IDLE`、`RUNNING` 等）。 |
| `current_index` | 現在のWaypointインデックス。 |
| `front_blocked_majority` | Hint多数決の前方閉塞判定。 |
| `left_offset_m_median` / `right_offset_m_median` | Hintサンプルの中央値[m]。 |
| `avoidance_attempt_count` | 当該Waypointでの回避試行回数。 |
| `last_stagnation_reason` | 最新の滞留理由（`no_hint`、`no_space` 等）。 |

---

## 6. 状態遷移仕様

`FollowerStatus` 列挙により以下の状態を管理する。

| 状態 | 概要 |
|------|------|
| `IDLE` | 経路未設定／待機。 |
| `RUNNING` | 経路追従中。 |
| `WAITING_STOP` | line_stop／signal_stopで一時停止。 |
| `STAGNATION_DETECTED` | 滞留検出後の内部遷移（実装では `RUNNING` 内で処理、列挙値は保持のみ）。 |
| `AVOIDING` | L字回避サブゴール処理中。 |
| `WAITING_REROUTE` | `/report_stuck` 要求送信後の再ルート待機。 |
| `FINISHED` | 経路完了。 |
| `ERROR` | タイムアウトや致命的失敗。 |

WAITING_REROUTEは `route_active` フラグで解除を検知し、`reroute_wait_deadline` を過ぎた場合は `ERROR` に遷移する。

---

## 7. 主処理仕様

### 7.1 `tick()` の全体フロー
1. `/active_route`・`/amcl_pose` メールボックスから最新メッセージを取り出してクリア。
2. ルート適用時にインデックス・回避状態・滞留履歴を初期化し、`stagnation_grace_until` を更新。
3. 現在Poseを履歴へ蓄積し、ヒント統計のスナップショットを取得。
4. manual_start／sig_recog のスナップショットを取得。
5. 状態に応じて `_tick_main()` が目標Pose・状態遷移を決定し、`FollowerOutput` を返す。

### 7.2 滞留検知
- 直近 `window_sec` の移動距離 < `progress_epsilon_m` かつ平均速度 < `min_speed_mps` の状態が `stagnation_duration_sec` 継続した場合に滞留成立。
- STOP解除直後 (`stagnation_grace_until`) や回避中の猶予期間中は判定しない。

### 7.3 L字回避
- Hint多数決が `front_blocked=True` かつ中央値が閾値を満たす場合に `_start_avoidance_sequence()` を実行。
- シフト方向は左右開放長とHint中央値から決定し、横移動→前進の2ステップをキューへ積む。
- サブゴール到達で次のステップへ移行し、完了後は RUNNING へ復帰。
- 回避中に再滞留した場合は `avoidance_failed` を理由に WAITING_REROUTE へ遷移。

### 7.4 WAITING_STOP
- `line_stop` 到達: `/manual_start` Trueで解除。
- `signal_stop` 到達: `/manual_start` True または `/sig_recog==1` で解除。
- 解除後は次Waypointへ進み、短い滞留猶予(1秒)を再設定する。最終Waypointの場合は `FINISHED`。

### 7.5 `/report_stuck` 呼び出し
- Coreが WAITING_REROUTE へ入ると `RouteFollowerNode._handle_stuck_report()` が `/report_stuck` クライアントを起動する。
- 非同期コールバックで結果を受け取り、`ReportStuck.Response.DECISION_FAILED` の場合は `FollowerCore.notify_reroute_failed()` を呼んで `ERROR` へ遷移させる。
- 成功時は Core 側が新ルート適用（`route_active=True`）を検知して RUNNING に復帰する。

### 7.6 `/active_target` 再送制御
- Coreから得た目標Poseが変化した場合にpublishする。変化がなくても `resend_interval_sec` 経過で保険再送。

---

## 8. クラス構成

### 8.1 RouteFollowerNode
- `_on_route()` / `_on_pose()` / `_on_hint()` / `_on_manual_start()` / `_on_sig_recog()` : 各トピック受信時に Core へ更新を転送。
- `_on_timer()` : `tick()` 呼び出し、`_handle_target_publish()`、`_handle_state_publish()`、`_process_report_stuck_result()`、`_handle_stuck_report()` を順に実行。
- `_handle_state_publish()` : Coreの辞書から `FollowerState` を生成し、必要フィールドを埋めてpublish。
- `_handle_stuck_report()` : WAITING_REROUTE 時に `/report_stuck` サービスを非同期呼び出し。結果はロック付きで共有。

### 8.2 FollowerCore
- `update_route()` / `update_pose()` / `update_hint()` / `update_hint_stat()` / `update_control_inputs()` : mail boxへの書き込み。
- `tick()` : スナップショット取得と状態遷移のエントリポイント。
- `_start_avoidance_sequence()`、`_enter_waiting_reroute()`、`notify_reroute_failed()` 等で回避／再計画ロジックを実装。
- `_make_state_dict()` : `/follower_state` へ渡す辞書を生成。

---

## 9. QoS設計

| トピック | 信頼性 | 永続性 | 履歴 | 深度 |
|-----------|----------|----------|--------|------|
| `/active_route` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 |
| `/amcl_pose` | RELIABLE | VOLATILE | KEEP_LAST | 10 |
| `/obstacle_avoidance_hint` | BEST_EFFORT | VOLATILE | KEEP_LAST | 5 |
| `/active_target` | RELIABLE | VOLATILE | KEEP_LAST | 10 |
| `/follower_state` | RELIABLE | VOLATILE | KEEP_LAST | 10 |

---

## 10. エラー処理・ログ方針

- `/active_route` が空の場合はWARNを出して無視する。
- `/report_stuck` サービス未準備・呼び出し例外時はWARN/ERRORログを出し、結果は Core の `notify_reroute_failed()` でERROR状態へ移行。
- `control_rate_hz` や `resend_interval_sec` が異常値の場合は既定値へフォールバックする旨をWARN出力。

---

## 11. Phase3 拡張前提

- 回避失敗時の反対側リトライや `DECISION_RETRY` の取り扱いは今後の拡張項目。
- `FollowerState` に目標Poseや距離情報を追加する場合は `_make_state_dict()` と `_handle_state_publish()` の両方を拡張する。

---

## 12. テスト観点

| テスト種別 | 目的 | 検証項目 |
|-------------|------|----------|
| ユニットテスト | Coreのロジック確認 | 滞留検知、L字サブゴール生成、Hint統計処理 |
| コンポーネントテスト | ノード単体起動 | QoS・トピック変換、`tick()` 周期挙動 |
| 結合テスト | route_manager連携 | `/report_stuck` 呼び出しと再ルート復帰 |
| シナリオテスト | 実機検証 | 障害物回避挙動、STOP制御 |

---

## 13. まとめ

- Node／Core分離によりROS依存処理とロジックを明確に分担し、mail box + tick方式でスレッド安全性を確保した。
- `/report_stuck` は非同期で呼び出し、失敗時は Core が即座に `ERROR` 状態へ遷移する。
- L字回避は Hint中央値とWaypoint開放長を組み合わせて横シフト量を決定し、minorトライ回数で無限ループを防いでいる。

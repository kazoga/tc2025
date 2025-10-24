# route_manager_詳細設計書_phase2

## 0. 文書目的
本書は route_manager ノード（Phase2実装）の現行コードに整合する詳細設計を記述し、route_follower など他ノードの設計書と横並びで比較しても同一の章構成・粒度で理解できるよう整理したものである。ROS2依存層と非ROSロジック層の責務分担、入出力仕様、状態管理、パラメータおよびエラー処理方針を明示する。

---

## 1. ノード概要

| 項目 | 内容 |
|------|------|
| ノード名 | route_manager |
| 構成 | `RouteManagerNode`（ROS2 I/F） + `RouteManagerCore`（非ROSロジック） + `RouteManagerFSM`（非ROS状態機械） |
| 主機能 | 経路保持・配信、滞留報告処理、plannerサービス呼び出し、状態配信 |
| 実装言語 | Python3（Google Python Style + 型ヒント） |
| 対象ROS2ディストリビューション | Foxy |

`RouteManagerNode` はサービス／トピックI/FとQoS設定、`route_msgs` 型との変換、plannerサービス呼び出しの直列化を担当する。`RouteManagerCore` は経路モデルと `/report_stuck` の Update→SHIFT→SKIP→FAILED シーケンスを管理し、`RouteManagerFSM` が `GetRoute`／`UpdateRoute`／`ReportStuck` の非同期実行とタイムアウト制御を担う。

---

## 2. 責務とスコープ

### 2.1 責務
- `route_planner` から受領した `Route` を `RouteModel` に変換して保持し、`/active_route` をラッチ配信する。
- `/report_stuck` 要求を受理し、planner再計画→SHIFT→SKIP→FAILED の順で回避を試みてレスポンスを生成する。
- `RouteManagerFSM` と連携して `GetRoute`／`UpdateRoute`／`ReportStuck` の非同期タスクを逐次化し、タイムアウト時には `ERROR` 状態へ遷移する。
- `/manager_status` および `/route_state` を更新し、外部から追跡可能な状態・バージョン情報を提供する。
- `/mission_info` でシナリオ設定（start／goal／checkpoint）を起動直後に通知する。

### 2.2 スコープ外
- 障害物検知や `/block_detected` 購読による自律再計画（Phase3予定）。
- planner 以外の経路生成ソースや複数planner連携。

---

## 3. 入出力インタフェース

### 3.1 購読トピック
- なし（Phase2実装では外部トピックを購読しない）。

### 3.2 公開トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/active_route` | `route_msgs/Route` | 現在の経路をラッチ配信。再接続した follower も最新経路を取得可能。 |
| `/route_state` | `route_msgs/RouteState` | FSM状態と経路バージョン、現在インデックスを通知。 |
| `/mission_info` | `route_msgs/MissionInfo` | start／goal／checkpoints を起動直後に1度配信。 |
| `/manager_status` | `route_msgs/ManagerStatus` | state／decision／last_cause／route_version を配信。 |

### 3.3 サービス

| 名称 | 型 | 方向 | 説明 |
|------|----|------|------|
| `/report_stuck` | `route_msgs/srv/ReportStuck` | Server | follower滞留報告の受付。Coreが非同期シーケンスを実行しレスポンスを返す。 |
| `/get_route` | `route_msgs/srv/GetRoute` | Client | plannerへ初期経路取得を要求する。 |
| `/update_route` | `route_msgs/srv/UpdateRoute` | Client | plannerへ再計画を要求する。 |

---

## 4. パラメータ

| 名称 | 型 | 既定値 | 説明 |
|-------------|----|---------|------|
| `start_label` | string | "" | 初期 `GetRoute` 要求の開始ラベル。 |
| `goal_label` | string | "" | 初期 `GetRoute` 要求の終了ラベル。 |
| `checkpoint_labels` | string[] | [] | 中継ラベル一覧。 |
| `auto_request_on_startup` | bool | True | 起動直後に `GetRoute` を自動要求するか。 |
| `planner_timeout_sec` | double | 5.0 | plannerサービス待ちタイムアウト。 |
| `planner_retry_count` | int | 2 | planner呼び出し失敗時の再試行回数（現状は宣言のみ）。 |
| `planner_connect_timeout_sec` | double | 10.0 | `/get_route` 接続待ち最大時間。 |
| `state_publish_rate_hz` | double | 1.0 | 状態再送タイマー想定レート（タイマーは未実装）。 |
| `image_encoding_check` | bool | False | route_imageのエンコーディング検証フラグ。 |
| `report_stuck_timeout_sec` | double | 5.0 | `ReportStuck` 処理タイムアウト（FSMの `replan_timeout` に使用）。 |
| `offset_step_max_m` | double | 1.0 | SHIFT時に許容する最大横シフト量。 |

---

## 5. データ構造
- `RouteModel`: plannerから受領した `Route` をPythonオブジェクトへ展開し、major/minor version、waypoints、mission情報を保持する。
- `SimpleServiceResult(success: bool, reason: str, data: Optional[Any])`: Core内の各シーケンス結果を正規化し、Node層でレスポンスへ変換する。
- `_shift_preference`: waypointラベルごとに右(+1)/左(-1)シフト実績を記録して次回の優先方向を推定する辞書。
- `_skip_history`: waypointラベル単位でSKIP済みかを記録する集合。SKIPは1区間のみ許容する。
- `_planner_tasks`: FSMが生成した `asyncio.Task` の追跡用辞書。キャンセル時に整合を取る。
- `_status`: `ManagerStatus` publish用の状態辞書。`state`、`decision`、`last_cause`、`route_version` を保持する。

---

## 6. 状態遷移仕様

### 6.1 FSM状態
`RouteManagerFSM` が以下の状態を管理し、`handle_event()` でイベントを直列化する。各タスクには個別タイムアウトが設定される。

| 状態 | 概要 | 主なイベント | 遷移先 |
|------|------|--------------|--------|
| `IDLE` | 初期待機状態。 | `REQUEST_INITIAL_ROUTE` | `REQUESTING` |
| `REQUESTING` | `GetRoute` 実行中。 | 成功／失敗 | 成功→`ACTIVE` ／ 失敗→`ERROR` |
| `ACTIVE` | 通常運用。 | `REPORT_STUCK`／`UPDATE_ROUTE` | `WAITING_REROUTE`／`UPDATING` |
| `WAITING_REROUTE` | ReportStuck処理中。 | シーケンス成功／失敗 | 成功→`ACTIVE` ／ 失敗→`ERROR` |
| `UPDATING` | 明示的な `UpdateRoute` 処理中。 | 成功／失敗 | 成功→`ACTIVE` ／ 失敗→`ERROR` |
| `COMPLETED` | follower完走報告受領時。 | - | - |
| `ERROR` | 例外・タイムアウト発生時。 | `REQUEST_INITIAL_ROUTE` | `REQUESTING` |

### 6.2 公開状態の対応関係
- `RouteManagerCore._set_status()` が `/manager_status.state` に設定する値は `"running"`／`"holding"`／`"error"` など小文字文字列であり、FSM状態とは独立している。
- `/route_state` では FSM状態と `ManagerStatus` の双方を組み合わせ、列挙値へ正規化して publish する。

---

## 7. 主処理仕様

### 7.1 初期ルート取得
- `auto_request_on_startup` が真の場合、起動後に `REQUEST_INITIAL_ROUTE` イベントを投入する。
- `_planner_get_async()` が `route_msgs/srv/GetRoute` を呼び出し、成功時に `RouteModel.from_route()` で変換した結果を `_accept_route()` へ渡す。
- `_accept_route()` は内部状態を更新し、`/active_route`・`/route_state`・`/manager_status` を最新化する。

### 7.2 `/report_stuck` シーケンス
1. `_cb_replan()` が受信した `StuckReport` を `_sync_from_stuck_report()` で内部状態へ反映する。
2. `_try_update_route(reason="replan_first")` で planner の `/update_route` を呼び出す。成功時は minor version をリセットし `SimpleServiceResult(True, "replan_first")` を返す。
3. 失敗時は `_try_shift()` を実行。左右開放長と `_shift_preference` に基づいて横シフト量を計算し、成功時は minor version を +1、`offset_hint` を保持する。
4. SHIFTも失敗した場合 `_try_skip()` を実行。未スキップのWaypointのみ1区間スキップし、成功時は minor version を +1 する。
5. いずれも成功しない場合は `_set_status("holding", decision="failed", cause="avoidance_failed")` を呼び、`SimpleServiceResult(False, "avoidance_failed")` を返す。
6. Node層では `SimpleServiceResult` を `ReportStuck.Response` に変換し、成功時 `decision_code` を `REPLAN`／`SKIP` へ割り当て、`waiting_deadline` は現在200ms固定で返却する。

### 7.3 `/mission_info` 配信
- 初回 `RouteModel` 受理時に mission 情報を抽出し、`RouteFollower` との整合確保のためラッチ付きで1度配信する。

### 7.4 状態更新と再送
- `_publish_route_state()` が呼ばれるたびにノード側で `RouteState` メッセージへ変換し、`_normalize_route_state_status()` で `running`／`holding`／`error` などの文字列を列挙値へ正規化する。
- `state_publish_rate_hz` は周期再送タイマーの想定値として宣言されているが、現時点では明示的な再送は行っていない。

---

## 8. クラス構成

### 8.1 RouteManagerNode
- Publisher／Service サーバ・クライアントの初期化とQoS設定。
- `_on_report_stuck()` によるサービス応答生成と `SimpleServiceResult` からの変換。
- `run_async()` を介してCore/FSMの `asyncio` ループへ安全にタスクを投入する。

### 8.2 RouteManagerCore
- `RouteModel` の保持とローカルルート操作（SHIFT／SKIP）。
- `SimpleServiceResult` の生成、`ManagerStatus` の更新、および `/route_state` 更新トリガの発行。
- `_shift_preference`／`_skip_history`／`_last_replan_offset_hint` による学習的挙動の維持。

### 8.3 RouteManagerFSM
- `handle_event()` でイベントを直列化し、`asyncio` タスクを生成・監視する。
- `planner_timeout_sec`／`report_stuck_timeout_sec` を参照して各タスクへタイムアウトを設定し、失敗時は `ERROR` 状態へ遷移させる。

---

## 9. QoS設計

| トピック | 信頼性 | 永続性 | 履歴 | 深度 |
|-----------|----------|----------|--------|------|
| `/active_route` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 |
| `/route_state` | RELIABLE | VOLATILE | KEEP_LAST | 1 |
| `/mission_info` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 |
| `/manager_status` | RELIABLE | VOLATILE | KEEP_LAST | 1 |

plannerサービス呼び出しは `MutuallyExclusiveCallbackGroup` により直列実行とし、Node内の `asyncio` ループと競合しないよう `run_async()` で橋渡しする。

---

## 10. エラー処理・ログ方針

| 事象 | 挙動 | 状態遷移 |
|------|------|----------|
| `GetRoute`/`UpdateRoute` コールバック未設定 | `SimpleServiceResult(False, ...)` を返し FSM が `ERROR` へ遷移。 | `REQUESTING/UPDATING → ERROR` |
| planner サービス例外・失敗 | Node層が `success=False` を返し FSM が `ERROR` へ遷移。 | `REQUESTING/WAITING_REROUTE/UPDATING → ERROR` |
| SHIFT/SKIP 不成立 | `_set_status("holding", ...)` を実行し、レスポンスは `FAILED`。FSMは `ERROR` へ遷移。 | `WAITING_REROUTE → ERROR` |
| ルート検証失敗（frame_id 不一致 等） | `SimpleServiceResult(False, "invalid route")` を返却。 | `REQUESTING/WAITING_REROUTE → ERROR` |

ログは起動・主要状態遷移・サービス失敗時に `info`／`warn`／`error` レベルで出力し、滞留シーケンス結果（SHIFT成功など）も `info` で通知する。

---

## 11. Phase3 拡張想定
- `/block_detected` の購読と自律再計画イベント投入。
- planner への `offset_hint` 連携強化や複数候補経路の選択。
- `state_publish_rate_hz` に基づく周期再送タイマーの本実装。

---

## 12. テスト観点

| テスト種別 | 目的 | 検証項目 |
|-------------|------|----------|
| ユニットテスト | Coreロジックの検証 | SHIFT／SKIP 条件分岐、`SimpleServiceResult` の内容、`RouteModel` minor version 更新 |
| コンポーネントテスト | ノード単体起動 | QoS 設定、`/report_stuck` 応答の decision_code、FSM タイムアウト動作 |
| 結合テスト | planner／follower 連携 | `/get_route` 初期化→follower 実行→滞留報告のハッピーパスと失敗系 |
| シナリオテスト | 実機走行 | SHIFT／SKIP の適用可否、`/mission_info` の周知、`ManagerStatus` の監視 |

---

## 13. まとめ
- RouteManagerNode／Core／FSM の3層分離により、ROS I/F と非同期ロジックを安全に連携させている。
- `/report_stuck` シーケンスは planner再計画→SHIFT→SKIP→FAILED の優先度で実装され、各結果を `ManagerStatus` とレスポンスへ反映する。
- QoS・状態配信・エラー処理の方針を route_follower と共通の章立てで整理し、ノード間で一貫した設計書構造を維持する。

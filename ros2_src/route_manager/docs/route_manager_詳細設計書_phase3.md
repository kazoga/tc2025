# route_manager_詳細設計書_phase3.md

## 第1章　概要

`route_manager` は、全体経路の管理および再計画を担うROS2ノードである。  
本ノードは `route_planner` と `route_follower` の間に位置し、経路データの生成・更新・配信を行う。  
Phase3では、滞留検知に加えて**経路封鎖（road_blocked）シナリオへの対応強化**を行い、
固定ブロック区間での継続走行判定および可変区間でのリルート専念ロジックを導入した。

---

## 第2章　責務とスコープ

### 責務
- 経路管理：`route_planner` から生成された経路を保持し、`route_follower` に配信。
- 再計画制御：滞留通報（`/report_stuck`）に応じて再計画／スキップ／HOLDを判断。
- 状態管理：ノード自身の運用状態を `/manager_status` トピックで公開。
- 経路更新配信：新しい `/active_route` を TRANSIENT_LOCAL QoS で再送信。
- GUI・他ノードとの整合を保ち、シナリオ全体の状態遷移を統括。

### スコープ外
- 障害物検知（obstacle_monitor）やローカル制御（follower）は対象外。

---

## 第3章　状態遷移設計

Phase3でもPhase2で整備した4状態構成を踏襲しつつ、経路封鎖時の判断分岐をRUNNING→UPDATING_ROUTE内で強化した。

| 状態名 | 概要 | 主なイベント | 次状態 |
|---------|------|----------------|---------|
| **IDLE** | 初期または全停止状態。経路未設定。 | `/get_route`完了、初回ルート受信 | RUNNING |
| **RUNNING** | 通常運用中。経路追従ノード稼働。 | `/report_stuck`受信 | UPDATING_ROUTE |
| **UPDATING_ROUTE** | 再計画またはスキップ中。 | `/active_route`配信完了 | RUNNING |
| **HOLDING** | planner失敗・通信エラー・封鎖などによる停止状態。 | operator再開／新ルート受信 | RUNNINGまたはIDLE |

### 状態遷移図（テキスト）

```
[初期状態]
    ↓
┌────────────┐
│   IDLE     │
└────────────┘
    │
    │ シナリオ開始／初回ルート受信
    ↓
┌────────────┐
│  RUNNING   │
└────────────┘
    │
    │ followerから report_stuck 受信
    ↓
┌──────────────────┐
│  UPDATING_ROUTE  │
└──────────────────┘
    │
    ├─ 成功／skip完了 → RUNNING
    └─ timeout／失敗 → HOLDING
              │
              ├─ operator再開 → RUNNING
              └─ mission終了 → IDLE
```

---

## 第4章　ノード構成概要

| 要素 | 役割 |
|------|------|
| `/active_route` (Publisher) | followerへ経路を配信。QoS=RELIABLE, TRANSIENT_LOCAL |
| `/manager_status` (Publisher) | manager自身の状態を通知 |
| `/report_stuck` (ServiceServer) | followerからのスタック通報を受け、方針を決定して応答 |
| `/update_route` (ServiceClient) | route_plannerへの再計画要求 |
| `/get_route` (ServiceClient) | 初期ルート要求 |

> Phase3では経路封鎖イベントを専用トピックではなく `/report_stuck` の `reason_code=ROAD_BLOCKED` として受信し、
> `Waypoint.segment_is_fixed` と組み合わせて判断する。

---

## 第5章　機能仕様

### 5.1 通常運用（RUNNING）
- `/active_route` にて現在有効な経路をfollowerへ配信。
- `/report_stuck` 要求が到着するまで通常運用を継続。
- `/manager_status.state="running"` を周期1Hzで発行。

### 5.2 滞留通報処理（/report_stuck）
followerから滞留検知を受けた場合、`ReportStuck.Request.reason_code` と `Waypoint.segment_is_fixed` を併用し、
以下の優先順位で対処する。

#### 先行判定：経路封鎖（road_blocked）
- `reason_code == ROAD_BLOCKED` の場合、まず現在ターゲットの `segment_is_fixed` を参照する。
- **固定ブロック（segment_is_fixed=True）**：経路を改変せず minor version のみをインクリメントし、
  現行ルートを `active_route` として再配信する。`decision_code=REPLAN` として follower へ継続走行を指示する。
- **可変ブロック（segment_is_fixed=False）**：plannerへの `UpdateRoute` のみを試行する。
  `UpdateRoute` が失敗した場合は SHIFT/SKIP を行わず `decision_code=FAILED` を返し、ノード状態を HOLDING へ遷移させる。

#### 第1層：planner再計画
- 先行判定で処理されなかった場合、`UpdateRoute` を呼び出して外部plannerへ再計画を要求する。
- 成功時は minor version をリセットし、新ルートを `active_route` として配信する。

#### 第2層：左右オフセット判定
- `reason_code in (FRONT_BLOCKED, AVOIDANCE_FAILED)` のときのみ実施し、
  余白量と履歴に応じて `offset_hint` を算出する。
- オフセット適用後は minor version を+1したローカルルートを生成し、`decision_code=REPLAN` で応答する。

#### 第3層：スキップ判定
- 直近WPが `not_skip=False` かつスキップ履歴が閾値以下の場合に実行する。
- 実施時は次WPを現在ターゲットとしてローカルルートを再生成し、`decision_code=SKIP` とする。

#### 第4層：失敗判定
- 上記のいずれも成功しない場合は `decision_code=FAILED` を返却し、HOLDING 状態に移行する。
- 失敗理由として最後に受け取った `reason_label` を `ManagerStatus.last_cause` に保存し、GUIへ通知する。

### 5.3 HOLDING状態
- planner連続失敗や経路喪失時に遷移。
- `/manager_status.state="holding"` を発行。
- GUIまたはoperator指令により `/get_route` を再要求可能。

---

## 第6章　入出力インタフェース

### トピック一覧

| 名称 | 型 | QoS | 方向 | 説明 |
|------|----|-----|------|------|
| `/active_route` | `route_msgs/Route` | RELIABLE / TRANSIENT_LOCAL | Pub | 経路配信 |
| `/manager_status` | `route_msgs/ManagerStatus` | RELIABLE / VOLATILE | Pub | ノード状態通知 |

> Phase3では経路封鎖イベントを専用トピックではなく `/report_stuck` の `reason_code=ROAD_BLOCKED` として受信し、
> `Waypoint.segment_is_fixed` と組み合わせて判断する。

### サービス一覧

| 名称 | 型 | 方向 | 説明 |
|------|----|------|------|
| `/report_stuck` | `route_msgs/srv/ReportStuck` | Server | follower滞留通報受付 |
| `/update_route` | `route_msgs/srv/UpdateRoute` | Client | 再計画要求 |
| `/get_route` | `route_msgs/srv/GetRoute` | Client | 初期経路取得 |

---

## 第7章　メッセージ仕様

### ReportStuck.srv (再掲)

**Request**
```
int32 route_version
int32 current_index
string current_wp_label
geometry_msgs/Pose current_pose_map
uint8 REASON_UNKNOWN=0
uint8 REASON_FRONT_BLOCKED=1
uint8 REASON_ROAD_BLOCKED=2
uint8 REASON_NO_HINT=3
uint8 REASON_NO_SPACE=4
uint8 REASON_AVOIDANCE_FAILED=5
uint8 reason_code
string reason_detail
uint32 avoid_trial_count
bool last_hint_blocked
float32 last_applied_offset_m
```

**Response**
```
uint8 DECISION_NONE=0
uint8 DECISION_REPLAN=1
uint8 DECISION_SKIP=2
uint8 DECISION_FAILED=3

uint8 decision_code
builtin_interfaces/Duration waiting_deadline
float32 offset_hint
string note
```

### ManagerStatus.msg
```
std_msgs/Header header
string state        # "idle","running","updating_route","holding"
string decision     # "none","update","replan_first","shift","skip","failed"
string last_cause   # HOLDING時のみ滞留理由を保持（"front_blocked","no_hint","no_space"など）
uint32 route_version
```

- `RouteState.message` には直近で適用したイベント（`route_ready`、`update_route`、`replan_first`、`shift_right(0.5m)` など）を格納し、GUI 側で「Ev:」表記として表示する。
- `ManagerStatus.decision` は `update` / `replan_first` / `shift` / `skip` / `failed` を採用し、`state="holding"` のときのみ `last_cause` に滞留理由を残す。RUNNING へ復帰した時点で `last_cause` は空文字へ戻す。

---

## 第8章　パラメータ

| パラメータ | 型 | 既定値 | 説明 |
|-------------|----|---------|------|
| `planner_timeout_sec` | float | 5.0 | route_planner応答待機時間 |
| `waiting_deadline_sec` | float | 8.0 | followerのWAITING最大待機時間 |
| `skip_threshold_m` | float | 0.8 | スキップ距離閾値 |
| `avoid_max_retry` | int | 3 | follower側局所回避上限回数 |
| `offset_step_max_m` | float | 1.0 | 再計画指示時に許容する横ずれ最大値[m] |

---

## 第9章　エラー処理・リカバリ

| 事象 | 処理 | 状態遷移 |
|------|------|----------|
| route_planner応答timeout | HOLDINGへ移行 | UPDATING_ROUTE→HOLDING |
| 再計画失敗 | HOLDINGへ移行しGUI通知 | UPDATING_ROUTE→HOLDING |
| follower未応答 | 無処理（再通報待ち） | RUNNING維持 |
| 通信断復旧後 | 自動で再接続 | 状態維持 |

---

## 第10章　Phase3拡張想定

Phase3では、動的経路封鎖（`road_block`）検知に基づき、
`RUNNING→UPDATING_ROUTE` への自律遷移を許可する。  
`offset_hint` パラメータをroute_plannerに転送し、
左右オフセット（最大 `offset_step_max_m`）を考慮した回避ルート生成を実現予定。

---

## 第11章　設計上の留意事項

- `/active_route` のQoSは **TRANSIENT_LOCAL** とし、follower再起動時にも受信可能にする。
- `/manager_status` はVOLATILEで構わないが、周期送信によりGUIが最新状態を把握。
- 状態は4種類に固定し、内部フラグで細分化しても外部公開しない。
- `/report_stuck` 応答は同期完了後、即座にdecision_codeを返す。blocking時間は200ms以下を目標。
- route_plannerとの通信は非同期futureを用い、timeout管理を徹底。

---

## 第12章　まとめ

- Phase3では、road_blocked通報時の判定フローを強化し、固定ブロック区間での継続走行と可変区間でのリルート専念を両立させた。
- 従来の3層判断（planner→SHIFT→SKIP）は `reason_code` に応じて分岐し、road_blocked では SHIFT/SKIP を抑制する。
- managerの状態は `IDLE / RUNNING / UPDATING_ROUTE / HOLDING` の4つを維持しつつ、`ManagerStatus.last_cause` に封鎖理由を通知する。

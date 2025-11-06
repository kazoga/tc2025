# route_manager_phase3_検討記録.md
（Phase3検討記録／経路封鎖・リルート対応）

## 1. 目的

`route_manager` は、経路追従ノード (`route_follower`) から滞留検知（スタック）通報を受け取り、
通報内容・経路属性・障害情報をもとに、
**再計画（replan）／スキップ（skip）／保留（hold）** のいずれかの方針を即時決定し、
必要に応じて `route_planner` に再計画を指示する。  

Phase3では、road_blocked 通報時の判断を中心に再整理し、固定ブロック継続か可変区間でのリルート専念かを即時判定できるようにする。

---

## 2. 状態定義（4状態モデル）

route_managerは、外部から観測可能な以下の4状態を持つ。  
内部同期処理（DECIDING, SKIPPING等）は明示せず、安定状態のみを定義する。

| 状態名 | 概要 | 主なトリガ | 次状態 | 備考 |
|---------|------|-------------|---------|------|
| **IDLE** | シナリオ開始前または全停止状態。 | `/get_route`完了／初回ルート受信 | RUNNING | mission終了時にも戻る。 |
| **RUNNING** | followerが通常追従中。ルート有効。 | followerから`/report_stuck`受信 | UPDATING_ROUTE | followerの `RUNNING→REPORTING_STUCK→WAITING_REROUTE` に対応。 |
| **UPDATING_ROUTE** | 再計画またはスキップ処理中。 | `/active_route`配信完了 | RUNNING | replan／skip共通の経路更新状態。timeout発生時はHOLDING。 |
| **HOLDING** | planner失敗・通信異常・経路喪失時など。オペレータ介入待機。 | operator再開／新ルート適用 | RUNNING or IDLE | followerの `HOLD` に対応。 |

---

## 3. 状態遷移図（テキスト表記）

```
[初期状態]
    ↓
┌────────────┐
│   IDLE     │  シナリオ未開始／全停止
└────────────┘
    │
    │ シナリオ開始または初回ルート受信
    ↓
┌────────────┐
│  RUNNING   │  通常運用（follower追従）
└────────────┘
    │
    │ followerから /report_stuck サービス受信
    ↓
┌──────────────────┐
│  UPDATING_ROUTE  │  再計画またはスキップ処理中
└──────────────────┘
    │
    ├─ planner成功 or skip完了 → /active_route配信 → RUNNING
    │
    └─ timeout / 失敗 / route消失 → HOLDING
              │
              ├─ operator再開／新ルート取得 → RUNNING
              │
              └─ mission終了 → IDLE
```

---

## 4. followerとの状態対応

| follower状態 | manager状態 | 説明 |
|----------------|----------------|------|
| IDLE | IDLE | シナリオ開始前または終了後。 |
| RUNNING | RUNNING | 通常経路追従中。 |
| WAITING_REROUTE | UPDATING_ROUTE | 新ルート更新待ち（replanまたはskip）。 |
| HOLD | HOLDING | planner失敗・経路喪失・人手介入待ち。 |

→ 状態数および意味が完全に1対1対応し、ログ解析やGUI同期が容易。

---

## 5. サービスインタフェース設計  
### `/report_stuck` サービス仕様（同期応答）

**Request**

| フィールド | 型 | 意味 |
|-------------|----|------|
| `reason_code` | uint8 | followerが検知した滞留理由コード。0=unknown,1=front_blocked,2=road_blocked 等 |
| `reason_detail` | string | コードに収まらない補足理由。空文字の場合は code を採用 |
| `current_wp_label` | string | 現在のwaypointラベル |
| `avoid_trial_count` | uint32 | followerが行った局所回避試行回数 |
| `last_hint_blocked` | bool | 最後に受信したobstacle_hintが閉塞を示しているか |
| `last_applied_offset_m` | float32 | followerが直前に適用した横方向オフセット値[m] |

**Response**

| フィールド | 型 | 意味 |
|-------------|----|------|
| `decision_code` | uint8 | 1=REPLAN, 2=SKIP, 3=FAILED(HOLD) |
| `waiting_deadline` | Duration | followerがWAITING_REROUTEで待機する上限時間 |
| `offset_hint` | float32 | ±値で左右オフセット提案（m）。0ならオフセットなし |
| `note` | string | ログ／GUI向け説明メッセージ |

---

## 6. 意思決定ロジック（3層判断モデル）

`route_manager` は、滞留報告を受けた際に以下の3層手順で判断を行う。

### 先行判定：road_blocked（経路封鎖）

**目的**
道路封鎖検知時に即座に固定／可変区間を区別し、再開条件を決定する。

**条件と処理**
- `ReportStuck.reason_code == ROAD_BLOCKED` を受信した場合に発火。
- 現在の waypoint が `segment_is_fixed=True` なら minor version のみを更新した `active_route` を再配信し、
  followerへ継続走行を指示する。
- `segment_is_fixed=False` なら SHIFT/SKIP を実施せず `UpdateRoute` のみを試行し、失敗時は `decision_code=FAILED` とする。

**備考**
road_blocked時は `/road_blocked` トピックの解除待ちではなく、新しい `active_route` を受信させることを再開条件とする。

### 第1層：左右オフセット判定（局所回避余地の評価）

**目的**  
経路上にまだ左右いずれかの空間余裕があるかを確認し、再計画を指示する。

**判定条件**  
- `reason` ∈ {"front_blocked","avoidance_failed"}
- `avoid_trial_count < avoid_max_retry`  
- `last_hint_blocked == True`  

**判定結果**
```
if has_left_space and last_applied_offset_m <= 0 → offset_hint = -min(left_open, offset_step_max_m)
elif has_right_space and last_applied_offset_m >= 0 → offset_hint = +min(right_open, offset_step_max_m)
else → 第2層へ
```

結果は `/update_route` へパラメータとして渡され、
plannerは「左右寄せルート」再生成を実施する（Phase3対応想定）。

---

### 第2層：スキップ判定（短区間通過不能時の飛ばし）

**目的**  
通行困難な短区間を安全にスキップし、後続waypointへ接続できるか判断する。

**条件**
- `reason` ∈ {"no_hint","no_space","avoidance_failed"}  
- 現在WPが`skippable=True`  
- 次WPとの距離が`dist < skip_threshold_m`  
- 経路曲率が急変（90°超）でない

**判定結果**
- OK → `decision_code=SKIP`、新ルートをスライスして `/active_route` 配信  
- NG → 第3層へ

---

### 第3層：再計画／失敗判定

**再計画（replan）**  
- 第1・2層で該当なし  
- planner通信可能であれば `/update_route` 呼び出し  
- 成功→新ルート配信、失敗→FAILED(HOLD)

**失敗（hold）**  
- planner timeout／連続失敗／`avoid_trial_count`上限到達  
- `/manager_status.state="holding"` を発行しGUI通知

---

## 7. 意思決定全体フロー（簡易図）

```
[report_stuck受信]
       │
       ↓
(0) reason_code == ROAD_BLOCKED ?
       ├─ yes → segment_is_fixed ?
       │             ├─ True  → minor+1で再配信 (REPLAN)
       │             └─ False → UpdateRouteのみ試行 → 失敗時FAILED
       └─ no
            │
            ↓
(1) オフセット余地あり？
       ├─ yes → offset_hint付きで再計画指示
       └─ no
            │
            ↓
(2) スキップ可？
       ├─ yes → active_routeスライス配信 (SKIP)
       └─ no
            │
            ↓
(3) planner再計画実施
       ├─ 成功 → active_route配信 (REPLAN)
       └─ 失敗 → HOLDING
```

---

## 8. manager状態遷移と判断結果の対応

| 状態 | 発生イベント | decision_code | 出力動作 |
|-------|---------------|-----------|-----------|
| RUNNING | followerから通報受信 | offset／skip／replan／failed | /report_stuck 応答送信 |
| UPDATING_ROUTE | 再計画またはスキップ実行中 | replan or skip | /active_route配信 |
| HOLDING | 再計画失敗・timeout | failed | /manager_status.state="holding" |
| IDLE | mission未開始または終了 | - | 状態維持 |

---

## 9. Phase3（経路封鎖・動的リルート）への拡張分析

| 項目 | 対応方法 | 状態追加の要否 |
|------|-----------|----------------|
| 経路封鎖検出 | `/report_stuck` の `reason_code=ROAD_BLOCKED` と `Waypoint.segment_is_fixed` を参照 | 不要 |
| 封鎖解消再開 | 新しい `active_route` を受信したら navigator の停止を解除 | HOLDING内処理で対応 |
| オフセットルート生成 | 第1層の offset_hint を route_planner へ転送（road_blocked時は実施しない） | 不要 |
| 複数候補ルート | UPDATING_ROUTE内で比較／選択 | 不要 |

---

## 10. manager内部実装の擬似コード例

```python
def handle_report_stuck(req):
    wp = self.route_table.find(req.current_wp_label)

    # Layer1: offset judgment
    if (
        req.reason_code in [
            ReportStuck.Request.REASON_FRONT_BLOCKED,
            ReportStuck.Request.REASON_AVOIDANCE_FAILED,
        ]
        and req.avoid_trial_count < self.param.avoid_max_retry
        and req.last_hint_blocked
    ):
        if wp.has_left_space and req.last_applied_offset_m <= 0:
            return Decision(REPLAN, offset_hint=-min(wp.left_open, self.param.offset_step_max_m))
        elif wp.has_right_space and req.last_applied_offset_m >= 0:
            return Decision(REPLAN, offset_hint=+min(wp.right_open, self.param.offset_step_max_m))

    # Layer2: skip judgment
    if wp.skippable and wp.dist_to_next < self.param.skip_threshold_m:
        self.slice_active_route(wp.label)
        self.publish_active_route()
        return Decision(SKIP)

    # Layer3: replan / hold
    if self.planner_available():
        success = self.call_update_route()
        return Decision(REPLAN if success else FAILED)
    else:
        return Decision(FAILED)
```

---

## 11. 妥当性と設計方針まとめ

| 観点 | 評価 | 理由 |
|------|------|------|
| **状態最小化** | ◎ | 4状態（IDLE / RUNNING / UPDATING_ROUTE / HOLDING）で十分。 |
| **followerとの整合** | ◎ | 1:1完全対応。 |
| **判断ロジックの明確性** | ◎ | 3層構造により可読・拡張容易。 |
| **Phase3拡張性** | ◎ | 経路封鎖リルートもUPDATING_ROUTEで吸収。 |
| **安全性・運用性** | ○ | HOLDINGで確実に停止・通知。 |

---

## 12. 結論

- **外部状態は4つに限定**：`IDLE / RUNNING / UPDATING_ROUTE / HOLDING`
- **road_blocked先行判定**を導入し、固定区間ではminor更新による継続、可変区間ではplanner再計画専念とした。
- **判断ロジックは (road_blocked→) オフセット → スキップ → 再計画／失敗** の順で整理し、SHIFT/SKIPの適用条件を `reason_code` によって制御する。
- followerとの同期、GUI表示、運用制御がすべて一貫する。

---

本ドキュメントは、Phase3における `route_manager` の行動設計・判断仕様の基礎を定めるものであり、
詳細設計書（`route_manager_詳細設計書_phase3.md`）の第3章・第5章に直接反映している。

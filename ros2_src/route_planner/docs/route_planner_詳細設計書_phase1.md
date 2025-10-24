# route_planner_詳細設計書_phase1

## 0. 文書目的
本書は route_planner パッケージの現行実装（`route_planner/route_planner.py` および `graph_solver.py`）に整合する詳細設計を示し、route_manager や route_follower の設計書と横並びで読める章構成で整理する。GetRoute／UpdateRoute サービスサーバとしての責務、データ構造、再計画アルゴリズム、QoS・エラー処理方針を明示し、今後のフェーズ拡張に備えた共通理解を提供する。

---

## 1. ノード概要

| 項目 | 内容 |
|------|------|
| ノード名 | route_planner |
| 構成 | `RoutePlannerNode`（単一ノード。内部関数と dataclass で処理を分割） |
| 主機能 | YAML＋CSV からルートを構築して `/get_route` に応答、封鎖情報を受けた再探索で `/update_route` に応答 |
| 実装言語 | Python3（Google Python Style + 型ヒント） |
| 対象ROS2ディストリビューション | Foxy |

`RoutePlannerNode` は起動時に YAML 定義と CSV セグメントを読み込みキャッシュする。`/get_route` 呼び出しで固定／可変ブロックを連結して初期 Route を生成し、`/update_route` 呼び出しでは現在走行中ブロックのグラフから封鎖エッジを除外して再探索し、仮想エッジを補って連結した結果を返す。

---

## 2. 責務とスコープ

### 2.1 責務
- YAML で定義されたブロック列に従って Waypoint シーケンスを連結し、Route メッセージを生成する。
- 可変ブロックに対して `graph_solver.solve_variable_route()` を呼び出し、封鎖情報・チェックポイントを考慮した最短経路を算出する。
- 現行 Route・由来メタデータ（`WaypointOrigin`）・封鎖エッジ集合・訪問チェックポイント履歴を保持し、UpdateRoute に利用する。
- PNG 形式のルート画像を読み込み `sensor_msgs/Image(bgr8)` として Route に添付し、取得できない場合はプレースホルダ画像を用意する。
- サービス呼び出し間で直列性を確保し、入力異常時は失敗レスポンスとログで通知する。

### 2.2 スコープ外
- サービス以外のトピック通信（現フェーズでは publisher/subscriber を持たない）。
- planner 側でのルート最適化アルゴリズムの詳細実装。
- 封鎖イベントの自動検出や複数可変ブロックの並列探索管理。

---

## 3. 入出力インタフェース

### 3.1 購読トピック
- なし（サービス要求のみを受け付ける）。

### 3.2 公開トピック
- なし（Route はサービスレスポンスで返却）。

### 3.3 サービス

| 名称 | 型 | 方向 | 説明 |
|------|----|------|------|
| `/get_route` | `route_msgs/srv/GetRoute` | Server | start／goal／checkpoint 指定に従い固定・可変ブロックを連結した Route を生成して返す。 |
| `/update_route` | `route_msgs/srv/UpdateRoute` | Server | 現行 Route と封鎖エッジ情報を基に可変ブロックを再探索し、新たな Route を返す。固定ブロック封鎖時は失敗を返す。 |

---

## 4. パラメータ

| 名称 | 型 | 既定値 | 説明 |
|-------------|----|---------|------|
| `config_yaml_path` | string | `routes/config.yaml` | ブロック構成 YAML のパス。パッケージ共有ディレクトリを起点に解決。 |
| `csv_base_dir` | string | `routes` | セグメント CSV の基準ディレクトリ。空文字なら YAML 所在ディレクトリを使用。 |

---

## 5. データ構造
- `SegmentCacheEntry(segment_id: str, waypoints: List[Waypoint])`: CSV から読み込んだ waypoint 配列のキャッシュ。逆方向連結時も利用する。
- `WaypointOrigin`: 各 waypoint が所属するブロック名／エッジ情報／ローカルインデックス／向きを保持し、UpdateRoute 時に prev/next の検証と仮想エッジ生成へ利用する。
- `_blocks`: YAML を正規化した辞書配列。`type=fixed|variable`、`segment_id`、`nodes`／`edges` 等を含む。
- `_closed_edges: Set[frozenset[str]]`: 現在封鎖済みの可変エッジ集合。同一ブロック内で累積する。
- `_visited_checkpoints_hist: Dict[str, Set[str]]`: ブロック毎の訪問済みチェックポイント履歴。複数回の UpdateRoute を跨いで保持する。
- `_current_route: Route` と `_current_route_origins: List[WaypointOrigin]`: 最新 Route と各 waypoint の由来メタ。UpdateRoute の整合性検証に使用。

---

## 6. 状態遷移仕様

本ノードは明示的な FSM を持たないが、以下の状態フラグで動作を管理する。

| 状態 | 概要 | 遷移契機 |
|------|------|----------|
| `UNINITIALIZED` | `_current_route` 未設定状態。起動直後または GetRoute 失敗時。 | 起動、GetRoute 失敗 |
| `INITIALIZED` | `_current_route` を保持し、UpdateRoute が有効。 | GetRoute 成功 |

`UpdateRoute` リクエストでは `INITIALIZED` 状態を前提とし、`route_version` 不一致や prev/next 不整合時は失敗レスポンスを返して `INITIALIZED` 状態を保持する。

---

## 7. 主処理仕様

### 7.1 起動時初期化
- `config_yaml_path`／`csv_base_dir` を解決しログに出力する。取得できない場合は警告を出しつつ空文字扱いで継続する。
- `_load_blocks_from_yaml()` で YAML を読み込み、`fixed`／`variable` ブロックを正規化して `_blocks` に格納する。`csv_base_dir` が未指定なら YAML 所在ディレクトリを設定する。
- `_load_csv_segments()` で `_blocks` から参照される CSV を走査し、`SegmentCacheEntry` としてキャッシュする。可変ブロックの nodes／edges ファイルもここで読み込む。

### 7.2 `/get_route`
1. リクエストの `start_label`／`goal_label`／`checkpoint_labels` を取得しログに出力する。
2. `_blocks` を先頭から走査し、`fixed` ブロックはキャッシュ済みセグメントを連結、`variable` ブロックは `solve_variable_route()` を呼び出して可変エッジ列を得る。
3. 可変エッジを連結する際に向き情報を保持し、`WaypointOrigin` を生成して由来を記録、逆走区間は姿勢再計算対象として控える。
4. リクエスト start／goal でスライスしインデックスを再採番、距離を再計算する。逆走区間とブロック末尾に対して `adjust_orientations()` で姿勢を補正する。
5. 可変ブロックが存在し画像が取得できた場合は solver 生成画像を採用、失敗時はプレースホルダを添付する。
6. `route_version` を 1 に設定し、Route メッセージと内部状態を更新して成功レスポンスを返す。例外時はエラーログと失敗レスポンスを返す。

### 7.3 `/update_route`
1. 現行 Route の存在と `route_version` 一致を検証する。不一致時は現行 Route を添えて失敗応答。
2. `prev_index`／`next_index` が隣接し、ラベルが一致するか `WaypointOrigin` メタで検証する。固定ブロックの場合は警告を出して失敗応答。
3. 同一ブロック内で封鎖エッジ集合を更新し、`_build_graph_with_closures()` で封鎖除外済みグラフを構築する。訪問済みチェックポイント履歴を更新して未訪問集合を求める。
4. 現在位置から封鎖点までの仮想エッジ waypoint 群を `_make_virtual_edge_waypoints()` で生成し、`WaypointOrigin` に `__virtual__` として記録する。
5. solver へ封鎖エッジを除外したグラフと未訪問チェックポイントを渡して可変エッジ列を取得する。固定ブロック後続も含めて連結し、`concat_with_dedup()` と `stamp_edge_end_labels()` で整形する。
6. 連結後に距離・姿勢を補正し、`route_version` を `current_route.version + 1` に更新する。新 Route を内部状態とレスポンスに反映する。
7. 例外が発生した場合は詳細をログに出し、現行 Route を添えて失敗レスポンスを返す。

### 7.4 画像処理ユーティリティ
- `_load_png_as_image()` で PNG を読み込み `bgr8` Image を生成する。OpenCV エラー時は `FileNotFoundError` を送出。
- `make_text_png_image()` は画像が無い場合の 1x1 プレースホルダを生成する。

---

## 8. クラス構成

### 8.1 RoutePlannerNode
- Node 初期化、パラメータ宣言、YAML／CSV ロード、サービス登録、内部状態の保持を担当する。
- `_resolve_service_name()` でリマップ後名称を取得しログに利用する。

### 8.2 補助 dataclass・関数
- `SegmentCacheEntry`、`WaypointOrigin` が Route の由来管理を担う。
- `parse_waypoint_csv()`、`concat_with_dedup()`、`adjust_orientations()` などのユーティリティはノード外に定義し、テストや再利用を容易にしている。
- `graph_solver.solve_variable_route()` は別モジュールに実装され、ノードは結果の整形に集中する。

---

## 9. QoS設計
- サービスは ROS2 既定 QoS（リクエスト／レスポンス型）を使用する。並列呼び出しを避けるため、`MutuallyExclusiveCallbackGroup` を適用し逐次処理とする。
- トピック通信を行わないため QoS 設定は不要。

---

## 10. エラー処理・ログ方針
- 起動時に YAML／CSV の読み込み失敗を `self.get_logger().error()` で出力しつつ例外を捕捉し、サービス実行時に失敗レスポンスへ反映する。
- `handle_get_route()`／`handle_update_route()` では try-except で例外を捕捉し、トレースを含めてエラーログへ出力する。
- 入力検証失敗（ラベル不一致、prev/next 不整合など）は `RuntimeError` を送出して失敗レスポンスに理由文字列を格納する。

---

## 11. Phase3 拡張想定
- ブロック単位でのタイムアウト制御や優先度付き再探索、複数 planner 連携による候補経路提示。
- サービス以外の状態配信（現在の封鎖情報やチェックポイント進捗のトピック化）。
- Route 画像生成のカスタマイズ（サイズ／配色／凡例）とキャッシュ。

---

## 12. テスト観点

| テスト種別 | 目的 | 検証項目 |
|-------------|------|----------|
| ユニットテスト | ユーティリティ関数検証 | `parse_waypoint_csv()` の列解釈、`concat_with_dedup()` の重複排除、仮想エッジ生成のラベル刻印 |
| コンポーネントテスト | サービス呼び出し検証 | `/get_route` 正常応答、固定ブロック欠落時のエラー、`/update_route` の封鎖累積挙動 |
| 結合テスト | manager/follower 連携 | Route 生成→配信→封鎖→再探索のハッピーパスと失敗系、画像添付の有無 |
| シナリオテスト | 実機走行 | 可変ブロックでの封鎖連続発生時の挙動、チェックポイント再訪問時の履歴管理 |

---

## 13. まとめ
- `RoutePlannerNode` は YAML／CSV をキャッシュして即応性を確保しつつ、封鎖情報を蓄積して再探索するサービスサーバである。
- dataclass で waypoint の由来情報を保持し、UpdateRoute 時の検証・仮想エッジ生成・画像添付まで一貫して扱う。
- route_manager／route_follower と同一章構成で設計情報を整理し、パッケージ横断で整合の取れたドキュメント体系を維持する。

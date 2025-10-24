# 日本語文書

## フェーズ1（情報分析）検討記録

### 1. 目的
運用時に頻繁なタブ切り替えが難しい条件を踏まえ、robot_console のUIはダッシュボードを常時表示しながら、主要ノードの状態と操作を即時に行える構成が求められる。本検討では既存ノードが発行・購読する情報を分析し、ダッシュボード中心の情報提示・操作方針を整理する。

### 2. 対象ノードと主要インタフェース
- `route_manager`：`active_route`、`route_state`、`mission_info`、`manager_status`を配信し、`report_stuck`サービスを受け付ける。
- `route_follower`：`active_route`、`amcl_pose`、`obstacle_avoidance_hint`、`manual_start`、`sig_recog`を購読し、`active_target`と`follower_state`を配信する。
- `obstacle_monitor`：`/scan`・`amcl_pose`・`active_target`を購読し、`obstacle_avoidance_hint`と`sensor_viewer`画像を配信する。
- `robot_navigator`：`active_target`・`amcl_pose`・`odom`・`scan`（または`obstacle_avoidance_hint`）を購読し、`cmd_vel`と`direction_marker`を配信する。

### 3. `route_state` と `follower_state` の表示整理
- `route_state`（`route_manager_node.py` で生成）からは以下を常時掲示する。
  - `status`：`RouteState.STATUS_*` 列挙値を文字列化し、`manager_status.state` と併せて運転フェーズを判定する。
  - `current_index`／`total_waypoints`：走破率をパーセントと進捗バーで表示し、`current_label` を現在地点ラベルとして併記する。
  - `route_version`：再計画発生を検出する指標とし、直近3回の変化履歴をタイムライン化する。
- `follower_state`（`route_follower_node.py` の `_handle_state_publish`）は以下を統合する。
  - `state`（`FollowerStatus` 列挙名）と `current_index`：`route_state` との差分で追従遅延や停止状態を色分けする。
  - `avoidance_attempt_count` と `last_stagnation_reason`：滞留対処状況をアラートカードで示す。
  - `front_blocked_majority`・`left_offset_m_median`・`right_offset_m_median`：障害物統計値として画像オーバレイに利用する。
  - `route_version` および `current_waypoint_label`／`next_waypoint_label`：ルート更新と次目標の整合チェックに活用する。

### 4. ダッシュボード中心の表示・操作方針
1. **状態サマリ**
   - 上段カードに各ノードの最終ハートビート、`manager_status.state`、`route_state.status`、`follower_state.state` をまとめ、停止や再計画を即座に判別する。
   - `route_state.current_index` と `follower_state.current_index` を並列に描画し、ズレ検出時は警告色で強調する。
   - `mission_info` と `route_state.route_version` の履歴を小型タイムラインに表示し、再計画理由の追跡を容易にする。

2. **障害物状況（画像重畳）**
   - `sensor_viewer` 画像左上に透過パネルを設け、以下の数値を重畳表示する。
     - 「前方遮蔽: YES/NO」（`front_blocked_majority`）
     - 「前方余裕: X.X m」（`obstacle_avoidance_hint.front_clearance_m`）
     - 「左/右回避中央値: ±X.XX m」（`left_offset_m_median`・`right_offset_m_median`）
   - ゲージ表現は廃止し、小数1～2桁で統一するとともに、最新受信時刻も同パネル内に併記する。

3. **信号・手動操作状況**
   - `manual_start` と `sig_recog` の最新値をラッチし、送信履歴を2行テーブルで表示する。
   - `road_blocked`（想定新規 Bool トピック）を同列に追加し、True 時は画面上部に警告バナーを表示する。
   - 下段に送信UIを常設し、`manual_start`／`sig_recog`／`obstacle_avoidance_hint` テスト送信／`road_blocked` の各ボタンの結果を即時に表示する。

4. **目標距離インジケータ**
   - `active_target`（PoseStamped）と `amcl_pose`（PoseWithCovarianceStamped）の水平距離を算出し、数値表示と簡易ゲージを併記する。
   - ゲージの最大値は「前回ターゲットと現在ターゲット間の距離」を基準に動的設定し、`arrival_threshold` 未満で警告色へ遷移させる。
   - `signal_stop` を含むウェイポイントに接近した場合は、外部カメラ切替（後述）と連動する。

5. **画像パネル（3面固定表示）**
   - `active_route.route_image`、`sensor_viewer` 画像、外部カメラ映像の3枚を常時表示し、比率は地図40%・センサ30%・カメラ30%を目安とする。
   - 外部カメラ領域は `follower_state.state == WAITING_STOP` かつ該当ウェイポイントが `signal_stop=True` の間は「信号監視カメラ」トピックへ切替え、それ以外は通常走行カメラトピックを表示する。
   - 各映像の更新レートは2～5Hzに制御し、タイムスタンプ遅延を検出した場合は警告を表示する。

6. **ノード起動・設定パネル**
   - 画面右側のスライドアウトパネルに、パッケージ単位の起動／停止ボタンと YAML パラメータ選択ドロップダウンを配置する。
   - Simulator を持つパッケージには「Simulator 同時起動」チェックボックスを設け、チェック時は起動コマンドへ simulator ノードを自動追加する。
   - 選択した YAML ファイルと起動コマンドを履歴に保存し、再起動時に再利用できるようにする。

7. **制御コマンドパネル**
   - `manual_start`・`sig_recog`・`obstacle_avoidance_hint` テスト送信・`road_blocked` の4カードをダッシュボード下段に配置する。
   - `obstacle_avoidance_hint` は「左回避」「右回避」「停止保持」などのプリセットトグルを用意し、送信中はカード上部に進行中バナーを表示する。
   - 各カードの下部に最新送信結果とトピック名を1行で表示し、連続操作時の確認コストを削減する。

8. **コンソールログパネル（タブ切替）**
   - 画面下部にタブ付きログビューを追加し、`route_manager`・`route_follower`・`obstacle_monitor`・`robot_navigator` ごとに stdout/stderr を分離して表示する。
   - ノード起動パネルから立ち上げたプロセスを監視し、出力が発生したタブに未読バッジを付与する。
   - タブは任意で切替可能としつつ、ショートカットでダッシュボードへ即時復帰できるようにする。

### 5. 詳細情報へのアクセス設計
- 詳細グラフや履歴はダッシュボードのモーダルまたはスライドパネルで提示し、タブ遷移を伴わないアクセスを基本とする。
- 例：`follower_state` 履歴、`manager_status.last_cause` のログ、`cmd_vel` 履歴グラフはカード上の「詳細」ボタンからオーバーレイ表示する。

### 6. 次のステップ
- 本方針でユーザー承認を得られれば、GUIモック（フェーズ2）でダッシュボード常時表示を前提にしたレイアウトを実装する。

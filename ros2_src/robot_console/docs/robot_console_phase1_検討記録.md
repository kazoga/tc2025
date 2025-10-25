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
  - `current_index`／`total_waypoints`：走破率をパーセントと進捗バーで表示し、ラベル表示はフォロワカードへ集約する。
  - `route_version`：再計画発生を検出する指標とし、直近3回の変化履歴をタイムライン化する。
- `follower_state`（`route_follower_node.py` の `_handle_state_publish`）は以下を統合する。
  - `state`（`FollowerStatus` 列挙名）と `current_index`／`current_waypoint_label`：ルート進捗カードと同列で並べ、インデックス差を視認しやすくする。
  - `avoidance_attempt_count` と `last_stagnation_reason`：滞留対処状況をアラートカードで示す。
  - `front_blocked_majority`・`left_offset_m_median`・`right_offset_m_median`：障害物統計値として画像オーバレイに利用する。
  - `route_version` および `next_waypoint_label`：ルート更新と次目標の整合チェックに活用する。

### 4. ダッシュボード中心の表示・操作方針
0. **モック整合性**
   - 実装時はフェーズ2で承認済みのモックUIとウィジェット構成・配置・サイズ比率を完全一致させる。変更が必要な場合は事前にユーザーへ仕様調整を提案し承認を得る。
1. **状態サマリ**
   - 上段は 3 カラム構成で、左から `route_state`、`follower_state`、ロボット速度／目標距離のスタックカードを配置する。各カラムは固定比率で幅を揃え、上端が水平にそろうよう uniform 設定を適用する。
   - `route_state` の「状態」欄には `route_manager` の `manager_status.state` を併記し、進捗バーと `current_index`／`total_waypoints` の数値を組み合わせて遅延を検知する。再計画要因と遷移時刻は「遷移要因」欄へまとめ、専用カードを廃止して情報を集約する。
   - 右カラムは `cmd_vel` を上段、`active_target` までの距離ゲージを下段に縦積みし、常時視界に入る形で速度と到達見込みを提示する。

2. **障害物状況（画像重畳）**
   - `sensor_viewer` 画像左上に透過パネルを設け、以下の数値を重畳表示する。
     - 「前方遮蔽: YES/NO」（`front_blocked_majority`）
     - 「前方余裕: X.X m」（`obstacle_avoidance_hint.front_clearance_m`）
     - 「左/右回避中央値: ±X.XX m」（`left_offset_m_median`・`right_offset_m_median`）
   - ゲージ表現は廃止し、小数1～2桁で統一するとともに、最新受信時刻も同パネル内に併記する。

3. **手動・信号・封鎖イベント**
   - `manual_start`・`sig_recog`・`road_blocked` は同時発生しない前提で切替表示される共通バナーとし、「手動・信号・封鎖イベント」というカードタイトルで制御コマンドパネルの左側に常設する。
   - バナーは未発生時でも固定幅を維持し、テキスト量に依存して制御コマンド欄の比率が揺らがないよう `grid` の `uniform` 設定でレイアウトする。`road_blocked` を検知した場合は最優先で警告色を表示し、`manual_start`／`sig_recog` は直近送信の状態を数秒間表示する。

4. **目標距離インジケータとロボット速度**
   - `cmd_vel` の並進速度と角速度（deg/s表示）を左カード、`active_target` までの距離ゲージを右カードに固定配置し、横並びで更新する。
   - ゲージの最大値は「前回ターゲットと現在ターゲット間の距離」を基準に動的設定し、`arrival_threshold` 未満で警告色へ遷移させる。
   - `signal_stop` を含むウェイポイントに接近した場合は、外部カメラ切替（後述）と連動する。

5. **画像パネル（3面固定表示）**
   - `active_route.route_image`、`sensor_viewer` 画像、外部カメラ映像の3枚を常時表示し、比率は地図40%・センサ30%・カメラ30%を目安とする。
   - 外部カメラ領域は `sig_recog` が STOP の間は `/sig_det_imgs` を表示し、それ以外は `/usb_cam/image_raw` を表示する。いずれも受信できない場合は「No Image」プレースホルダを描画する。
   - 各映像の更新レートは2～5Hzに制御し、タイムスタンプ遅延を検出した場合は警告を表示する。
   - 各画像はコンテナの領域内でアスペクト比を保持したまま最大サイズまでスケーリングし、不足分は背景色と同色のレターボックスで補正する。外部カメラは通常時16:9、信号監視時4:3を想定し、入力比率の切替にも破綻なく追随する。

6. **ノード起動・設定パネル**
   - 画面右側のスライドアウトパネルに、`route_planner`／`route_manager`／`route_follower`／`robot_navigator`／`obstacle_monitor` の順で起動カードを並べる。`route_planner` を追加し、順序はトップダウンで統一する。
   - 各カードには YAML パラメータ選択ドロップダウンを備え、Simulator を持つパッケージ（`robot_navigator` と `obstacle_monitor`）のみ「Simulator 同時起動」チェックボックスを表示する。
   - スクロール可能なフレーム構造を維持し、画面サイズによって下部が隠れる場合でもスクロールでアクセスできるようにする。

7. **制御コマンドパネル**
   - `manual_start`／`sig_recog`／`obstacle_avoidance_hint`／`road_blocked` の4項目をNotebookタブとして切り替え表示し、各タブは1～2行に収まるレイアウトで縦幅を半分以下に抑える。
   - `manual_start` タブは True／False をラジオボタンで選択し、「現在値」と「最終送信時刻」を1行にまとめて表示する。送信後は指定値をGUI状態にも反映し、road_blocked と同様に現状を把握できるようにする。
   - `sig_recog` は GO／STOP の2択ラジオボタンと送信ボタンで構成し、最終送信内容を履歴欄に表示する。
   - `obstacle_avoidance_hint` は1段目に余裕距離・左右オフセットのSpinbox、2段目に `front_blocked` 切替と送出開始／停止ボタン、3段目に現在の送出状態を置いてユーザーが視覚的に把握できるようにする。
   - `road_blocked` タブも True／False を選択して送信できるようにし、送信後は「現在値／最終送信時刻」を1行で表示する。

8. **コンソールログパネル（タブ切替）**
   - ダッシュボードとは別タブのログ画面を用意し、`route_planner`／`route_manager`／`route_follower`／`robot_navigator`／`obstacle_monitor` の順に 2 列グリッドへ配置する。上段左から右へ順に並べ、残る 1 枠は 3 行目に単独配置して上下方向の読み順が保たれるようにする。
   - 各パネルは縦スクロール対応の `Text` ウィジェットで、必要時に手動で切替えて参照する。未読バッジなどの監視は行わず、操作者が任意タイミングでログを確認する前提とする。ログデータは各ノードにつき最大2000行を保持し、上限超過時は古い行から破棄する。

### 5. 詳細情報へのアクセス設計
- 詳細グラフや履歴はダッシュボードのモーダルまたはスライドパネルで提示し、タブ遷移を伴わないアクセスを基本とする。
- 例：`follower_state` 履歴、`manager_status.last_cause` のログ、`cmd_vel` 履歴グラフはカード上の「詳細」ボタンからオーバーレイ表示する。

### 6. 次のステップ
- 本方針でユーザー承認を得られれば、GUIモック（フェーズ2）でダッシュボード常時表示を前提にしたレイアウトを実装する。

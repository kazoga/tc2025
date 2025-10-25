# 日本語文書

# robot_console 詳細設計書（フェーズ3）

## 1. 文書目的
robot_console パッケージの正式実装に先立ち、GUI構造・ROS通信仕様・更新周期・スレッド設計・ノード起動管理の詳細を定義する。本書はフェーズ2で承認されたモックUIを基礎に、ROS2 実装時に参照すべき要件を体系化したものである。実装段階ではモックで確定した画面仕様との完全整合を前提とし、差異が生じる場合は必ずユーザーへ仕様調整を提案し承認を得た後に反映する。

## 2. システム概要
### 2.1 モジュール構成
| モジュール | ファイル | 主責務 |
| --- | --- | --- |
| `RobotConsoleNode` | `robot_console/robot_console_node.py` | ROS2 ノード本体。購読・発行・サービス呼び出し・ノード起動要求の受付を担い、GUI層へ正規化済みデータを供給する。 |
| `GuiCore` | `robot_console/gui_core.py` | ROSスレッドとGUIスレッドの橋渡し。最新状態のリポジトリ、制御コマンドのキュー、画像変換キャッシュ、ノード起動マネージャを保持し、双方向通信を仲介する。 |
| `UiMain` | `robot_console/ui_main.py` | tkinter を用いた画面生成。Dashboard／ノード起動サイドバー／コンソールログタブを構築し、`GuiCore` から取得したスナップショットで描画を更新する。 |
| UI部品群 | `robot_console/ui_components/` | 状態カード・画像パネル・制御タブなど再利用部品を提供し、`UiMain` のレイアウトロジックを単純化する。 |
| 共有ユーティリティ | `robot_console/utils/` | dataclass 定義、色設定、PIL/NumPy 変換、フォーマッタなどをまとめる。 |
| モック／テスト | `tools/mock_ui.py` / `tools/tests/` | tkinter モックと pytest ベースのロジック単体テスト。GUI 実装時の回帰確認に使用する。 |

### 2.2 プロセス／スレッド構成
1プロセス構成で動作させ、以下のスレッドを常駐させる。
- **GUIスレッド**：`tkinter.Tk` が実行されるメインスレッド。`after(200ms)` で `GuiCore` からスナップショットを取得して再描画する。
- **ROS実行スレッド**：`rclpy.executors.MultiThreadedExecutor` を別スレッドで実行し、各購読コールバックから `GuiCore` へ状態を投入する。スレッド間共有には `threading.Lock` と `queue.Queue` を併用する。
- **ノード起動監視スレッド**：`NodeLaunchManager` が `subprocess.Popen` で起動したプロセスの stdout/stderr を非同期に読み取り、ログバッファへ流す。`selectors` モジュールでノンブロッキング読み取りを行い、GUI へイベント通知する。

### 2.3 GuiCore の内部構成
- **状態リポジトリ**：購読コールバックから受信した各メッセージを dataclass として保持する辞書。更新時は `threading.Lock` で排他し、GUI スナップショット生成時にコピーを提供する。
- **コマンドキュー**：GUI からの送信要求（manual_start、sig_recog、obstacle_hint、road_blocked、report_stuck など）を `queue.Queue` に積む。ROS 実行スレッド内のワーカーが逐次処理し、結果を状態リポジトリへ書き戻す。
- **画像変換キャッシュ**：`sensor_msgs/Image` を受信直後に `numpy.ndarray` として保持し、GUI スレッドでの `PIL.Image` 生成時に再利用する。レターボックス描画の矩形計算も GuiCore 側で実施し、UI へパラメータとして渡す。
- **イベント通知ハブ**：コマンド完了・ノード起動結果などのイベントを `asyncio.Event` 相当のフラグで管理し、GUI スレッドの `after()` ループが検知して表示更新を行う。tkinter へ直接的な他スレッド操作を行わないことで安全性を担保する。

### 2.4 スナップショット生成フロー
1. GUI スレッドが 200ms 周期で `GuiCore.snapshot()` を呼び出す。
2. `GuiCore` は状態リポジトリを `deepcopy` し、画像キャッシュは `PIL.Image` と描画矩形のみを含む `GuiSnapshot` dataclass にまとめる。
3. GUI スレッドは `GuiSnapshot` を参照してカード・バナー・画像パネル・制御タブを更新する。更新後の入力値は `GuiCore` の公開メソッドを介してコマンドキューへ投入する。

### 2.5 データモデル
`gui_core.py` では以下の dataclass を中心に状態を保持する。
- `RouteStateView`: `route_state` + `manager_status.state` を統合し進捗率・遷移履歴・最新バージョンを格納。
- `FollowerStateView`: インデックス・現在ラベル・次ラベル・滞留理由・左右オフセット中央値を保持。
- `ObstacleHintView`: `front_clearance_m`、`front_blocked_majority`、左右オフセット、最終更新時刻を保持。
- `ManualSignalView`: `manual_start`、`sig_recog`、`road_blocked` の最新値と送信タイムスタンプを保持し、`road_blocked` は外部ノードからの受信を常に優先する。
- `TargetDistanceView`: `active_target` と `amcl_pose` の距離と基準距離（前回ターゲット距離）。
- `CmdVelView`: 並進速度 [m/s] と角速度 [rad/s]。GUI 表示時に deg/s に変換する。
- `ImageBundle`: ルート地図・障害物ビュー・外部カメラ（走行／信号監視）の PIL 画像と最終更新時刻。
- `NodeLaunchState`: 起動可否、選択中パラメータ、シミュレータ有効状態、最終アクション時刻、ログリングのファイルディスクリプタ。
- `GuiSnapshot`: 上記 View 群とノード起動状態を束ねた読み取り専用のスナップショット。GUI スレッドにはこの構造体を渡す。

## 3. GUI構成設計
### 3.1 全体レイアウト
- GUI レイアウト・ウィジェット配置・表示順はフェーズ2で承認済みのモックUIと完全に一致させることを必須要件とする。差異が発生する場合は実装前にユーザーへ仕様調整を提案し、承認後に反映する。
- 基準解像度は 1280×720 (16:9)。ウィンドウリサイズ時はアスペクト比を固定し、コンテンツは `place(relwidth, relheight)` ではなく `grid` の `weight` と `uniform` を利用して比率を保持する。
- 画面は `Notebook` で「Dashboard」「Console Logs」の2タブ構成。運用時は Dashboard を常用し、ログは手動で参照する。
- Dashboard 内の主レイアウトは左右2列。左列に状態カード・画像・制御タブ、右列に折り畳み式ノード起動サイドバーを配置する。

### 3.2 ダッシュボード構成要素
| 項目 | 配置 | 情報源・更新 | 表示内容 |
| --- | --- | --- | --- |
| ルート進捗カード | 上段左 | `route_state` (1Hz) + `manager_status.state`（イベント駆動） | 状態ラベル、進捗バー (`current_index/total_waypoints`)、ルートバージョン履歴、再計画要因と時刻。 |
| フォロワ状態カード | 上段中央 | `follower_state` (20Hz) | `state`、`current_index`、`current_waypoint_label`、`next_waypoint_label`、滞留理由、左右中央値、回避試行回数。高速更新だが GUI では 5Hz に間引き表示。 |
| ロボット速度カード | 上段右 (上段) | `cmd_vel` (20Hz) | 並進速度 [m/s]、角速度 [deg/s]、閾値超過時の色変化。 |
| 目標距離カード | 上段右 (下段) | `active_target` + `amcl_pose` (5Hz) | 現在距離、基準距離、距離比ゲージ。到達閾値 1.0m 以下で警告色。 |
| 手動・信号・封鎖イベント | 中央左 | `manual_start`・`sig_recog`・`road_blocked` (イベント) | 3種のうち優先度順 (road_blocked > manual_start > sig_recog) で1件をバナー表示。未発生時は無地を維持しレイアウト幅を固定。 |
| 制御コマンドタブ | 中央右 | GUI操作 | Notebook で manual_start / sig_recog / obstacle_hint / road_blocked を切替。各タブは1～2行構成で縦幅を最小化。 |
| 画像パネル3面 | 下段 | `route_image` (更新時), `sensor_viewer` (5Hz), 外部カメラ (信号監視:4:3, 走行:16:9 / 5Hz) | それぞれ専用キャンバスに描画。アスペクト比を維持し、余白は背景色と同色でレターボックス処理。障害物ビュー左上に2行オーバレイ (遮蔽/余裕、左/右オフセット)。 |

### 3.3 制御コマンドタブ詳細
- **manual_start**: True/False ラジオボタン、送信ボタン、現在ラッチ値と最終送信時刻を1行表示。
- **sig_recog**: GO/STOP ラジオボタン、送信ボタン、最終送信値と時刻を表示。送信時に `std_msgs/Int32` で 1=GO, 2=STOP を送信。
- **obstacle_avoidance_hint**: 1段目に余裕距離・左右オフセットの `Spinbox`。2段目に `front_blocked` トグルボタン + 送出開始/停止ボタン。3段目に現在送出状態（自動／固定値／停止中）と最終更新時刻。
- **road_blocked**: True/False ラジオボタンと送信ボタン。現在値と最終送信時刻を1行表示。

### 3.4 ノード起動サイドバー
- 右端に折り畳み式の `PanedWindow`。開閉ボタンで幅を 320px（開）／40px（閉）に切替。
- 項目順：`route_planner`、`route_manager`、`route_follower`、`robot_navigator`、`obstacle_monitor`。
- サイドバー上部に「全起動」「全停止」ボタンを常設し、現在の Combobox 選択値とシミュレータ設定を尊重しながら、`NodeLaunchProfile` の優先順位順に逐次処理する。
- 各カードに含める要素：
  - 状態インジケータ（停止／起動中／エラー）。
  - パラメータファイル `Combobox`（既定は `params/default.yaml`）。
  - `robot_navigator` と `obstacle_monitor` のみ「Simulator 同時起動」チェックボックス (`robot_simulator` / `laser_scan_simulator`)。
  - 起動／停止ボタン。アクション時刻をツールチップに表示。
- パネル全体を `Canvas` + `Scrollbar` で包み、縦スクロールに対応。
- YAML 候補抽出は `NodeLaunchManager` 初期化時に実施する。`ament_index_python.packages.get_package_share_directory()` で各パッケージの `share/<package>` を特定し、以下の順で `glob('*.yaml')` を行う。
  1. `share/<package>/params/` と `share/<package>/config/` ディレクトリ（存在する場合）。
  2. `robot_console/config/node_params/<package>/`（運用側が追加オーバーライドを配置する想定の共有ディレクトリ）。
  3. `robot_console/config/node_launch_profiles.yaml` に定義した追加パス（例：共通テスト設定など）。
- 取得したパスは正規化したうえで重複除去し、ソートしたリストを `NodeLaunchProfile.available_params` に格納する。Combobox で選択できるのはこのリストに含まれるファイルのみとし、空リストの場合は起動ボタンを無効化して GUI に警告ラベルを表示する。

### 3.5 コンソールログタブ
- `Notebook` の第2タブに 2列グリッドで 5 セクションを配置。順序は上段左から `route_planner`、`route_manager`、`route_follower`、下段左 `robot_navigator`、下段右 `obstacle_monitor`。
- 各セクションは `LabelFrame` + `Text` ウィジェット + 縦スクロールバー。起動マネージャが受信した stdout/stderr を `INFO/ERR` プレフィックス付きで追記する。表示領域は操作者が必要に応じて開き、未読管理は行わない。

### 3.6 UI と ROS インタフェースの対応表
| UIコンポーネント | 入力/表示内容 | ROS側インタフェース | 備考 |
| --- | --- | --- | --- |
| ルート進捗カード | 状態・進捗率・再計画履歴 | `/route_state`, `/manager_status` | モックの表示内容と完全一致させる。 |
| フォロワ状態カード | 状態・インデックス・ラベル・オフセット | `/follower_state` | `current_waypoint_label` は Route メッセージのラベル辞書を参照。 |
| ロボット速度カード | 並進・角速度 | `/cmd_vel` | 角速度は GUI 側で deg/s に変換。 |
| 目標距離カード | 現在距離・基準距離 | `/active_target`, `/amcl_pose` | ターゲット切替時に基準距離を更新。 |
| 手動・信号・封鎖バナー | 優先度付きイベント表示 | `/manual_start`, `/sig_recog`, `/road_blocked` | 各トピックを購読し、未受信時はデフォルト状態を維持する。 |
| manual_start タブ | True/False 選択と送信 | `/manual_start` | 送信値は `Bool.data`。現在値は購読結果（未受信時は既定False）を表示。 |
| sig_recog タブ | GO/STOP 送信 | `/sig_recog` | 1=GO, 2=STOP を送信。現在値は購読結果（未受信時は未定義表示）を表示。 |
| obstacle_hint タブ | 余裕距離・左右オフセット・front_blocked・送出制御 | `/obstacle_avoidance_hint` | 固定値送出中は同一トピックへ上書き送信し、停止後は購読値へ復帰。 |
| road_blocked タブ | True/False 送信 | `/road_blocked` | トピックを購読し、外部ノードからの受信値を常に優先表示する。未接続の場合は直近送信値を明示する。 |
| 画像パネル（ルート地図） | 2100×1200 のアスペクト比を維持して縮小 | `/active_route` 内に含まれる `route_image` | 受信できない場合は「No Image」プレースホルダを表示する。 |
| 画像パネル（障害物ビュー） | 1:1 画像 + 数値オーバレイ | `/sensor_viewer`, `/obstacle_avoidance_hint` | オーバレイ文字列は GuiCore で整形。 |
| 画像パネル（外部カメラ） | 16:9 / 4:3 レターボックス描画 | `/usb_cam/image_raw`, `/sig_det_imgs` | `sig_recog` が STOP の間は `/sig_det_imgs` を表示し、未取得時は「No Image」プレースホルダを描画。 |
| ノード起動カード各種 | 起動/停止、パラメータ選択、Simulator チェック | `ros2 launch` 経由で `route_planner` などを起動 | パラメータファイルは自動抽出リストを利用。 |
| コンソールログタブ | stdout/stderr 表示 | NodeLaunchManager の `stdout`, `stderr` | 未読管理は行わず、常に最新状態のみ表示。 |

### 3.7 入力操作と実行処理の対応
| UI操作 | 処理内容 | 呼び出し先／トピック | 備考 |
| --- | --- | --- | --- |
| manual_start「送信」ボタン | 選択中の True/False を `queue.Queue` へ投入し、ROS ワーカーが `/manual_start` へラッチ送信する。 | `GuiCore.request_manual_start()` → `NodeCommandWorker.publish_bool()` | 送信結果は購読コールバックで確認し、GUI の現在値へ反映。 |
| sig_recog「送信」ボタン | GO/STOP を 1/2 に変換して送信。 | `GuiCore.request_sig_recog()` → `/sig_recog` | 送信後に購読値を待ち、受信が無い場合はタイムアウト警告。 |
| obstacle_hint「送出開始」ボタン | Spinbox／front_blocked の設定値で固定値送出を開始。 | `GuiCore.start_obstacle_override()` → `/obstacle_avoidance_hint` | 0.5Hz タイマーで送信し続け、状態を「固定値送出中」に更新。 |
| obstacle_hint「送出停止」ボタン | 固定値送出タイマーを停止し、通常受信値へ復帰。 | `GuiCore.stop_obstacle_override()` | 停止時に 0 クリアメッセージを送信。 |
| obstacle_hint「front_blocked」トグル | GUI内部状態の True/False を切替。 | GUI 内部 | 送出開始時に最新値が使用される。 |
| road_blocked「送信」ボタン | 選択中の True/False を送信。 | `GuiCore.request_road_blocked()` → `/road_blocked` | 未購読状態の場合は GUI で「送信済み／受信待ち」を表示。 |
| ノードカード「起動」ボタン | 選択された YAML とシミュレータ設定で個別起動。 | `GuiCore.request_launch(profile_id)` → `NodeLaunchManager.launch()` | 起動完了後に PID とログストリームを登録。 |
| ノードカード「停止」ボタン | 対応ノードのプロセス停止。 | `GuiCore.request_stop(profile_id)` → `NodeLaunchManager.stop()` | タイムアウト時は kill を実行し、状態にエラーを反映。 |
| サイドバー「全起動」ボタン | 定義順に起動を連続実行（失敗時は処理停止しエラー表示）。 | `GuiCore.request_launch_all()` | 各ノードの選択 YAML／Simulator 設定を尊重。 |
| サイドバー「全停止」ボタン | 稼働中ノードを逆順で停止。 | `GuiCore.request_stop_all()` | kill 実行時は警告ダイアログを表示。 |
| YAML Combobox 選択 | `NodeLaunchState.selected_param` を更新。 | GUI 内部 | 起動要求時に参照される。ファイル存在チェックに失敗した場合は赤枠表示。 |
| Simulator チェックボックス | `NodeLaunchState.simulator_enabled` を更新。 | GUI 内部 | 起動要求時に参照される。 |
| ノードサイドバー開閉トグル | `PanedWindow` の幅を切替。 | GUI 内部 | 状態は GuiCore に保持し、再表示時に復元。 |

## 4. 通信仕様
### 4.1 購読トピック
| トピック名 | 型 | 想定更新 | 利用箇所 |
| --- | --- | --- | --- |
| `/route_state` | `route_msgs/msg/RouteState` | 1Hz (`state_publish_rate_hz`) | ルート進捗カード、再計画履歴。 |
| `/manager_status` | `route_msgs/msg/ManagerStatus` | 状態遷移時 | 状態ラベル・再計画要因。 |
| `/active_route` | `route_msgs/msg/Route` | ルート更新時 | ルート地図画像、ウェイポイント一覧。 |
| `/mission_info` | `route_msgs/msg/MissionInfo` | 起動時／ルート更新時 | ミッション概要（カード内ツールチップ）。 |
| `/follower_state` | `route_msgs/msg/FollowerState` | 20Hz (`control_rate_hz`) | フォロワ状態カード、障害物統計。 |
| `/active_target` | `geometry_msgs/msg/PoseStamped` | 20Hz | 目標距離カード、地図描画。 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 20Hz | ロボット速度カード。 |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 10Hz | 目標距離カード、地図描画。 |
| `/odom` | `nav_msgs/msg/Odometry` | 30Hz | 速度比較（将来拡張用ログ）。 |
| `/obstacle_avoidance_hint` | `route_msgs/msg/ObstacleAvoidanceHint` | 10Hz | 障害物パネル、オーバレイ数値。 |
| `/sensor_viewer` | `sensor_msgs/msg/Image` | 5Hz | 障害物画像パネル。 |
| `/usb_cam/image_raw` | `sensor_msgs/msg/Image` | 5Hz | 走行時カメラパネル (16:9)。 |
| `/sig_det_imgs` | `sensor_msgs/msg/Image` | 5Hz | 信号停止時カメラパネル (4:3)。 |
| `/manual_start` | `std_msgs/msg/Bool` | イベント（ラッチ） | 手動・信号・封鎖バナー、manual_start タブの現在値。 |
| `/sig_recog` | `std_msgs/msg/Int32` | イベント（ラッチ） | 手動・信号・封鎖バナー、sig_recog タブの現在値。 |
| `/road_blocked` | `std_msgs/msg/Bool` | イベント（ラッチ） | 手動・信号・封鎖バナー、road_blocked タブの現在値。 |

### 4.2 発行トピック
| トピック名 | 型 | 送信契機 | 内容 |
| --- | --- | --- | --- |
| `/manual_start` | `std_msgs/msg/Bool` | manual_start タブ送信時 | True/False をラッチ送信し、復帰命令を発行。 |
| `/sig_recog` | `std_msgs/msg/Int32` | sig_recog タブ送信時 | 1=GO、2=STOP をラッチ送信。 |
| `/obstacle_avoidance_hint` | `route_msgs/msg/ObstacleAvoidanceHint` | 固定値送出開始／停止 | GUI入力値をそのまま配信し、停止時はゼロ値を送信して obstacle_monitor へ制御を戻す。 |
| `/road_blocked` | `std_msgs/msg/Bool` | road_blocked タブ送信時 | 道路封鎖検知の仮想トピック。現在は他ノード未購読だがログに残す。 |

### 4.3 サービス・アクション
- `route_manager/report_stuck` サービスを GUI から任意送信できるよう、`GuiCore` に再送 API を用意する（フェーズ4で必要に応じてUIへ追加）。
- ノード起動・停止は ROS サービスではなく、`launch` API で独自管理する。

### 4.4 トピック送信処理の実装フロー
- `RobotConsoleNode` 起動時に `/manual_start`、`/sig_recog`、`/obstacle_avoidance_hint`、`/road_blocked` 用の `Publisher` を生成し、QoS は既存ノードの期待に合わせて以下を採用する。
  - `/manual_start`・`/sig_recog`：`QoSProfile(depth=1, reliability=RELIABLE, durability=TRANSIENT_LOCAL)`。既存ノードがラッチ値を参照するため、Transient Local で保持する。
  - `/obstacle_avoidance_hint`：`QoSProfile(depth=10, reliability=RELIABLE, durability=VOLATILE)`。固定送出時のみ継続配信し、停止時にゼロクリアを送る。
  - `/road_blocked`：`QoSProfile(depth=1, reliability=RELIABLE, durability=TRANSIENT_LOCAL)`。外部ノードの発行が存在する場合は外部値を優先するため、GUI 送信後は購読値で上書きされ得る。
- GUI 操作で生成されたコマンドは `GuiCore` のコマンドキューへ投入され、`RobotConsoleNode` 内の `NodeCommandWorker`（ROS スレッド側）が FIFO で処理する。
- `NodeCommandWorker` はコマンド種別ごとに専用メソッドを呼び出し、送信前に以下の検証を行う。
  - 最終送信からの経過時間（連打防止）。
  - GUI 側で入力された値の型・範囲チェック（例：距離が0以上か、オフセットが許容範囲か）。
- 送信後は `GuiCore` に結果をフィードバックし、対応する View（例：`ManualSignalView`）のラッチ値を更新する。購読コールバックから受信した最新値が矛盾する場合は受信値を優先する。

## 5. robot_console パラメータ設計
- `robot_console/config/robot_console.yaml` を既定のパラメータファイルとし、`RobotConsoleNode` 起動時に `declare_parameters` で読み込む。
- 主なパラメータ定義（セクション別に整理する）。
  - `ui.refresh_period_ms`（default: 200）：GUI スレッドがスナップショットを取得する周期。100〜500ms の範囲で調整可能。
  - `ui.image_rate_limit_hz.route_image` / `sensor_viewer` / `camera_drive` / `camera_signal`（default: 2 / 5 / 5 / 5）：画像毎の最大更新周波数。
  - `ui.distance.arrival_threshold_m`（default: 1.0）：目標距離カードの警告閾値。
  - `commands.cooldown_ms.manual_start` / `sig_recog` / `road_blocked`（default: 500）：同一コマンドを連打した際に無視するクールダウン時間。
  - `commands.override_timer_hz.obstacle_hint`（default: 0.5）：障害物固定送出の周期。
  - `logs.buffer.max_lines`（default: 2000）：ログリングバッファの上限。超過時は古い行から破棄する。
  - `logs.console_level`（default: `info`）：robot_console 自身のログ出力レベル。`debug` を指定するとデバッグ時のみ追加情報を出力する。
  - `topics.camera.drive`（default: `/usb_cam/image_raw`）、`topics.camera.signal`（default: `/sig_det_imgs`）：外部カメラの購読トピック。
  - `topics.road_blocked.external_priority`（default: `true`）：外部ノードからの road_blocked 値を GUI 操作より優先するかどうか。
- パラメータが未設定の場合は既定値で補完し、欠落を `warn` ログで通知する。`ros2 param set` による変更が入った場合は即時に反映し、`topics.*` を更新した際は購読の再生成を行う。

## 6. 更新・同期設計
- **GUI更新**：`UiMain` が 200ms ごとに `GuiCore.snapshot()` を呼び出す。スナップショットはディープコピーされた dataclass 群で、GUI 側での加工によりスレッド安全性を確保する。
- **画像処理**：ROS コールバックで `sensor_msgs/Image` を受信後、`cv_bridge` で OpenCV 画像へ変換し、`PIL.Image` に変換。アスペクト比計算を `GuiCore` 側で行い、Tkinter では `PhotoImage` を生成する。画像の描画サイズは各パネルのピクセル数から算出し、背景色 (`#1f1f1f`) と同色のレターボックスを描く。
- **距離計算**：`active_target` と `amcl_pose` の距離は `tf_transformations` を用いず、単純なユークリッド距離で算出。新しいターゲット受信時に `reference_distance_m` を更新し、到達割合をリセットする。
- **イベントバナー**：`ManualSignalView` を参照し、`road_blocked` → `manual_start=True` → `sig_recog` の順に優先。`road_blocked` は外部ノードからの受信値を GUI 操作よりも優先して表示する。`manual_start=False` 送信時は5秒でバナーをクリアする。
- **障害物固定送出**：送出開始時に内部状態 `override_active=True` を設定し、0.5Hz のタイマーで上書きメッセージを送信する。停止ボタンでタイマーを解除し、通常の受信値表示に復帰する。

## 7. ノード起動管理
- `NodeLaunchProfile`（dataclass）で以下を定義：`profile_id`, `display_name`, `package`, `launch_file`, `param_argument`, `default_param`, `simulator_launch_file`。
- プロファイル定義は `robot_console/config/node_launch_profiles.yaml` に集約し、各ノードごとに以下を記載する。
  - `default_param`: 既定で選択する YAML ファイル。
  - `param_search_paths`: `share` 直下からの相対パスまたは絶対パスのリスト。ワイルドカード対応。
  - `simulator_profile`: シミュレータ併起動に必要な launch 設定（対象パッケージ、実行可能名、追加パラメータ）。
  - `launch_priority`: 「全起動」操作時の起動順序。小さいほど優先。
- `NodeLaunchManager` 初期化時に `NodeLaunchProfile` を読み込み、前述のパス探索ルールに沿って `available_params` を構築する。`share/<package>/params/` 等に存在しない場合はログへ警告を残し、GUI 側では該当カードの起動ボタンを無効化する。また、`route_manager`、`route_follower`、`robot_navigator`、`obstacle_monitor`、`route_planner` の `setup.py` を確認し、いずれも `share/<package>/params/` へ YAML がインストールされることを事前に検証済みである。
- `NodeLaunchManager` は `subprocess.Popen` を用いて `ros2 launch <package> <launch_file>` を実行し、各ノードを独立プロセスとして起動する。`NodeLaunchProfile` には `package`, `launch_file`, `param_argument`, `default_param`, `simulator_launch_file`, `display_name`, `profile_id` を保持する。`param_argument` が指定されている場合のみ `param_file:=<path>` を引数に追加し、`simulator_launch_file` があるプロファイル（robot_navigator と obstacle_monitor）ではチェックボックスに応じてシミュレータ用の `ros2 launch` を追加起動する。
- 追加パラメータファイルは `share/<package>/params/` と `share/<package>/config/` を起点に探索し、`robot_console/config/node_params/<package>/` や `node_launch_profiles.yaml` に定義されたパスも併せて取り込む。`route_planner`、`route_manager`、`route_follower`、`robot_navigator`、`obstacle_monitor` の `setup.py` を確認し、いずれも `share/<package>/params/` へ YAML がインストールされることを事前に確認済みである。
- **起動要求の処理手順**：
  1. GUI から `GuiCore.request_launch()` が呼ばれると、コマンドキューに `profile_id` が積まれる。
  2. ROS 側ワーカーがキューを取り出し、対応する `NodeLaunchProfile` と `NodeLaunchState.selected_param` を参照してコマンド配列 `['ros2', 'launch', package, launch_file, param_argument:=<path>]` を組み立てる。
  3. `subprocess.Popen(text=True)` でプロセスを起動し、PID と状態を `NodeLaunchState` に反映する。stdout/stderr は行単位で読み出すスレッドを生成し、`ConsoleLogBuffer` に追記する。シミュレータを併起動した場合はログ行頭に `[SIM]` を付与する。
- **停止要求の処理手順**：
  1. `GuiCore.request_stop()` が呼ばれると対象 `profile_id` のプロセスを取得し、まず `SIGINT` を送信する。
  2. 5 秒待っても終了しない場合は `terminate()`（SIGTERM）を発行し、さらに 3 秒待機する。
  3. それでも終了しない場合は `kill()` を実行し、最終的な終了を確認する。同時に起動しているシミュレータにも同じ手順を適用する。
  4. 停止結果は `NodeLaunchState.status` と `process_id`／`simulator_process_id` に反映し、GUI に伝達する。
- **ログ収集**：標準出力・標準エラーを監視するスレッドはプロファイルごとに生成し、テキストモードで読み出した行を `ConsoleLogBuffer(capacity=2000)` に格納する。容量超過時は古い行から破棄し、未読管理は行わない。
- **全起動／全停止**：`GuiCore.request_launch_all()` は `default_launch_profiles()` の定義順に起動を連続要求し、失敗したプロファイルが出た時点で処理を停止してエラーを状態に記録する。`request_stop_all()` は逆順に停止要求を出し、強制終了が発生した場合は `NodeLaunchStatus.ERROR` とエラーメッセージを表示する。
- **エラーハンドリング**：`ros2 launch` コマンドが開始できなかった場合や非ゼロ終了コードを返した場合は `NodeLaunchStatus.ERROR` とし、stderr の先頭行を `NodeLaunchState.error_message` に格納して GUI 側に通知する。再起動ボタンを押すまで状態は保持する。

## 8. デバッグログ方針
- 動作確認や障害調査はユーザーが robot_console を実行し、結果をフィードバックする前提である。GUI 操作中に常時表示するログは `info` 以上に限定し、デバッグ時のみ詳細情報を `debug` レベルで出力する。
- `logs.console_level` パラメータが `debug` の場合にのみ以下の追加ログを有効化する。
  - 購読トピックごとの最終受信タイムスタンプと遅延秒数。
  - コマンド送信結果（送信値、QoS 設定、パブリッシュ時刻）。
  - ノード起動要求で実行した `ros2 launch` コマンドと起動所要時間。
- デバッグログは `robot_console` ノードの stdout に出力し、GUI のログタブには転送しない。ユーザーが CLI 実行結果を共有することで開発側が状況を把握できるようにする。
- エラー調査時には `ros2 run robot_console robot_console --ros-args --log-level debug` で実行してもらう運用を想定し、ドキュメントに手順を記載する。

## 9. エラーハンドリング
- ROS通信途絶（タイムアウト）時は該当カードに「更新停止 (XX秒)」の警告を表示。`GuiCore` が最終更新時刻からの経過秒数を算出する。
- 画像変換失敗時はプレースホルダ画像とともにログにエントリを追加する。
- ノード起動失敗時は stderr の先頭 5 行をダイアログ表示し、状態カードに「起動失敗」ラベルを表示。
- サービス呼び出し失敗・コマンド送信例外は GUI 下部のステータスバーに赤字で表示し、10 秒後に自動クリア。

## 10. テスト計画
| テスト種別 | 目的 | 対象 | 手法 |
| --- | --- | --- | --- |
| ユニットテスト | データ変換・距離計算の正当性確認 | `gui_core` | `pytest`＋ダミーメッセージ生成。 |
| UI テスト | ウィジェット配置の回帰検証 | `ui_main` | `tools/mock_ui.py` を基準スクリーンショット比較（手動）。 |
| プロセス制御テスト | ノード起動・ログ収集の確認 | `NodeLaunchManager` | `subprocess` をモック化し、状態遷移を検証。 |
| 統合テスト | ROS通信とGUI更新の連携確認 | `robot_console` | `rclpy` の `Node` を用いた通信モックと Tkinter イベントループの結合試験（CI では headless 実行）。 |

## 11. 今後の拡張余地
- `rviz` からの Marker 表示や地図重畳を `matplotlib` ベースのサブビューで提供する余地がある。
- `rosbag2` 録画制御やログ出力の保存先切替を GUI から操作できるようにする。
- タブ切替なしで詳細履歴を参照できるオーバーレイダイアログ（例：速度履歴チャート）の追加。

## 12. 未確定事項と対処方針
- **report_stuck サービス UI**：現行モックでは省略しているが、将来的に GUI から `ReportStuck` を送信する需要がある場合は、ダイアログ型の追加入力を検討する。要否についてユーザーと改めて協議し、承認を得た上でレイアウトへ組み込む。

## 13. 実装スケジュール目安
1. `gui_core` のデータモデルと ROS 購読実装（約2日）。
2. `ui_components` の部品実装と `ui_main` への組み込み（約2日）。
3. ノード起動マネージャとログ取得（約1日）。
4. 発行トピック／障害物固定送出とテスト整備（約1日）。
5. 総合調整とドキュメント更新（約1日）。

以上をもって、robot_console パッケージの実装フェーズに進むための詳細設計とする。

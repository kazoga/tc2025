# obstacle_monitor_詳細設計書_phase2

## 0. 文書目的
本書は obstacle_monitor パッケージに含まれる `ObstacleMonitorNode` の現行実装を対象とし、route_manager 系設計書と同じ章構成で詳細仕様を整理する。LiDAR からの障害物ヒント生成、viewer 可視化、QoS やテスト観点を明確にして、他ノード（robot_navigator など）との連携設計を理解しやすくする。

---

## 1. ノード概要

| 項目 | 内容 |
|------|------|
| ノード名 | obstacle_monitor |
| クラス構成 | `ObstacleMonitorNode`（Node 派生 + OpenCV/CvBridge + numpy 補助関数） |
| 主機能 | `/scan` を解析して前方閉塞判定と回避オフセットを算出し、`ObstacleAvoidanceHint` とデバッグ画像を配信する。 |
| 実装言語 | Python3（Google Python Style + 型ヒント） |
| 対象 ROS2 ディストリビューション | Foxy |

---

## 2. 責務とスコープ

### 2.1 責務
- `/scan` から前方 ±90° の点群を抽出し、障害物距離と左右開放幅を解析する。
- legacy 実装に準じた回避オフセット算出ロジックを用いて `ObstacleAvoidanceHint` を生成する。
- `/sensor_viewer` に LiDAR 点群や回避候補を描画した画像を publish する。
- `/amcl_pose` と `/active_target` を参照して viewer 上のロボット姿勢・目標位置を更新する。

### 2.2 スコープ外
- 複雑な地図ベースの経路計画。障害物検出とヒント出力までを担い、迂回決定は他ノードに委ねる。
- `/scan` 以外のセンサデータ処理。マルチセンサ統合は対象外。
- 画像 viewer の GUI 例外処理。基本的な OpenCV エラー捕捉のみを行う。

---

## 3. 入出力インタフェース

### 3.1 購読トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/scan` | `sensor_msgs/LaserScan` | 前方障害物検知用 LiDAR。SensorDataQoS で受信する。 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | viewer 描画時のロボット姿勢参照。 |
| `/active_target` | `geometry_msgs/PoseStamped` | viewer とログ用に目標位置を保持する。 |

### 3.2 公開トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/obstacle_avoidance_hint` | `route_msgs/ObstacleAvoidanceHint` | front_blocked、front_clearance_m、左右オフセットを配信。BestEffort／Volatile。 |
| `/sensor_viewer` | `sensor_msgs/Image` | LiDAR 点群と回避候補を描画した bgr8 画像。BestEffort／Volatile。 |

### 3.3 サービス
- なし。

---

## 4. パラメータ

| 名称 | 型 | 既定値 | 説明 |
|-------------|----|---------|------|
| `front_cone_half_deg` | double | 10.0 | 前方閉塞判定のくさび半角。 |
| `stop_dist_m` | double | 1.0 | くさび内で停止とみなす距離。 |
| `max_obstacle_distance_m` | double | 1.5 | 回避オフセット算出対象の最大距離。 |
| `hint_range_m` | double | `max_obstacle_distance_m` | `front_clearance_m` の通知上限。設定が小さい場合は警告し補正する。 |
| `robot_width_m` | double | 0.8 | 回避オフセット計算に用いるロボット幅。 |
| `avoid_offset_min_m` | double | 0.75 | 左右回避オフセットの下限。 |
| `viewer_map_range_m` | double | 8.0 | viewer 画像の表示範囲 [m]。 |
| `viewer_pixel_pitch` | int | 100 | 1m あたりのピクセル数。 |
| `viewer_window` | string | `laserscan` | viewer ウィンドウ名。 |
| `viewer_resize_px` | int | 500 | 表示用にリサイズする画像サイズ。 |

---

## 5. データ構造
- 最新の点群を numpy 配列（`points_xy`）として保持し、左右分離や距離ソートに利用する。
- viewer 描画用に `_last_points_xy`、`_last_hint`、`_last_pose` を保持して `CvBridge` で画像に変換する。
- `ObstacleAvoidanceHint` 生成時に使用する中間構造（`_left_offsets`、`_right_offsets`）を局所スコープで作成し、余分なメモリ保持を避ける。

---

## 6. 状態遷移仕様

| 状態 | 概要 | 遷移契機 |
|------|------|----------|
| `WAIT_SCAN` | `/scan` 未受信。ヒントは配信されない。 | 起動直後 |
| `ACTIVE` | `/scan` を処理しヒント・viewer を配信する通常状態。 | LaserScan 受信継続 |

---

## 7. 主処理仕様
- `laser_scan_callback()`: LaserScan を受信すると NaN/Inf/負値を除外し、前方 ±90° の点に限定する。`max_obstacle_distance_m` 以内の点を抽出し、左右に分離して |y| 昇順にソートする。
- `calc_avoidance_offset()`: legacy 相当のロジックで左右の最小回避オフセットを計算し、`avoid_offset_min_m` を下限として適用する。
- `build_hint_message()`: くさび内で `stop_dist_m` 未満の点があれば `front_blocked=True` とし、最小距離を `front_clearance_m` に設定。距離が `hint_range_m` を超える場合は警告してクリップする。
- `_publish_hint_and_viewer()`: 生成した `ObstacleAvoidanceHint` を publish し、viewer 画像を描画して `/sensor_viewer` に送出する。OpenCV 例外時はエラーログを出し描画をスキップする。

---

## 8. クラス構成
- `ObstacleMonitorNode`：Node 継承クラス。パラメータ宣言、QoS 設定、publisher／subscriber 登録、CvBridge 初期化を行う。
- 補助メソッド `_ensure_hint_range()`、`_create_viewer_image()` によりパラメータ補正と画像生成を分離し、メインコールバックの見通しを良くしている。

---

## 9. QoS設計
- `/scan` 購読は `qos_profile_sensor_data` を使用し、低遅延で受信する。
- `/obstacle_avoidance_hint` と `/sensor_viewer` の publisher は BestEffort／Volatile／depth=1 とし、最新値のみを共有する。
- `/amcl_pose`／`/active_target` 購読は RELIABLE／Volatile／depth=10 とし、姿勢情報の欠落を防ぐ。

---

## 10. エラー処理・ログ方針
- `hint_range_m` が `max_obstacle_distance_m` 未満の場合は WARN を出して自動的に引き上げる。
- viewer 生成中の OpenCV エラーは ERROR ログを出し、以後の描画をスキップする設計（実装では例外を表層で捕捉）。
- `/scan` 未受信時は WARN ログを一定間隔で出して監視し、最新値が得られない状態を可視化する。

---

## 11. Phase3 拡張想定
- `ObstacleAvoidanceHint` に回避候補スコアや推奨方向を追加し、Navigator 側の判断材料を増やす。
- `/scan` とカメラ画像を融合したデバッグビュー生成、遠距離フィルタリング機能の追加。
- 回避オフセットの履歴や一貫性チェックを導入し、振動的な左右切り替えを抑制する。

---

## 12. テスト観点

| テスト種別 | 目的 | 検証項目 |
|-------------|------|----------|
| ユニットテスト | 幾何計算検証 | 回避オフセット算出の境界条件、くさび判定、`_ensure_hint_range()` の補正処理 |
| コンポーネントテスト | ノード単体検証 | `/scan` 入力に対する hint 出力の妥当性、viewer 画像生成、パラメータ変更の反映 |
| 結合テスト | 他ノード連携 | `ObstacleAvoidanceHint` 経由で RobotNavigator が減速・停止すること、LaserScanSimulator とのループバック |
| シナリオテスト | 実環境検証 | 実機 LiDAR でのヒント挙動、閉塞・開放シナリオ、ヒント未受信時のフェールセーフ |

---

## 13. まとめ
- ObstacleMonitorNode は legacy 相当の回避ロジックを ROS2 上で再現し、`ObstacleAvoidanceHint` と viewer 可視化を提供する。
- 主要パラメータや QoS、エラーハンドリングを整理することで、他パッケージ文書との粒度を揃えた。
- 本書により障害物ヒント生成の仕様が単独で把握でき、レーザースキャンシミュレータ文書と併読しやすい構造とした。

# laser_scan_simulator_詳細設計書_phase2

## 0. 文書目的
本書は obstacle_monitor パッケージに含まれる `LaserScanSimulatorNode` の現行実装を対象とし、route_manager 系文書と同一章構成で詳細設計を整理する。画像地図からの疑似 LaserScan 生成手順、入出力仕様、パラメータ、状態遷移、テスト観点を明確化し、ObstacleMonitorNode との連携検証を支援する。

---

## 1. ノード概要

| 項目 | 内容 |
|------|------|
| ノード名 | laser_scan_simulator |
| クラス構成 | `LaserScanSimulatorNode`（Node 派生 + 画像レイキャスト処理 + OpenCV） |
| 主機能 | 2D 画像地図とロボット姿勢から LaserScan を擬似生成し `/scan` を publish、デバッグ描画を提供する。 |
| 実装言語 | Python3（Google Python Style + 型ヒント） |
| 対象 ROS2 ディストリビューション | Foxy |

---

## 2. 責務とスコープ

### 2.1 責務
- 地図画像（B/W）と解像度を読み込み、ロボット位置を画像座標へ変換する。
- `/amcl_pose` から取得した姿勢に基づき、レイキャストで障害物との距離を計算し LaserScan を生成する。
- `/scan` を設定レートで publish し、ObstacleMonitorNode などが利用できるようにする。
- デバッグ用のウィンドウ描画を行い、スキャン結果を視覚的に確認できるようにする。

### 2.2 スコープ外
- 地図の動的更新やマルチレイヤ処理。静的な画像地図を前提とする。
- 実センサのノイズモデル再現。単純な最大・最小距離制限のみを行う。
- `/scan` 以外の距離計測データ生成。ポイントクラウド等は対象外。

---

## 3. 入出力インタフェース

### 3.1 購読トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | シミュレータ内部のロボット姿勢を更新し、レイキャストの基準とする。 |

### 3.2 公開トピック

| トピック名 | 型 | 説明 |
|-------------|----|------|
| `/scan` | `sensor_msgs/LaserScan` | 画像地図からレイキャストした疑似スキャン。`angle_*` パラメータと `range_*` で設定された仕様に従う。 |

### 3.3 サービス
- なし。

---

## 4. パラメータ

| 名称 | 型 | 既定値 | 説明 |
|-------------|----|---------|------|
| `map_image_path` | string | `/tmp/map.bmp` | 使用する地図画像のパス。 |
| `map_resolution_m` | double | 0.2 | 画像 1 ピクセルあたりの実距離 [m]。 |
| `publish_rate_hz` | double | 40.0 | `/scan` 配信レート。 |
| `enable_debug_view` | bool | True | OpenCV ウィンドウ描画を行うか。 |
| `debug_view_rate_hz` | double | 10.0 | デバッグビュー更新レート。 |
| `angle_min_deg` | double | -135.0 | スキャン開始角度。 |
| `angle_max_deg` | double | 135.0 | スキャン終了角度。 |
| `angle_increment_deg` | double | 0.25 | スキャン角度刻み。 |
| `range_min` | double | 0.05 | 最小測距距離。 |
| `range_max` | double | 30.0 | 最大測距距離。 |
| `map_unknown_threshold` | int | 250 | 障害物判定に使用する画素閾値。 |
| `ray_step_m` | double | 0.05 | レイキャスト時のステップ幅 [m]。 |

---

## 5. データ構造
- `self.map_img`: OpenCV で読み込んだ地図画像。グレースケールで保持する。
- `self.last_scan_angles`／`self.last_scan_ranges`: numpy 配列で最後に計算した角度・距離を保持し、デバッグビューや再送に使用する。
- `self.robot_pose_xytheta`: 最新のロボット姿勢（x, y, θ）を保持するタプル。スレッドロックで更新する。
- `self._lock`: 姿勢とスキャン結果を保護する `threading.Lock`。タイマコールバックと購読コールバックの競合を避ける。

---

## 6. 状態遷移仕様

| 状態 | 概要 | 遷移契機 |
|------|------|----------|
| `WAIT_POSE` | `/amcl_pose` 未受信。初期画像上の基準位置を使用しスキャンは送出しない。 | 起動直後 |
| `TRACK_POSE` | AMCL から受信した姿勢に基づきレイキャストを実行し、`/scan` を配信する通常状態。 | `/amcl_pose` 受信 |

---

## 7. 主処理仕様
- `_on_pose()`: `/amcl_pose` を受信し、姿勢を画像座標へ変換する。ROS 座標系から画像座標系への +90° 回転・スケール変換を適用する。
- `_timer_publish_scan()`: `publish_rate_hz` に従って呼び出され、各角度にレイキャストして障害物距離を計算。距離が `range_min` 未満であれば `range_min` に、`range_max` を超えれば `range_max` に丸めた上で `LaserScan` を publish する。
- `_timer_debug_view()`: `enable_debug_view` が真の場合にのみ動作。最新スキャン結果を地図画像上に描画し、OpenCV ウィンドウに表示する。
- `cast_ray()`: 画像座標上で一定ステップ進みながら障害物を検出する内部メソッド。閾値より暗い画素を障害物とみなし、ヒット距離を返す。

---

## 8. クラス構成
- `LaserScanSimulatorNode`: Node 継承クラス。購読／タイマ登録、画像読み込み、QoS 設定を担う。
- ユーティリティ関数 `world_to_image()`、`image_to_world()` を備え、座標変換を分離して保守性を高めている。
- レイキャスト処理は `_cast_rays()` と `cast_ray()` に分離し、テストで個別検証できるようにしている。

---

## 9. QoS設計
- `/scan` publisher は BestEffort／Volatile／depth=1 を想定。高頻度更新で最新値のみを利用することを前提とする。
- `/amcl_pose` 購読は RELIABLE／Volatile／depth=10 とし、姿勢更新の欠落を防ぐ。

---

## 10. エラー処理・ログ方針
- 地図画像読み込み失敗時は ERROR ログを出し例外を送出してノード起動を停止する。
- `/amcl_pose` 受信前にスキャンを要求された場合は WARN を出して無視する。
- OpenCV ウィンドウ描画が失敗した場合は ERROR を出し、`enable_debug_view` を False に切り替えて継続する。

---

## 11. Phase3 拡張想定
- 動的障害物やノイズモデルを導入し、実センサに近い挙動を再現する。
- `/scan` 以外の補助トピック（例：可視化用 Marker）を追加し、デバッグ性を向上させる。
- 地図のオンライン切り替えや外部サービス経由での再読み込み機能を実装する。

---

## 12. テスト観点

| テスト種別 | 目的 | 検証項目 |
|-------------|------|----------|
| ユニットテスト | レイキャスト検証 | `cast_ray()` の境界条件、障害物閾値、`ray_step_m` による距離計算精度 |
| コンポーネントテスト | ノード単体検証 | `/scan` 出力レート、`enable_debug_view` オプション動作、姿勢更新の遅延時挙動 |
| 結合テスト | ObstacleMonitor 連携 | 生成した `/scan` が ObstacleMonitor で正しく解析され、ヒントが生成されること |
| シナリオテスト | システム試験 | 静的地図上の複数障害物配置でのスキャン確認、地図解像度変更時の挙動 |

---

## 13. まとめ
- LaserScanSimulatorNode は画像地図を基に高速に LaserScan を合成し、ObstacleMonitorNode や RobotNavigator の開発検証を支援する。
- パラメータや QoS、エラー処理を整理することで、単独文書として参照しやすくした。
- 本書と ObstacleMonitorNode の文書を併読することで、障害物検知系全体の仕様を統一粒度で理解できる。

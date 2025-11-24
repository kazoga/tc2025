# 日本語文書
# route_blockage_detector launch 時の挙動差分調査メモ

## 1. 事象の概要
- `ros2 run yolo_detector route_blockage_detector --ros-args --params-file ...` では仮封鎖→解除などの状態遷移が期待通りに動作する。
- `ros2 launch yolo_detector route_blockage_detector.launch.py` を引数無しで使うと、一度仮封鎖になると解除や抑止ロジックが機能せず、状態が停滞する。
- `ros2 launch yolo_detector yolo_ncnn_with_route_blockage.launch.py` では `route_blockage_detector` のコンソールログが画面に流れないように見える。

## 2. 実装から読み取れる要因
### 2.1 位置取得に失敗した場合の早期 return
`_detections_callback` は TF もしくは `/amcl_pose` から現在位置を取得できなかった場合、警告を出した上でカウントだけ記録し、以降の判定・解除処理をすべてスキップする。仮封鎖済みの状態では `_evaluate_decision` や `_maybe_clear_blocked_state` が呼ばれないため、`road_blocked` を解除する機会自体が失われる。【F:yolo_detector/route_blockage_detector_node.py†L102-L119】【F:yolo_detector/route_blockage_detector_node.py†L267-L301】【F:yolo_detector/route_blockage_detector_node.py†L337-L349】

### 2.2 launch ファイルと時刻源の不整合
- （修正前の課題）`route_blockage_detector.launch.py` はパラメータファイルのみを渡し、`use_sim_time` は設定しない。
- （修正前の課題）`yolo_ncnn_with_route_blockage.launch.py` では `use_sim_time` 引数を用意するものの、既定値は `false` のまま渡される。

シミュレーション環境や rosbag 再生では `/clock` を用いたシミュレーション時間で TF が流れる一方、`route_blockage_detector` は実時間のまま `lookup_transform('map', 'base_link', stamp)` を呼ぶ。この時間軸の食い違いで TF 取得に失敗すると、前述の早期 return に入り、仮封鎖解除や多重検知抑止が動かなくなる。

### 2.3 ログが画面に出ないように見える理由
`yolo_ncnn_with_route_blockage.launch.py` では `output='screen'` を指定しているため、本来は rclpy ログが標準出力へ流れる設計である。【F:launch/yolo_ncnn_with_route_blockage.launch.py†L37-L74】
しかし TF 取得失敗でコールバックが即時 return してしまうと、起動時の INFO 以降は条件付きログ（状態遷移時のみ）が発生しない。結果として「ログが出ていない」ように見えるが、実際には状態遷移まで到達していないことが原因と考えられる。

## 3. まとめ
- launch で `use_sim_time` を指定しない／既定値のままにすると、メッセージヘッダーのシミュレーション時刻とノードの時計がずれ、`map -> base_link` の TF 取得に失敗しやすい。
- 位置取得に失敗すると判定処理がすべてスキップされる実装のため、仮封鎖解除や多重検知抑止が動かなくなり、`road_blocked` が張り付く。
- 同じ理由で状態遷移ログも発生せず、複合 launch では route_blockage_detector のログが表示されないように見える。

## 4. 対応方針と修正内容
- `use_sim_time` を各 launch で明示し、既定値を `true` に変更して `/clock` を利用した時間軸と揃える。【F:launch/route_blockage_detector.launch.py†L10-L27】【F:launch/yolo_with_route_blockage.launch.py†L10-L44】【F:launch/yolo_ncnn_with_route_blockage.launch.py†L10-L74】
- `route_blockage_detector` だけでなく、タイマ駆動で推論を行う `yolo_node` にも `use_sim_time` を渡し、シミュレーション環境での挙動を安定させる。【F:launch/yolo_with_route_blockage.launch.py†L18-L44】

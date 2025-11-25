# ROS1版 Waypoint 可視化・編集手順

## 概要
`ros1_src/scripts` には RViz でウェイポイントを表示する `waypoint_map_view.py` と、RViz のゴール指定を使って CSV を編集する `waypoint_gui.py` が用意されています。以下では、ROS1 Noetic + catkin 環境で両スクリプトを組み合わせて運用するための手順をまとめます。

## 事前準備
- ROS Master を起動しておく。
  - 例: `roscore`
- `tc2025` パッケージを catkin ワークスペースでビルド・環境変数を反映済みであること。
  - 例: `source ~/catkin_ws/devel/setup.bash`
- RViz を使用できる環境（ディスプレイ転送設定を含む）を整えておく。

## Waypoint の可視化 (`waypoint_map_view.py`)
1. RViz で Marker を受信するため、`waypoint_map_view.py` を起動する。
   ```bash
   rosrun tc2025 waypoint_map_view.py _waypoint:=/path/to/waypoint.csv
   ```
   - `~waypoint` パラメータで CSV パスを指定する。デフォルトは `/home/nakaba/map/waypoint_tsukuba2023_lio11.csv`。【F:ros1_src/scripts/waypoint_map_view.py†L23-L25】
2. ノードは 1 Hz で CSV を読み込み、各行を矢印 Marker として `/waypoint` に配信し、番号はテキスト Marker で付与する。【F:ros1_src/scripts/waypoint_map_view.py†L63-L194】
3. `/move_base/goal` を監視し、受信したゴール位置を赤い球体 Marker として `/goal_marker` に公開する。【F:ros1_src/scripts/waypoint_map_view.py†L28-L59】
4. RViz 側では以下を追加表示する。
   - 「Marker」または「MarkerArray」Display を追加し、`/waypoint` をフレーム `map` で表示。
   - ゴール位置を確認したい場合は `goal_marker` も同様に追加。

## Waypoint の編集 (`waypoint_gui.py`)
1. ゴール入力用に RViz を立ち上げ、`2D Nav Goal` ツールで `/move_base_simple/goal` を送れる状態にする。
2. 編集 GUI を起動する。
   ```bash
   rosrun tc2025 waypoint_gui.py
   ```
3. GUI で「Open」を押し、編集対象の waypoint CSV を選択する。【F:ros1_src/scripts/waypoint_gui.py†L44-L58】
4. 「Select NUM」に行番号を入力し「Set NUM」を押す。【F:ros1_src/scripts/waypoint_gui.py†L96-L135】
5. RViz で選択した番号の waypoint にしたい位置・姿勢を `2D Nav Goal` で送信すると、GUI が `/move_base_simple/goal` を受信して該当行の座標と姿勢（`q3`/`q4`）を書き換える。【F:ros1_src/scripts/waypoint_gui.py†L23-L135】
6. 上書き前に自動で `old_waypoint` ディレクトリへタイムスタンプ付きバックアップを作成し、ヘッダ付きで CSV を保存する。「Save CSV」を押して反映する。【F:ros1_src/scripts/waypoint_gui.py†L59-L95】

## 可視化と編集を併用する例
1. `roscore` を起動する。
2. 可視化ノードを CSV パス指定で起動する。
3. もう一つの端末で編集 GUI を起動する。
4. RViz を起動し、`/waypoint` と `/goal_marker` を表示。`2D Nav Goal` で更新したい位置を送信。
5. GUI のステータスで更新を確認したら「Save CSV」で保存。保存済み CSV を監視している可視化ノード側は 1 秒ごとに再読込するため、変更が即座に反映される。【F:ros1_src/scripts/waypoint_map_view.py†L63-L194】

これらの手順により、既存の ROS1 スクリプトだけで waypoint の可視化と CSV 直接編集を安全に行えます。

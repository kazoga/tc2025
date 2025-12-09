# lidar_obstacle_sim

Gazebo 上で LiDAR 障害物環境を再現するための簡易パッケージです。ロボットモデルに赤色のウェッジと青色の矢印を追加し、前方が視覚的に分かるようにしました。

## 使い方

```
ros2 launch lidar_obstacle_sim lidar_obstacle_sim.launch.py
```

`world` 引数を差し替えることで別の SDF ワールドを指定できます。

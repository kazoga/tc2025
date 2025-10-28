# Odometry速度積分とGPS融合によるリアルタイム軌跡推定

## 概要

このドキュメントは、ROSの`/Odometry`トピック（`nav_msgs/Odometry`型）と`/ublox/fix`トピック（`sensor_msgs/NavSatFix`型）を用いて、オドメトリから計算した速度を積分した軌跡とGPS軌跡を同時に可視化する方法について説明します。

## 背景

`/Odometry`トピックには、ロボットの位置情報（xyz座標）が含まれていますが、速度情報（vw）は0が格納されているケースがあります。このような場合、位置の時系列データから速度を数値微分により求め、それを積分することでロボットの移動軌跡を再構築できます。

さらに、GPS情報を重畳表示することで、オドメトリの累積誤差を視覚的に確認でき、次のステップであるセンサー融合の基礎データとなります。

## アルゴリズム

### 1. 速度の計算

Odometryメッセージから取得した連続する2つの位置 $(x_{t-1}, y_{t-1})$ と $(x_t, y_t)$、および時間差 $\Delta t$ を用いて：

#### 線速度 v

$$v = \frac{\sqrt{(x_t - x_{t-1})^2 + (y_t - y_{t-1})^2}}{\Delta t}$$

#### 移動方向角 θ

$$\theta_{movement} = \arctan2(y_t - y_{t-1}, x_t - x_{t-1})$$

#### 角速度 w

$$w = \frac{\theta_t - \theta_{t-1}}{\Delta t}$$

ただし、角度差は $-\pi$ から $\pi$ の範囲に正規化します。

### 2. 位置・姿勢の積分

計算した速度を用いて、ロボットの位置と姿勢を積分により更新します：

#### 姿勢の更新

$$\theta_{integrated} = \theta_{movement}$$

#### 位置の更新

$$x_{integrated} = x_{integrated} + v \cdot \cos(\theta_{integrated}) \cdot \Delta t$$
$$y_{integrated} = y_{integrated} + v \cdot \sin(\theta_{integrated}) \cdot \Delta t$$

### 3. GPS座標の変換

GPS情報（緯度・経度）をUTM座標系に変換し、基準点からの相対座標として表示します。

#### 基準点（つくばチャレンジ）
- 緯度: 36.0830041°
- 経度: 140.0763757°
- 高度: 73.594 m

#### UTM変換
つくば周辺は**UTM Zone 54N**に位置します。

```python
utm_x, utm_y = pyproj.Proj(proj='utm', zone=54, ellps='WGS84')(経度, 緯度)
相対X = utm_x - 基準点のutm_x
相対Y = utm_y - 基準点のutm_y
```

### 4. オドメトリとGPSのアライメント

オドメトリは局所的に正確ですが、独自の座標系を持ちます。GPSはグローバルな座標系です。この2つを同じ座標系で表示するために、以下のアライメント処理を行います。

#### アライメント手順（改良版）

1. **データ収集条件**:
   - オドメトリの速度が **1.0 m/s以上** の時のみデータを収集
   - GPS更新ごとに方位差を計算

2. **GPS更新ごとの方位差計算**:
   - GPS前回位置と現在位置から方位を計算: `θ_gps = atan2(Δy_gps, Δx_gps)`
   - オドメトリの現在方位: `θ_odom`
   - 方位差: `Δθ = θ_gps - θ_odom`（-πからπに正規化）

3. **方位差の蓄積**:
   - 方位差を累積距離とともに記録
   - **累積距離が2m以上**になるまで蓄積

4. **平均方位差の計算**:
   - 蓄積した方位差の平均を計算: `θ_offset = mean(Δθ_list)`
   - 標準偏差も計算して精度を確認

5. **座標変換**:
   - 回転: オドメトリ座標を `θ_offset` だけ回転
   - 平行移動: GPS初期位置に合わせる

```python
# 回転行列
x' = x * cos(θ_offset) - y * sin(θ_offset)
y' = x * sin(θ_offset) + y * cos(θ_offset)

# 平行移動
x_aligned = x' + gps_x_initial
y_aligned = y' + gps_y_initial
```

#### この手法の利点

- **より正確**: 単一のサンプルではなく、複数の方位差の平均を使用
- **ノイズに強い**: GPS更新ごとに計算するため、外れ値の影響を軽減
- **移動中のみ**: 速度が1m/s以上の時のみ使用するため、静止時やGPS誤差の影響を回避
- **統計的信頼性**: 標準偏差を確認することで推定精度を評価可能

この処理により、オドメトリとGPSの軌跡がほぼピタリと重なります。

### 5. 座標系

- **UTM座標系**: 地理座標系（緯度経度）を平面直交座標系に投影したもの
- **方位**: 真東を0度（0ラジアン）とする
  - 東方向: θ = 0°
  - 北方向: θ = 90°
  - 西方向: θ = 180°（または -180°）
  - 南方向: θ = -90°（または 270°）
- **基準点**: (0, 0) は上記のGPS基準点を示す
- **アライメント**: オドメトリはGPS座標系に自動的に変換されて表示される

## スクリプト

### odom_velocity_integrator.py

#### 機能

1. `/Odometry`トピックを購読
2. `/ublox/fix`トピック（GPS）を購読
3. xy座標から速度（v）と角速度（w）を計算
4. 速度を積分して軌跡を再構築
5. GPS座標をUTM座標系に変換
6. **移動中（v≥1m/s）にGPS更新ごとに方位差を計算**し、2m分蓄積して平均を取得
7. **オドメトリをGPS座標系に自動アライン**（回転・平行移動）
8. 以下をリアルタイムでプロット：
   - 積分された軌跡（青線、GPS座標系にアライン済み）とGPS軌跡（緑線）を重畳表示
   - 現在位置（オドメトリ: 赤点、GPS: マゼンタ点）
   - ロボットの方位（赤い矢印）
   - 基準点（黒い星印）
9. **全軌跡が常に見えるように自動スケール調整**（範囲は拡大のみ、縮小しない）

#### 依存ライブラリ

```bash
# pyprojライブラリをインストール（UTM座標変換に必要）
pip3 install pyproj
```

#### 使用方法

```bash
# ROSマスターが起動していることを確認
roscore

# 別のターミナルでトピックがパブリッシュされていることを確認
rostopic echo /Odometry
rostopic echo /ublox/fix

# スクリプトを実行
rosrun <package_name> odom_velocity_integrator.py
# または
python3 /home/nkb/ros/tc2025/ros1_src/scripts/odom_velocity_integrator.py
```

#### 主要なパラメータ

- **履歴保存数**: 1000サンプル（約100秒分、10Hzの場合）
- **更新レート**: 10 Hz
- **矢印長さ**: 2.0メートル（方位表示用）

#### 出力例

```
[INFO] Odometry Velocity Integrator with GPS initialized.
[INFO] GPS Reference: lat=36.0830041, lon=140.0763757, alt=73.594
[INFO] Reference UTM coordinates: x=345678.123, y=3995432.456
[INFO] Initial heading: East (0 degrees)
[INFO] Coordinate system: UTM (local)
[INFO] Initialized at position: (0.123, 0.456)
[INFO] GPS initialized at: (1.234, -0.567) relative to reference
[INFO] Heading sample 1: GPS=87.3°, Odom=2.1°, Diff=85.2°, Distance=0.45m
[INFO] Heading sample 2: GPS=88.1°, Odom=3.0°, Diff=85.1°, Distance=0.92m
[INFO] Heading sample 3: GPS=86.9°, Odom=1.8°, Diff=85.1°, Distance=1.38m
[INFO] Heading sample 4: GPS=87.5°, Odom=2.5°, Diff=85.0°, Distance=1.84m
[INFO] Heading sample 5: GPS=87.2°, Odom=2.1°, Diff=85.1°, Distance=2.31m
[INFO] === Odometry-GPS Alignment Complete ===
[INFO]   Samples used: 5
[INFO]   Distance accumulated: 2.31m
[INFO]   Heading offset (average): 85.1°
[INFO]   Heading offset (std dev): 0.08°
[INFO]   Position offset: (1.234, -0.567)
[INFO]   Odometry trajectory will now be aligned with GPS
[INFO] v=1.234 m/s, w=0.123 rad/s, pos=(5.678, 2.345), theta=45.6°
```

## プロットの見方

### 軌跡プロット

- **青い線**: オドメトリから積分した軌跡（GPS座標系にアライン済み、局所的に精度が高い）
- **緑の線**: GPSの軌跡（グローバルに安定）
- **重なり具合**: 正常にアライメントされると、両者はほぼピタリと重なる
- **赤い点**: 現在のロボット位置（オドメトリ）
- **マゼンタの点**: 現在のロボット位置（GPS）
- **赤い矢印**: ロボットの現在の方位（矢印の向きが進行方向）
- **黒い星**: GPS基準点（原点）
- **X軸**: 東方向（メートル、基準点からの相対距離）
- **Y軸**: 北方向（メートル、基準点からの相対距離）
- **スケール**: 全軌跡が常に表示されるように自動調整（範囲は拡大のみ、縮小しない）

## オドメトリとGPSの違い

### オドメトリ（青線）の特徴
- **長所**:
  - 高周波数で更新される（10Hz以上）
  - 局所的な動きを正確に捉える
  - 滑らかな軌跡
  - 短時間・短距離では非常に正確
- **短所**:
  - 時間とともに誤差が累積
  - 独自の座標系を持つ（グローバル位置が不明）

### GPS（緑線）の特徴
- **長所**:
  - グローバルな位置が正確
  - 累積誤差がない
  - 長時間の走行でも安定
- **短所**:
  - 更新頻度が低い（1Hz程度）
  - マルチパスやビル影響で精度低下
  - 軌跡が不連続になりやすい

### アライメント後の効果
- GPS初期方位を使ってオドメトリを補正することで、**両者がピタリと重なる**
- オドメトリの高精度な局所軌跡と、GPSのグローバル位置を統合
- 時間経過によるドリフトを視覚的に確認可能

## 注意事項

### 1. 累積誤差

オドメトリは速度の数値微分と積分を繰り返すため、時間経過とともに誤差が累積します。特に：
- 低速時のノイズの影響
- サンプリングレートの影響
- センサーの精度

GPS軌跡と比較することで、この累積誤差を視覚的に確認できます。

### 2. 初期化とアライメント

- **オドメトリ**: スクリプト起動時の位置を原点とし、初期方位を真東(0°)とします
- **GPS**: 基準点（lat=36.0830041, lon=140.0763757）を原点(0, 0)とします
- **アライメント条件**:
  - オドメトリの速度が **1.0 m/s以上** の時のみデータを収集
  - GPS更新ごとに方位差を計算
  - 累積距離が **2m以上** になると自動的に実行されます
- **重要**:
  - 屋外でGPS信号を受信できる環境で実行してください
  - 速度1m/s以上で2m以上直進してください（カーブは避ける）

### 3. 座標系の統一

GPS座標（緯度・経度）をUTM座標系に変換し、オドメトリをGPS座標系に回転・平行移動することで、両者を同じ座標系で可視化しています。

## 次のステップ

このスクリプトは、以下を実現しています：

1. ✅ Odometryのxy座標から速度を計算
2. ✅ 速度を積分して軌跡を再構築
3. ✅ GPS情報をUTM座標系に変換
4. ✅ 移動中（v≥1m/s）にGPS更新ごとに方位差を計算
5. ✅ 2m分の方位差を蓄積して平均を計算（ノイズに強い）
6. ✅ オドメトリをGPS座標系に自動アライメント（回転・平行移動）
7. ✅ オドメトリとGPSの軌跡を重畳表示（ほぼピタリと重なる）
8. ✅ 全体が見えるように自動スケール調整
9. ✅ リアルタイムプロット

次のステップでは：
- **センサー融合**:
  - Extended Kalman Filter (EKF) の実装
  - Unscented Kalman Filter (UKF) の検討
  - robot_localization パッケージの活用
- **軌跡の評価**:
  - 誤差の定量的評価（RMSE、最大誤差など）
  - ドリフト量の計測
  - 時系列での誤差増加の可視化

## 参考

### ROSメッセージ型

#### nav_msgs/Odometry
```
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
    geometry_msgs/Vector3 angular
  float64[36] covariance
```

#### sensor_msgs/NavSatFix
```
std_msgs/Header header
sensor_msgs/NavSatStatus status
  int8 status
  uint16 service
float64 latitude   # 緯度（度）
float64 longitude  # 経度（度）
float64 altitude   # 高度（メートル）
float64[9] position_covariance
uint8 position_covariance_type
```

### 関連コマンド

```bash
# トピックの情報を確認
rostopic info /Odometry
rostopic info /ublox/fix

# メッセージの内容を表示
rostopic echo /Odometry
rostopic echo /ublox/fix

# パブリッシュレートを確認
rostopic hz /Odometry
rostopic hz /ublox/fix

# トピックのグラフ構造を可視化
rqt_graph

# GPS座標を確認
rostopic echo /ublox/fix | grep -E "latitude|longitude|altitude"
```

## トラブルシューティング

### pyprojがインストールされていない

```bash
pip3 install pyproj
# または
sudo apt install python3-pyproj
```

### プロットウィンドウが表示されない

- X11転送が有効か確認: `echo $DISPLAY`
- matplotlibのバックエンドを確認: `python3 -c "import matplotlib; print(matplotlib.get_backend())"`

### オドメトリ軌跡が描画されない

- Odometryトピックがパブリッシュされているか確認
- 座標値が変化しているか確認（`rostopic echo /Odometry`）

### GPS軌跡が描画されない

- `/ublox/fix`トピックがパブリッシュされているか確認
- GPS信号を受信しているか確認（屋外で実行）
- `rostopic echo /ublox/fix`でlatitude/longitudeに有効な値が入っているか確認

### オドメトリとGPSの軌跡が大きくずれている

考えられる原因：
1. **アライメント未実行**: 累積距離が2m未満
   - 解決策: 屋外でロボットを速度1m/s以上で2m以上直進させる
2. **速度不足**: オドメトリの速度が1m/s未満
   - 解決策: もっと速く走行する
3. **GPS信号不良**: GPS位置の精度が低い
   - 解決策: GPS受信状態が良好な場所（空が開けた場所）で実行する
4. **カーブ走行**: 直進していない
   - 解決策: できるだけ直進で走行する（カーブ時の方位差は正確ではない）

正常にアライメントされると、ログに以下が表示されます：
```
[INFO] Heading sample 1: GPS=87.3°, Odom=2.1°, Diff=85.2°, Distance=0.45m
[INFO] Heading sample 2: GPS=88.1°, Odom=3.0°, Diff=85.1°, Distance=0.92m
...
[INFO] === Odometry-GPS Alignment Complete ===
[INFO]   Samples used: 5
[INFO]   Heading offset (average): 85.1°
[INFO]   Heading offset (std dev): 0.08°
```

標準偏差が小さいほど（例: 0.1°未満）、アライメントの精度が高いことを示します。

### 速度が異常に大きい/小さい

- サンプリングレート（dt）を確認
- Odometryの単位を確認（通常はメートル）
- センサーのキャリブレーションを確認

### スケールが頻繁に変わる

全体が見えるように自動調整しているため、データの範囲が変わるとスケールも変わります。これは正常な動作です。

## ライセンス

このスクリプトは、つくばチャレンジ2025プロジェクトの一部です。

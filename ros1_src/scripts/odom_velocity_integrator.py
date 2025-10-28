#!/usr/bin/env python3
# coding=utf-8

"""
Odometry Velocity Integrator with GPS Fusion
オドメトリのxy座標から速度(v)と角速度(w)を計算し、
それを積分してリアルタイムでUTM座標系にプロットするノード
GPS情報も重畳表示する
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import math
from collections import deque
import pyproj

class OdomVelocityIntegrator:
    def __init__(self):
        rospy.init_node('odom_velocity_integrator', anonymous=False)

        # GPS基準位置（つくばチャレンジ）
        self.ref_lat = 36.0830041
        self.ref_lon = 140.0763757
        self.ref_alt = 73.594

        # UTM変換器の設定
        self.setup_utm_converter()

        # パラメータ
        self.dt = 0.0  # 時間差分（自動計算）
        self.prev_time = None

        # 前回の位置
        self.prev_x = None
        self.prev_y = None

        # 積分により推定された位置と姿勢
        # 初期方位は真東（0度）だが、GPS方位で補正される
        self.integrated_x = 0.0
        self.integrated_y = 0.0
        self.integrated_theta = 0.0  # 真東を0度とする

        # GPS方位推定用のデータ
        self.prev_gps_x = None
        self.prev_gps_y = None
        self.heading_differences = []  # 方位差のリスト
        self.heading_diff_distances = []  # 各方位差を取得した時の累積距離
        self.total_alignment_distance = 0.0  # アライメント用の累積距離
        self.gps_heading_estimated = False

        # 現在の速度（オドメトリから）
        self.current_velocity = 0.0

        # オドメトリのGPS座標系への変換パラメータ
        self.odom_to_gps_x_offset = 0.0
        self.odom_to_gps_y_offset = 0.0
        self.odom_to_gps_heading_offset = 0.0  # オドメトリ座標系とGPS座標系の方位差
        self.alignment_done = False
        self.just_aligned = False  # アライメント直後フラグ

        # GPS初期位置
        self.gps_initial_x = None
        self.gps_initial_y = None

        # 軌跡データを保存（UTM座標系）
        self.trajectory_x = deque(maxlen=30000)
        self.trajectory_y = deque(maxlen=30000)

        # GPS軌跡データを保存（UTM座標系）
        self.gps_trajectory_x = deque(maxlen=30000)
        self.gps_trajectory_y = deque(maxlen=30000)

        # 初期化フラグ
        self.initialized = False
        self.gps_initialized = False

        # サブスクライバー
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)

        # プロット設定
        self.setup_plot()

        rospy.loginfo("Odometry Velocity Integrator with GPS initialized.")
        rospy.loginfo(f"GPS Reference: lat={self.ref_lat}, lon={self.ref_lon}, alt={self.ref_alt}")
        rospy.loginfo("Initial heading: East (0 degrees)")
        rospy.loginfo("Coordinate system: UTM (local)")

    def setup_utm_converter(self):
        """UTM座標変換の設定"""
        # WGS84からUTM座標系への変換器
        # 日本のつくば周辺はUTM Zone 54N
        self.utm_proj = pyproj.Proj(proj='utm', zone=54, ellps='WGS84', south=False)

        # 基準点のUTM座標を計算
        self.ref_utm_x, self.ref_utm_y = self.utm_proj(self.ref_lon, self.ref_lat)
        rospy.loginfo(f"Reference UTM coordinates: x={self.ref_utm_x:.3f}, y={self.ref_utm_y:.3f}")

    def setup_plot(self):
        """matplotlibのリアルタイムプロット設定"""
        plt.ion()
        self.fig, self.ax1 = plt.subplots(1, 1, figsize=(12, 10))

        # 軌跡プロット（UTM座標系）
        self.ax1.set_xlabel('X [m] (East relative to reference)', fontsize=12)
        self.ax1.set_ylabel('Y [m] (North relative to reference)', fontsize=12)
        self.ax1.set_title('Odometry (GPS-aligned) + GPS Trajectories (UTM Coordinate System)', fontsize=14)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal', adjustable='box')

        # オドメトリ積分軌跡（GPS座標系にアライン）
        self.line_traj, = self.ax1.plot([], [], 'b-', linewidth=2, label='Odometry (GPS-aligned)', alpha=0.7)
        self.point_current, = self.ax1.plot([], [], 'ro', markersize=10, label='Current position (Odom)')

        # GPS軌跡
        self.line_gps, = self.ax1.plot([], [], 'g-', linewidth=2, label='GPS trajectory', alpha=0.7)
        self.point_gps_current, = self.ax1.plot([], [], 'mo', markersize=10, label='Current position (GPS)')

        # 基準点を表示
        self.ax1.plot([0], [0], 'k*', markersize=15, label='GPS reference point', zorder=10)

        self.ax1.legend(loc='upper right', fontsize=10)

        # 方位を示す矢印
        self.arrow = None

        plt.tight_layout()

    def align_odometry_to_gps(self):
        """オドメトリ座標系をGPS座標系に合わせる"""
        if len(self.heading_differences) < 3:
            rospy.logwarn("Not enough heading difference samples for alignment")
            return

        # 方位差の平均を計算
        avg_heading_offset = np.mean(self.heading_differences)
        std_heading_offset = np.std(self.heading_differences)

        self.odom_to_gps_heading_offset = avg_heading_offset

        # 位置オフセットを計算（GPS初期位置に合わせる）
        if self.gps_initial_x is not None and self.gps_initial_y is not None:
            self.odom_to_gps_x_offset = self.gps_initial_x
            self.odom_to_gps_y_offset = self.gps_initial_y
        else:
            self.odom_to_gps_x_offset = 0.0
            self.odom_to_gps_y_offset = 0.0

        self.alignment_done = True
        self.gps_heading_estimated = True
        self.just_aligned = True  # 軸範囲リセット用フラグ

        rospy.loginfo("=== Odometry-GPS Alignment Complete ===")
        rospy.loginfo(f"  Samples used: {len(self.heading_differences)}")
        rospy.loginfo(f"  Distance accumulated: {self.total_alignment_distance:.2f}m")
        rospy.loginfo(f"  Heading offset (average): {math.degrees(avg_heading_offset):.1f}°")
        rospy.loginfo(f"  Heading offset (std dev): {math.degrees(std_heading_offset):.2f}°")
        rospy.loginfo(f"  Position offset: ({self.odom_to_gps_x_offset:.3f}, {self.odom_to_gps_y_offset:.3f})")
        rospy.loginfo("  Odometry trajectory will now be aligned with GPS")

    def transform_odom_to_gps(self, odom_x, odom_y):
        """オドメトリ座標をGPS座標系に変換"""
        if not self.alignment_done:
            return odom_x, odom_y

        # 回転行列を適用
        cos_theta = math.cos(self.odom_to_gps_heading_offset)
        sin_theta = math.sin(self.odom_to_gps_heading_offset)

        # 回転
        rotated_x = odom_x * cos_theta - odom_y * sin_theta
        rotated_y = odom_x * sin_theta + odom_y * cos_theta

        # 平行移動
        transformed_x = rotated_x + self.odom_to_gps_x_offset
        transformed_y = rotated_y + self.odom_to_gps_y_offset

        return transformed_x, transformed_y

    def gps_callback(self, msg):
        """GPSメッセージのコールバック"""
        # GPS座標をUTM座標に変換
        utm_x, utm_y = self.utm_proj(msg.longitude, msg.latitude)

        # 基準点からの相対座標に変換
        rel_x = utm_x - self.ref_utm_x
        rel_y = utm_y - self.ref_utm_y

        # GPS初期化
        if not self.gps_initialized:
            self.gps_initialized = True
            self.gps_initial_x = rel_x
            self.gps_initial_y = rel_y
            self.prev_gps_x = rel_x
            self.prev_gps_y = rel_y
            rospy.loginfo(f"GPS initialized at: ({rel_x:.3f}, {rel_y:.3f}) relative to reference")

        # GPS方位推定のためのデータ収集（速度が1m/s以上の時のみ）
        if not self.gps_heading_estimated and self.prev_gps_x is not None:
            # オドメトリの現在速度をチェック
            if self.current_velocity >= 1.0:
                # GPS前回位置との差分
                dx_gps = rel_x - self.prev_gps_x
                dy_gps = rel_y - self.prev_gps_y
                gps_distance = math.sqrt(dx_gps**2 + dy_gps**2)

                # GPS更新があった場合（位置が変わった）
                if gps_distance > 0.1:  # 10cm以上移動
                    # GPSから方位を計算
                    gps_heading = math.atan2(dy_gps, dx_gps)

                    # オドメトリの現在方位
                    odom_heading = self.integrated_theta

                    # 方位差を計算
                    heading_diff = gps_heading - odom_heading
                    # -πからπに正規化
                    heading_diff = math.atan2(math.sin(heading_diff), math.cos(heading_diff))

                    # 方位差を記録
                    self.heading_differences.append(heading_diff)
                    self.heading_diff_distances.append(self.total_alignment_distance)
                    self.total_alignment_distance += gps_distance

                    rospy.loginfo(f"Heading sample {len(self.heading_differences)}: "
                                f"GPS={math.degrees(gps_heading):.1f}°, "
                                f"Odom={math.degrees(odom_heading):.1f}°, "
                                f"Diff={math.degrees(heading_diff):.1f}°, "
                                f"Distance={self.total_alignment_distance:.2f}m")

                    # 2m以上データを蓄積したらアライメント実行
                    if self.total_alignment_distance >= 2.0:
                        self.align_odometry_to_gps()

        # GPS外れ値検出（急激なジャンプを無視）
        if len(self.gps_trajectory_x) > 0:
            last_x = self.gps_trajectory_x[-1]
            last_y = self.gps_trajectory_y[-1]
            jump = math.sqrt((rel_x - last_x)**2 + (rel_y - last_y)**2)
            if jump > 100.0:  # 100m以上の急激なジャンプは無視
                rospy.logwarn(f"GPS jump detected: {jump:.1f}m - ignoring this sample")
                return

        # 前回のGPS位置を更新
        self.prev_gps_x = rel_x
        self.prev_gps_y = rel_y

        # GPS軌跡に追加
        self.gps_trajectory_x.append(rel_x)
        self.gps_trajectory_y.append(rel_y)

    def odom_callback(self, msg):
        """Odometryメッセージのコールバック"""
        current_time = msg.header.stamp.to_sec()
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # 初回は前回の位置として保存するのみ
        if not self.initialized:
            self.prev_x = current_x
            self.prev_y = current_y
            self.prev_time = current_time
            self.initialized = True

            # 初期位置を軌跡に追加
            self.trajectory_x.append(self.integrated_x)
            self.trajectory_y.append(self.integrated_y)

            rospy.loginfo(f"Initialized at position: ({current_x:.3f}, {current_y:.3f})")
            return

        # 時間差分を計算
        self.dt = current_time - self.prev_time

        # dtが0または負の場合はスキップ
        if self.dt <= 0.0:
            rospy.logwarn(f"Invalid dt: {self.dt}, skipping this message.")
            return

        # 位置の差分を計算
        dx = current_x - self.prev_x
        dy = current_y - self.prev_y

        # 線速度 v を計算（2次元距離の変化率）
        v = math.sqrt(dx**2 + dy**2) / self.dt

        # 現在速度を更新（GPS方位推定で使用）
        self.current_velocity = v

        # 移動方向の角度を計算
        movement_theta = math.atan2(dy, dx)

        # 角速度 w を計算（方位の変化率）
        # 前回の姿勢との差分
        dtheta = movement_theta - self.integrated_theta

        # 角度を-piからpiの範囲に正規化
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

        w = dtheta / self.dt

        # 速度を積分して位置と姿勢を更新
        # 姿勢を更新
        self.integrated_theta = movement_theta

        # 位置を更新（速度ベクトルを積分）
        self.integrated_x += v * math.cos(self.integrated_theta) * self.dt
        self.integrated_y += v * math.sin(self.integrated_theta) * self.dt

        # 軌跡に追加
        self.trajectory_x.append(self.integrated_x)
        self.trajectory_y.append(self.integrated_y)

        # ログ出力（定期的に、5秒ごとに概算）
        # 簡易的なカウンター（time_historyがないので軌跡長で代用）
        if len(self.trajectory_x) % 50 == 0 and len(self.trajectory_x) > 0:
            rospy.loginfo(f"v={v:.3f} m/s, w={w:.3f} rad/s, "
                         f"pos=({self.integrated_x:.3f}, {self.integrated_y:.3f}), "
                         f"theta={math.degrees(self.integrated_theta):.1f}°")

        # 次の計算のために現在値を保存
        self.prev_x = current_x
        self.prev_y = current_y
        self.prev_time = current_time

    def update_plot(self):
        """プロットを更新"""
        if len(self.trajectory_x) < 2 and len(self.gps_trajectory_x) < 2:
            return

        # 変換後の軌跡と現在位置を保持する変数
        transformed_traj_x = []
        transformed_traj_y = []
        current_x = 0.0
        current_y = 0.0

        # オドメトリ軌跡プロット（GPS座標系に変換）
        if len(self.trajectory_x) > 0:
            # dequeのコピーを作成してから反復（mutation errorを回避）
            traj_x_list = list(self.trajectory_x)
            traj_y_list = list(self.trajectory_y)

            # 全軌跡を変換
            for x, y in zip(traj_x_list, traj_y_list):
                tx, ty = self.transform_odom_to_gps(x, y)
                transformed_traj_x.append(tx)
                transformed_traj_y.append(ty)

            self.line_traj.set_data(transformed_traj_x, transformed_traj_y)

            # 現在位置を変換
            current_x, current_y = self.transform_odom_to_gps(self.integrated_x, self.integrated_y)
            self.point_current.set_data([current_x], [current_y])

            # 方位を示す矢印を更新（方位も変換）
            if self.arrow is not None:
                self.arrow.remove()

            arrow_length = 2.0  # 矢印の長さ（メートル）
            # 方位もGPS座標系に変換
            transformed_theta = self.integrated_theta + self.odom_to_gps_heading_offset
            dx_arrow = arrow_length * math.cos(transformed_theta)
            dy_arrow = arrow_length * math.sin(transformed_theta)
            self.arrow = self.ax1.arrow(current_x, current_y,
                                         dx_arrow, dy_arrow,
                                         head_width=0.5, head_length=0.5,
                                         fc='red', ec='red', alpha=0.7)

        # GPS軌跡プロット
        gps_x_list = []
        gps_y_list = []
        if len(self.gps_trajectory_x) > 0:
            gps_x_list = list(self.gps_trajectory_x)
            gps_y_list = list(self.gps_trajectory_y)
            self.line_gps.set_data(gps_x_list, gps_y_list)
            # 最新のGPS位置を表示
            self.point_gps_current.set_data([gps_x_list[-1]], [gps_y_list[-1]])

        # 軸の範囲を自動調整（毎フレーム再計算、縮小も許可）
        # 実際に表示されているデータから範囲を計算
        all_x = []
        all_y = []

        # オドメトリの変換後の座標を追加
        if len(transformed_traj_x) > 0:
            all_x.extend(transformed_traj_x)
            all_y.extend(transformed_traj_y)

        # GPS座標を追加
        if len(gps_x_list) > 0:
            all_x.extend(gps_x_list)
            all_y.extend(gps_y_list)

        # データの範囲を計算
        if len(all_x) > 0:
            x_min = min(all_x)
            x_max = max(all_x)
            y_min = min(all_y)
            y_max = max(all_y)

            x_range = max(x_max - x_min, 1e-6)  # ゼロ除算回避
            y_range = max(y_max - y_min, 1e-6)

            # 正方形の範囲を作成（set_aspect='equal'のため）
            # 中心点を計算
            cx = 0.5 * (x_min + x_max)
            cy = 0.5 * (y_min + y_max)

            # 最大範囲にマージンを追加
            max_range = max(x_range, y_range)
            margin = max(0.2 * max_range, 10.0)  # 20%、最小10m
            half = 0.5 * max_range + margin

            # 範囲を設定（毎フレーム再計算、縮小も許可）
            self.ax1.set_xlim(cx - half, cx + half)
            self.ax1.set_ylim(cy - half, cy + half)

            # ログ出力（デバッグ用、定期的に）
            if len(self.trajectory_x) % 50 == 0 and len(self.trajectory_x) > 0:
                rospy.loginfo(f"Plot range: X=[{cx-half:.1f}, {cx+half:.1f}], "
                             f"Y=[{cy-half:.1f}, {cy+half:.1f}]")
                rospy.loginfo(f"  Center: ({cx:.1f}, {cy:.1f}), Half-size: {half:.1f}m")
                rospy.loginfo(f"  Data: {len(all_x)} points, Current pos: ({current_x:.1f}, {current_y:.1f})")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run(self):
        """メインループ"""
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()

def main():
    try:
        integrator = OdomVelocityIntegrator()
        integrator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception, exiting...")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()

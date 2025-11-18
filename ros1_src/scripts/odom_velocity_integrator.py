#!/usr/bin/env python3
# coding=utf-8

"""
Odometry Velocity Integrator with GPS Fusion using Kalman Filter
オドメトリのxy座標から速度(v)と角速度(w)を計算し、
カルマンフィルタでGPSと融合してリアルタイムでUTM座標系にプロットするノード
tkinterのUIで誤差楕円パラメータを調整可能
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
import tkinter as tk
from tkinter import ttk
import threading

class ExtendedKalmanFilter:
    """
    拡張カルマンフィルタ (EKF) for GPS-Odometry Fusion
    状態ベクトル: [x, y, theta, v]
    - x, y: 位置 (UTM座標系)
    - theta: 方位角 (rad)
    - v: 速度 (m/s)
    """
    def __init__(self):
        # 状態ベクトル [x, y, theta, v]
        self.state = np.zeros(4)

        # 状態共分散行列 (初期は大きめに設定)
        self.P = np.eye(4) * 10.0

        # プロセスノイズ共分散行列 (調整可能)
        self.Q = np.diag([0.1, 0.1, 0.01, 0.5])  # [x, y, theta, v]

        # オドメトリ観測ノイズ共分散行列 (調整可能)
        # オドメトリは相対的な変化を観測
        self.R_odom = np.diag([0.5, 0.5, 0.1])  # [dx, dy, dtheta]

        # GPS観測ノイズ共分散行列 (調整可能)
        self.R_gps = np.diag([2.0, 2.0])  # [x, y]

        # 速度観測ノイズ (調整可能)
        self.R_vel = 0.2  # m^2/s^2

        self.initialized = False

    def initialize(self, x, y, theta=0.0, v=0.0):
        """初期化"""
        self.state = np.array([x, y, theta, v])
        self.initialized = True

    def predict(self, dt, w):
        """
        予測ステップ
        dt: 時間差分
        w: 角速度 (rad/s)
        """
        if not self.initialized or dt <= 0:
            return

        x, y, theta, v = self.state

        # 状態遷移モデル (運動モデル)
        # x_{k+1} = x_k + v_k * cos(theta_k) * dt
        # y_{k+1} = y_k + v_k * sin(theta_k) * dt
        # theta_{k+1} = theta_k + w * dt
        # v_{k+1} = v_k (速度は一定と仮定)

        x_new = x + v * math.cos(theta) * dt
        y_new = y + v * math.sin(theta) * dt
        theta_new = theta + w * dt
        # 角度を-πからπに正規化
        theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))
        v_new = v

        self.state = np.array([x_new, y_new, theta_new, v_new])

        # ヤコビ行列 F
        F = np.array([
            [1, 0, -v * math.sin(theta) * dt, math.cos(theta) * dt],
            [0, 1,  v * math.cos(theta) * dt, math.sin(theta) * dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # プロセスノイズをdtでスケーリング（サンプリング周期依存性を考慮）
        Q_dt = np.diag([
            self.Q[0, 0] * dt,
            self.Q[1, 1] * dt,
            self.Q[2, 2] * dt,
            self.Q[3, 3] * dt
        ])

        # 共分散行列の更新
        self.P = F @ self.P @ F.T + Q_dt

    def update_velocity(self, v_meas):
        """
        速度観測による更新ステップ
        v_meas: 測定された速度 (m/s)
        """
        if not self.initialized:
            return

        # 観測行列 H: 速度成分のみを観測
        H = np.array([[0, 0, 0, 1]])  # v を直接観測

        # 観測値
        z = np.array([v_meas])

        # 予測された観測値
        z_pred = np.array([self.state[3]])

        # イノベーション（観測残差）
        y_innov = z - z_pred

        # イノベーション共分散
        R_vel_matrix = np.array([[self.R_vel]])
        S = H @ self.P @ H.T + R_vel_matrix

        # カルマンゲイン
        K = self.P @ H.T @ np.linalg.inv(S)

        # 状態の更新
        self.state = self.state + K @ y_innov

        # 方位角を正規化
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))

        # 共分散行列の更新 (Joseph形で数値安定性向上)
        I = np.eye(4)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_vel_matrix @ K.T

    def update_gps(self, gps_x, gps_y):
        """
        GPSによる更新ステップ
        gps_x, gps_y: GPS位置 (UTM座標系)
        """
        if not self.initialized:
            return

        # 観測モデル: GPSは位置を直接観測
        # z = [x, y]
        z = np.array([gps_x, gps_y])

        # 予測された観測値
        z_pred = self.state[:2]

        # 観測行列 H
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # イノベーション（観測残差）
        y_innov = z - z_pred

        # イノベーション共分散
        S = H @ self.P @ H.T + self.R_gps

        # カルマンゲイン
        K = self.P @ H.T @ np.linalg.inv(S)

        # 状態の更新
        self.state = self.state + K @ y_innov

        # 方位角を正規化
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))

        # 共分散行列の更新
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P

    def get_state(self):
        """現在の状態を取得"""
        return self.state.copy()

    def get_covariance(self):
        """現在の共分散行列を取得"""
        return self.P.copy()

    def set_process_noise(self, q_x, q_y, q_theta, q_v):
        """プロセスノイズを設定"""
        self.Q = np.diag([q_x, q_y, q_theta, q_v])

    def set_gps_noise(self, r_x, r_y):
        """GPS観測ノイズを設定"""
        self.R_gps = np.diag([r_x, r_y])

    def set_odom_noise(self, r_dx, r_dy, r_dtheta):
        """オドメトリ観測ノイズを設定"""
        self.R_odom = np.diag([r_dx, r_dy, r_dtheta])


class KalmanFilterUI:
    """
    tkinterベースのカルマンフィルタパラメータ調整UI
    """
    def __init__(self, ekf):
        self.ekf = ekf
        self.root = None
        self.running = False
        self.use_gps = True  # GPS使用フラグ
        self.gps_just_turned_off = False  # GPS OFF直後フラグ

    def create_ui(self):
        """UIを作成（別スレッドで実行される）"""
        self.root = tk.Tk()
        self.root.title("Kalman Filter Parameter Tuning")
        self.root.geometry("400x600")

        # メインフレーム
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # GPS使用チェックボックス
        self.use_gps_var = tk.BooleanVar(value=True)
        gps_checkbox = ttk.Checkbutton(main_frame, text="Use GPS (uncheck for odometry-only mode)",
                                        variable=self.use_gps_var, command=self.toggle_gps)
        gps_checkbox.grid(row=0, column=0, columnspan=3, pady=10, sticky=tk.W)

        # プロセスノイズセクション
        ttk.Label(main_frame, text="Process Noise (Q)", font=('Arial', 12, 'bold')).grid(row=1, column=0, columnspan=2, pady=5)

        # Q_x
        ttk.Label(main_frame, text="Q_x (position):").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.q_x_var = tk.DoubleVar(value=0.1)
        self.q_x_scale = ttk.Scale(main_frame, from_=0.01, to=5.0, variable=self.q_x_var,
                                     orient=tk.HORIZONTAL, length=200, command=self.update_parameters)
        self.q_x_scale.grid(row=2, column=1, pady=2)
        self.q_x_label = ttk.Label(main_frame, text=f"{self.q_x_var.get():.2f}")
        self.q_x_label.grid(row=2, column=2, padx=5)

        # Q_y
        ttk.Label(main_frame, text="Q_y (position):").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.q_y_var = tk.DoubleVar(value=0.1)
        self.q_y_scale = ttk.Scale(main_frame, from_=0.01, to=5.0, variable=self.q_y_var,
                                     orient=tk.HORIZONTAL, length=200, command=self.update_parameters)
        self.q_y_scale.grid(row=3, column=1, pady=2)
        self.q_y_label = ttk.Label(main_frame, text=f"{self.q_y_var.get():.2f}")
        self.q_y_label.grid(row=3, column=2, padx=5)

        # Q_theta
        ttk.Label(main_frame, text="Q_theta (angle):").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.q_theta_var = tk.DoubleVar(value=0.01)
        self.q_theta_scale = ttk.Scale(main_frame, from_=0.001, to=0.5, variable=self.q_theta_var,
                                         orient=tk.HORIZONTAL, length=200, command=self.update_parameters)
        self.q_theta_scale.grid(row=4, column=1, pady=2)
        self.q_theta_label = ttk.Label(main_frame, text=f"{self.q_theta_var.get():.3f}")
        self.q_theta_label.grid(row=4, column=2, padx=5)

        # Q_v
        ttk.Label(main_frame, text="Q_v (velocity):").grid(row=5, column=0, sticky=tk.W, pady=2)
        self.q_v_var = tk.DoubleVar(value=0.5)
        self.q_v_scale = ttk.Scale(main_frame, from_=0.01, to=5.0, variable=self.q_v_var,
                                     orient=tk.HORIZONTAL, length=200, command=self.update_parameters)
        self.q_v_scale.grid(row=5, column=1, pady=2)
        self.q_v_label = ttk.Label(main_frame, text=f"{self.q_v_var.get():.2f}")
        self.q_v_label.grid(row=5, column=2, padx=5)

        # GPSノイズセクション
        ttk.Separator(main_frame, orient=tk.HORIZONTAL).grid(row=6, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        ttk.Label(main_frame, text="GPS Measurement Noise (R)", font=('Arial', 12, 'bold')).grid(row=7, column=0, columnspan=2, pady=5)

        # R_gps_x
        ttk.Label(main_frame, text="R_GPS_x:").grid(row=8, column=0, sticky=tk.W, pady=2)
        self.r_gps_x_var = tk.DoubleVar(value=2.0)
        self.r_gps_x_scale = ttk.Scale(main_frame, from_=0.1, to=50.0, variable=self.r_gps_x_var,
                                         orient=tk.HORIZONTAL, length=200, command=self.update_parameters)
        self.r_gps_x_scale.grid(row=8, column=1, pady=2)
        self.r_gps_x_label = ttk.Label(main_frame, text=f"{self.r_gps_x_var.get():.2f}")
        self.r_gps_x_label.grid(row=8, column=2, padx=5)

        # R_gps_y
        ttk.Label(main_frame, text="R_GPS_y:").grid(row=9, column=0, sticky=tk.W, pady=2)
        self.r_gps_y_var = tk.DoubleVar(value=2.0)
        self.r_gps_y_scale = ttk.Scale(main_frame, from_=0.1, to=50.0, variable=self.r_gps_y_var,
                                         orient=tk.HORIZONTAL, length=200, command=self.update_parameters)
        self.r_gps_y_scale.grid(row=9, column=1, pady=2)
        self.r_gps_y_label = ttk.Label(main_frame, text=f"{self.r_gps_y_var.get():.2f}")
        self.r_gps_y_label.grid(row=9, column=2, padx=5)

        # オドメトリノイズセクション
        ttk.Separator(main_frame, orient=tk.HORIZONTAL).grid(row=10, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        ttk.Label(main_frame, text="Odometry Measurement Noise (R)", font=('Arial', 12, 'bold')).grid(row=11, column=0, columnspan=2, pady=5)

        # R_odom_dx
        ttk.Label(main_frame, text="R_Odom_dx:").grid(row=12, column=0, sticky=tk.W, pady=2)
        self.r_odom_dx_var = tk.DoubleVar(value=0.5)
        self.r_odom_dx_scale = ttk.Scale(main_frame, from_=0.01, to=5.0, variable=self.r_odom_dx_var,
                                           orient=tk.HORIZONTAL, length=200, command=self.update_parameters)
        self.r_odom_dx_scale.grid(row=12, column=1, pady=2)
        self.r_odom_dx_label = ttk.Label(main_frame, text=f"{self.r_odom_dx_var.get():.2f}")
        self.r_odom_dx_label.grid(row=12, column=2, padx=5)

        # R_odom_dy
        ttk.Label(main_frame, text="R_Odom_dy:").grid(row=13, column=0, sticky=tk.W, pady=2)
        self.r_odom_dy_var = tk.DoubleVar(value=0.5)
        self.r_odom_dy_scale = ttk.Scale(main_frame, from_=0.01, to=5.0, variable=self.r_odom_dy_var,
                                           orient=tk.HORIZONTAL, length=200, command=self.update_parameters)
        self.r_odom_dy_scale.grid(row=13, column=1, pady=2)
        self.r_odom_dy_label = ttk.Label(main_frame, text=f"{self.r_odom_dy_var.get():.2f}")
        self.r_odom_dy_label.grid(row=13, column=2, padx=5)

        # R_odom_dtheta
        ttk.Label(main_frame, text="R_Odom_dtheta:").grid(row=14, column=0, sticky=tk.W, pady=2)
        self.r_odom_dtheta_var = tk.DoubleVar(value=0.1)
        self.r_odom_dtheta_scale = ttk.Scale(main_frame, from_=0.01, to=1.0, variable=self.r_odom_dtheta_var,
                                               orient=tk.HORIZONTAL, length=200, command=self.update_parameters)
        self.r_odom_dtheta_scale.grid(row=14, column=1, pady=2)
        self.r_odom_dtheta_label = ttk.Label(main_frame, text=f"{self.r_odom_dtheta_var.get():.2f}")
        self.r_odom_dtheta_label.grid(row=14, column=2, padx=5)

        # リセットボタン
        ttk.Button(main_frame, text="Reset to Defaults", command=self.reset_parameters).grid(row=15, column=0, columnspan=3, pady=10)

        self.running = True
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def toggle_gps(self):
        """GPS使用/不使用を切り替え"""
        prev_use_gps = self.use_gps
        self.use_gps = self.use_gps_var.get()
        if self.use_gps:
            rospy.loginfo("GPS fusion ENABLED")
        else:
            rospy.loginfo("GPS fusion DISABLED - odometry-only mode")
            # GPS OFF直後フラグを立てる
            if prev_use_gps:
                self.gps_just_turned_off = True

    def update_parameters(self, *args):
        """スライダーの値が変更されたときに呼ばれる"""
        # ラベルの更新
        self.q_x_label.config(text=f"{self.q_x_var.get():.2f}")
        self.q_y_label.config(text=f"{self.q_y_var.get():.2f}")
        self.q_theta_label.config(text=f"{self.q_theta_var.get():.3f}")
        self.q_v_label.config(text=f"{self.q_v_var.get():.2f}")
        self.r_gps_x_label.config(text=f"{self.r_gps_x_var.get():.2f}")
        self.r_gps_y_label.config(text=f"{self.r_gps_y_var.get():.2f}")
        self.r_odom_dx_label.config(text=f"{self.r_odom_dx_var.get():.2f}")
        self.r_odom_dy_label.config(text=f"{self.r_odom_dy_var.get():.2f}")
        self.r_odom_dtheta_label.config(text=f"{self.r_odom_dtheta_var.get():.2f}")

        # カルマンフィルタのパラメータを更新
        self.ekf.set_process_noise(
            self.q_x_var.get(),
            self.q_y_var.get(),
            self.q_theta_var.get(),
            self.q_v_var.get()
        )
        self.ekf.set_gps_noise(
            self.r_gps_x_var.get(),
            self.r_gps_y_var.get()
        )
        self.ekf.set_odom_noise(
            self.r_odom_dx_var.get(),
            self.r_odom_dy_var.get(),
            self.r_odom_dtheta_var.get()
        )

    def reset_parameters(self):
        """パラメータをデフォルトに戻す"""
        self.q_x_var.set(0.1)
        self.q_y_var.set(0.1)
        self.q_theta_var.set(0.01)
        self.q_v_var.set(0.5)
        self.r_gps_x_var.set(2.0)
        self.r_gps_y_var.set(2.0)
        self.r_odom_dx_var.set(0.5)
        self.r_odom_dy_var.set(0.5)
        self.r_odom_dtheta_var.set(0.1)
        self.update_parameters()

    def on_closing(self):
        """ウィンドウを閉じる時の処理"""
        self.running = False
        if self.root:
            self.root.quit()
            self.root.destroy()

    def start(self):
        """UIを別スレッドで起動"""
        ui_thread = threading.Thread(target=self.create_ui, daemon=True)
        ui_thread.start()


class OdomVelocityIntegrator:
    def __init__(self):
        rospy.init_node('odom_velocity_integrator', anonymous=False)

        # GPS基準位置（つくばチャレンジ）
        self.ref_lat = 36.0830041
        self.ref_lon = 140.0763757
        self.ref_alt = 73.594

        # UTM変換器の設定
        self.setup_utm_converter()

        # カルマンフィルタの初期化
        self.ekf = ExtendedKalmanFilter()

        # パラメータ調整UI
        self.ui = KalmanFilterUI(self.ekf)
        self.ui.start()

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
        # 角速度
        self.current_angular_velocity = 0.0

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

        # カルマンフィルタ融合軌跡データ
        self.fused_trajectory_x = deque(maxlen=30000)
        self.fused_trajectory_y = deque(maxlen=30000)

        # GPS OFFモード用の積分変数
        self.odom_only_x = 0.0
        self.odom_only_y = 0.0
        self.odom_only_theta = 0.0
        self.odom_only_initialized = False

        # 初期化フラグ
        self.initialized = False
        self.gps_initialized = False

        # サブスクライバー
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)

        # プロット設定
        self.setup_plot()

        rospy.loginfo("Odometry Velocity Integrator with Kalman Filter GPS Fusion initialized.")
        rospy.loginfo(f"GPS Reference: lat={self.ref_lat}, lon={self.ref_lon}, alt={self.ref_alt}")
        rospy.loginfo("Initial heading: East (0 degrees)")
        rospy.loginfo("Coordinate system: UTM (local)")
        rospy.loginfo("Kalman Filter UI started - adjust parameters in the UI window")

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
        self.ax1.set_title('Kalman Filter Fusion: Odometry + GPS (UTM Coordinate System)', fontsize=14)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal', adjustable='box')

        # カルマンフィルタ融合軌跡
        self.line_fused, = self.ax1.plot([], [], 'r-', linewidth=3, label='Fused (Kalman Filter)', alpha=0.9, zorder=5)
        self.point_fused_current, = self.ax1.plot([], [], 'r*', markersize=15, label='Current position (Fused)', zorder=6)

        # オドメトリ積分軌跡（GPS座標系にアライン）
        self.line_traj, = self.ax1.plot([], [], 'b-', linewidth=1.5, label='Odometry (GPS-aligned)', alpha=0.5)
        self.point_current, = self.ax1.plot([], [], 'bo', markersize=8, label='Current position (Odom)', alpha=0.5)

        # GPS軌跡
        self.line_gps, = self.ax1.plot([], [], 'g-', linewidth=1.5, label='GPS trajectory', alpha=0.5)
        self.point_gps_current, = self.ax1.plot([], [], 'go', markersize=8, label='Current position (GPS)', alpha=0.5)

        # 基準点を表示
        self.ax1.plot([0], [0], 'k*', markersize=15, label='GPS reference point', zorder=10)

        self.ax1.legend(loc='upper right', fontsize=9)

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

        # 位置オフセットを計算（現在位置を合わせる）
        if len(self.gps_trajectory_x) > 0:
            # 現在のGPS位置を取得
            cur_gps_x = self.gps_trajectory_x[-1]
            cur_gps_y = self.gps_trajectory_y[-1]

            # 現在のオドメトリ位置を回転
            cos_t = math.cos(self.odom_to_gps_heading_offset)
            sin_t = math.sin(self.odom_to_gps_heading_offset)
            rot_x = self.integrated_x * cos_t - self.integrated_y * sin_t
            rot_y = self.integrated_x * sin_t + self.integrated_y * cos_t

            # 回転後の位置とGPS位置の差分がオフセット
            self.odom_to_gps_x_offset = cur_gps_x - rot_x
            self.odom_to_gps_y_offset = cur_gps_y - rot_y
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

        # カルマンフィルタのGPS更新ステップ（GPS使用モードの場合のみ）
        if self.ekf.initialized and self.ui.use_gps:
            self.ekf.update_gps(rel_x, rel_y)
            # 更新後の状態を取得
            state = self.ekf.get_state()
            # 融合軌跡の最後の点を更新（予測後にGPS更新するため）
            if len(self.fused_trajectory_x) > 0:
                self.fused_trajectory_x[-1] = state[0]
                self.fused_trajectory_y[-1] = state[1]

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

        # 移動距離を計算
        dist = math.hypot(dx, dy)

        # 小さすぎる移動はノイズ扱い（静止時の方位スナップを防止）
        if dist < 1e-4:  # 0.1mm未満は静止とみなす
            v = 0.0
            w = 0.0
            movement_theta = self.integrated_theta  # 方位は保持
        else:
            # 線速度を計算
            v = dist / self.dt

            # 移動方向の角度を計算
            movement_theta = math.atan2(dy, dx)

            # 角速度を計算（方位の変化率）
            dtheta = movement_theta - self.integrated_theta
            # 角度を-piからpiの範囲に正規化
            dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
            w = dtheta / self.dt

        # 現在速度・角速度を更新（GPS方位推定で使用）
        self.current_velocity = v
        self.current_angular_velocity = w

        # 速度を積分して位置と姿勢を更新
        # 姿勢を更新
        self.integrated_theta = movement_theta

        # 位置を更新（速度ベクトルを積分）
        self.integrated_x += v * math.cos(self.integrated_theta) * self.dt
        self.integrated_y += v * math.sin(self.integrated_theta) * self.dt

        # 軌跡に追加
        self.trajectory_x.append(self.integrated_x)
        self.trajectory_y.append(self.integrated_y)

        # カルマンフィルタの予測ステップ（GPS座標系で）
        if self.alignment_done:
            # オドメトリ座標をGPS座標系に変換
            odom_gps_x, odom_gps_y = self.transform_odom_to_gps(self.integrated_x, self.integrated_y)

            # GPS使用モードの場合はカルマンフィルタを使用
            if self.ui.use_gps:
                # カルマンフィルタが初期化されていない場合は初期化
                if not self.ekf.initialized:
                    self.ekf.initialize(odom_gps_x, odom_gps_y, self.integrated_theta + self.odom_to_gps_heading_offset, v)
                    rospy.loginfo("Kalman Filter initialized with odometry position")

                # 予測ステップ
                self.ekf.predict(self.dt, w)

                # 速度観測更新（オドメトリから得られた速度で更新）
                self.ekf.update_velocity(v)

                # 融合結果の軌跡に追加
                state = self.ekf.get_state()
                self.fused_trajectory_x.append(state[0])
                self.fused_trajectory_y.append(state[1])

                # GPS OFFモードの初期化フラグをリセット
                self.odom_only_initialized = False
            else:
                # GPS OFFモード: その時点の位置からv, wを積分

                # GPS OFF直後の初期化
                if self.ui.gps_just_turned_off or not self.odom_only_initialized:
                    # カルマンフィルタの現在位置を起点とする
                    if self.ekf.initialized:
                        state = self.ekf.get_state()
                        self.odom_only_x = state[0]
                        self.odom_only_y = state[1]
                        self.odom_only_theta = state[2]
                        rospy.loginfo(f"GPS OFF mode initialized at: ({self.odom_only_x:.3f}, {self.odom_only_y:.3f}), theta={math.degrees(self.odom_only_theta):.1f}°")
                    else:
                        # カルマンフィルタが初期化されていない場合は現在のオドメトリ位置を使用
                        self.odom_only_x = odom_gps_x
                        self.odom_only_y = odom_gps_y
                        self.odom_only_theta = self.integrated_theta + self.odom_to_gps_heading_offset
                        rospy.loginfo(f"GPS OFF mode initialized at: ({self.odom_only_x:.3f}, {self.odom_only_y:.3f}), theta={math.degrees(self.odom_only_theta):.1f}°")

                    self.odom_only_initialized = True
                    self.ui.gps_just_turned_off = False

                # v, wから積分
                self.odom_only_x += v * math.cos(self.odom_only_theta) * self.dt
                self.odom_only_y += v * math.sin(self.odom_only_theta) * self.dt
                self.odom_only_theta += w * self.dt
                # 角度を正規化
                self.odom_only_theta = math.atan2(math.sin(self.odom_only_theta), math.cos(self.odom_only_theta))

                # 融合軌跡に追加
                self.fused_trajectory_x.append(self.odom_only_x)
                self.fused_trajectory_y.append(self.odom_only_y)

        # ログ出力（定期的に、5秒ごとに概算）
        # 簡易的なカウンター（time_historyがないので軌跡長で代用）
        if len(self.trajectory_x) % 50 == 0 and len(self.trajectory_x) > 0:
            if self.ekf.initialized:
                state = self.ekf.get_state()
                rospy.loginfo(f"Odom: v={v:.3f} m/s, w={w:.3f} rad/s, "
                             f"pos=({self.integrated_x:.3f}, {self.integrated_y:.3f}), "
                             f"theta={math.degrees(self.integrated_theta):.1f}°")
                rospy.loginfo(f"Fused: pos=({state[0]:.3f}, {state[1]:.3f}), "
                             f"theta={math.degrees(state[2]):.1f}°, v={state[3]:.3f} m/s")
            else:
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

        # カルマンフィルタ融合軌跡プロット
        fused_x_list = []
        fused_y_list = []
        if len(self.fused_trajectory_x) > 0:
            fused_x_list = list(self.fused_trajectory_x)
            fused_y_list = list(self.fused_trajectory_y)
            self.line_fused.set_data(fused_x_list, fused_y_list)
            # 最新の融合位置を表示
            self.point_fused_current.set_data([fused_x_list[-1]], [fused_y_list[-1]])

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

        # 融合座標を追加
        if len(fused_x_list) > 0:
            all_x.extend(fused_x_list)
            all_y.extend(fused_y_list)

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

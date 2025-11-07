#!/usr/bin/env python3
# coding=utf-8

import rospy
import math
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
import threading

# UTM基準点（つくばチャレンジ用）
UTM_BASE_LAT = 36.0830041
UTM_BASE_LON = 140.0763757
UTM_BASE_ALT = 73.568

class GPSLocalization:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node('gps_localization', anonymous=False)

        # IMUベース方位推定のパラメータ
        self.imu_topic = rospy.get_param('~imu_topic', '/mid360/livox/imu')
        self.min_speed_for_gps_heading = rospy.get_param('~min_speed_for_gps_heading', 0.3)  # [m/s]
        self.gps_heading_gain_fast = rospy.get_param('~gps_heading_gain_fast', 0.4)  # 速い時の補正ゲイン
        self.gps_heading_gain_slow = rospy.get_param('~gps_heading_gain_slow', 0.2)  # 低〜中速の補正ゲイン
        self.bias_learn_gain = rospy.get_param('~bias_learn_gain', 0.02)  # ジャイロバイアス学習係数

        # Odometry補完機能のON/OFFフラグ（デフォルト: OFF、IMU使用のため不要）
        self.use_odom_complement = rospy.get_param('~use_odom_complement', False)

        # GPS情報を格納する変数
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.current_altitude = 0.0
        self.gps_lock = threading.Lock()
        self.gps_received = False

        # GPS由来のUTM座標（基準位置）
        self.gps_utm_x = 0.0
        self.gps_utm_y = 0.0
        self.gps_update_time = None

        # IMU関連の状態
        self.last_imu_time = None
        self.imu_received = False
        self.gyro_bias_z = 0.0  # ジャイロバイアス推定値
        self.has_abs_orientation = False  # IMUのorientationが有効かどうか
        self.imu_lock = threading.Lock()

        # Odometry情報を格納する変数（使用しない）
        self.odom_prev_x = None
        self.odom_prev_y = None
        self.odom_prev_time = None
        self.odom_lock = threading.Lock()

        # 補完された現在位置
        self.current_utm_x = 0.0
        self.current_utm_y = 0.0
        self.current_yaw = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        # 前回の位置（GPS進行方位計算用）
        self.prev_gps_utm_x = None
        self.prev_gps_utm_y = None
        self.prev_gps_time = None

        # 移動距離の閾値（メートル）
        self.movement_threshold = 0.5

        # /amcl_poseトピックのパブリッシャー
        self.amcl_pose_pub = rospy.Publisher(
            '/amcl_pose',
            PoseWithCovarianceStamped,
            queue_size=10
        )

        # TFブロードキャスター
        self.tf_broadcaster = tf.TransformBroadcaster()

        # GPS情報のサブスクライバー
        rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)

        # IMU情報のサブスクライバー
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)

        # Odometry情報のサブスクライバー（使用しない）
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback)

        # パブリッシュレート（Hz）- IMUに合わせて高頻度に
        self.publish_rate = rospy.Rate(50)

        rospy.loginfo("GPS+IMU Heading Fusion Localization Node initialized")
        rospy.loginfo(f"Base point: lat={UTM_BASE_LAT}, lon={UTM_BASE_LON}, alt={UTM_BASE_ALT}")
        rospy.loginfo(f"IMU topic: {self.imu_topic}")
        rospy.loginfo(f"Min speed for GPS heading: {self.min_speed_for_gps_heading} m/s")
        rospy.loginfo(f"GPS heading gain (slow/fast): {self.gps_heading_gain_slow}/{self.gps_heading_gain_fast}")
        rospy.loginfo(f"Gyro bias learn gain: {self.bias_learn_gain}")

    def wrap_angle(self, a):
        """角度を-πからπの範囲に正規化"""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def imu_callback(self, msg):
        """IMU情報から角速度を積分してヨーを更新"""
        with self.imu_lock:
            # タイムスタンプ
            stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

            # orientationが有効ならそれを使う（cov[0] < 0 は無効の慣例）
            ori_cov0 = msg.orientation_covariance[0] if len(msg.orientation_covariance) > 0 else -1.0
            orientation_valid = (ori_cov0 >= 0.0)

            if orientation_valid:
                # orientationが有効な場合はそれを使用
                q = msg.orientation
                _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                self.current_yaw = self.wrap_angle(yaw)
                self.has_abs_orientation = True
            else:
                # 角速度のZを積分（ジャイロバイアスを補正）
                if self.last_imu_time is not None:
                    dt = (stamp - self.last_imu_time).to_sec()
                    if 0.0 < dt < 0.2:  # 5Hz〜100Hzくらいを想定した簡易ガード
                        wz = msg.angular_velocity.z  # 符号そのまま
                        self.current_yaw = self.wrap_angle(self.current_yaw + (wz - self.gyro_bias_z) * dt)

            self.last_imu_time = stamp
            self.imu_received = True

    def gps_callback(self, msg):
        """GPS情報を受信してUTM座標を更新し、GPS進行方位でヨーを補正"""
        with self.gps_lock:
            self.current_latitude = msg.latitude
            self.current_longitude = msg.longitude
            self.current_altitude = msg.altitude
            self.gps_received = True

            # GPS情報をUTM座標に変換
            utm_x, utm_y = self.latlon_to_utm(
                self.current_latitude,
                self.current_longitude,
                UTM_BASE_LAT,
                UTM_BASE_LON
            )

            # GPS由来の基準位置を更新
            self.gps_utm_x = utm_x
            self.gps_utm_y = utm_y
            self.gps_update_time = rospy.Time.now()

            # 現在位置もGPSで更新（位置はGPSをそのまま採用）
            self.current_utm_x = utm_x
            self.current_utm_y = utm_y

            # GPS進行方位によるヨー補正
            if self.prev_gps_utm_x is not None and self.prev_gps_utm_y is not None and self.prev_gps_time is not None:
                dx = utm_x - self.prev_gps_utm_x
                dy = utm_y - self.prev_gps_utm_y
                distance = math.hypot(dx, dy)
                dt_gps = (rospy.Time.now() - self.prev_gps_time).to_sec()

                if dt_gps > 0.0:
                    speed = distance / dt_gps  # [m/s]

                    # 移動が十分であれば、進行方位（course over ground）を測定として採用
                    if distance >= self.movement_threshold and speed >= self.min_speed_for_gps_heading:
                        # ROSのmap座標系: x=前方(北), y=左(西)
                        # UTM座標: utm_x=東, utm_y=北
                        # 北向き(y軸正)が0度、東向き(x軸正)が90度 → atan2(dx, dy)
                        yaw_gps = math.atan2(dx, dy)  # 引数順序修正

                        # 予測yaw（現状のIMUベース yaw）
                        yaw_pred = self.current_yaw

                        # イノベーション（角度差は [-pi, pi] に正規化）
                        innov = self.wrap_angle(yaw_gps - yaw_pred)

                        # 速度に応じてゲインを切り替え（単純な2段階）
                        k = self.gps_heading_gain_fast if speed > 1.0 else self.gps_heading_gain_slow

                        # 補正（コンプリメンタリ風）
                        self.current_yaw = self.wrap_angle(yaw_pred + k * innov)

                        # ジャイロバイアスのゆっくり学習
                        self.gyro_bias_z += self.bias_learn_gain * dt_gps * innov

                        rospy.loginfo(f"[GPS heading fuse] speed={speed:.2f} m/s, k={k:.2f}, innov={math.degrees(innov):.1f}°, bias_z={self.gyro_bias_z:.5f}")
                    else:
                        rospy.logdebug(f"GPS movement too small or speed too low: dist={distance:.3f}m, speed={speed:.2f}m/s")

            # 前回値更新
            self.prev_gps_utm_x = utm_x
            self.prev_gps_utm_y = utm_y
            self.prev_gps_time = rospy.Time.now()

            rospy.loginfo(f"GPS updated: UTM({utm_x:.2f}, {utm_y:.2f}), yaw={math.degrees(self.current_yaw):.1f}°")

    def odom_callback(self, msg):
        """Odometry情報（使用しない - IMU使用のため）"""
        # IMU使用のためOdometryは使わない
        return

    def latlon_to_utm(self, latitude, longitude, base_lat, base_lon):
        """
        緯度経度をUTM座標（メートル）に変換する簡易関数
        基準点からの相対位置を計算
        """
        # 地球の平均半径（メートル）
        R = 6378137.0

        # 緯度経度の差をラジアンに変換
        dlat = math.radians(latitude - base_lat)
        dlon = math.radians(longitude - base_lon)

        # 基準点の緯度をラジアンに変換
        base_lat_rad = math.radians(base_lat)

        # UTM座標に変換（メートル単位）
        utm_x = R * dlon * math.cos(base_lat_rad)
        utm_y = R * dlat

        return utm_x, utm_y

    def publish_pose_and_tf(self):
        """
        /amcl_poseトピックとTFを定期的にパブリッシュ
        """
        while not rospy.is_shutdown():
            if not self.gps_received:
                self.publish_rate.sleep()
                continue

            # 現在の補完された位置と方位角を使用（スレッドセーフにアクセス）
            with self.gps_lock:
                utm_x = self.current_utm_x
                utm_y = self.current_utm_y

            with self.imu_lock:
                yaw = self.current_yaw

            # クォータニオンに変換
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

            # 現在時刻
            current_time = rospy.Time.now()

            # /amcl_poseメッセージを作成
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = "map"

            # 位置を設定
            pose_msg.pose.pose.position.x = utm_x
            pose_msg.pose.pose.position.y = utm_y
            pose_msg.pose.pose.position.z = 0.0

            # 姿勢を設定
            pose_msg.pose.pose.orientation.x = quat[0]
            pose_msg.pose.pose.orientation.y = quat[1]
            pose_msg.pose.pose.orientation.z = quat[2]
            pose_msg.pose.pose.orientation.w = quat[3]

            # 共分散行列を設定（分散 = 標準偏差の2乗）
            sigma_x, sigma_y, sigma_z = 2.0, 2.0, 0.1
            sigma_rp, sigma_yaw = 0.1, 0.5  # [rad] の標準偏差
            covariance = [0.0] * 36
            covariance[0] = sigma_x**2    # x-x (分散)
            covariance[7] = sigma_y**2    # y-y (分散)
            covariance[14] = sigma_z**2   # z-z (分散)
            covariance[21] = sigma_rp**2  # roll-roll (分散)
            covariance[28] = sigma_rp**2  # pitch-pitch (分散)
            covariance[35] = sigma_yaw**2 # yaw-yaw (分散)
            pose_msg.pose.covariance = covariance

            # /amcl_poseをパブリッシュ
            self.amcl_pose_pub.publish(pose_msg)

            # TFをブロードキャスト（map -> base_link）
            self.tf_broadcaster.sendTransform(
                (utm_x, utm_y, 0.0),
                quat,
                current_time,
                "base_link",
                "map"
            )

            self.publish_rate.sleep()

    def run(self):
        """メインループを実行"""
        try:
            self.publish_pose_and_tf()
        except rospy.ROSInterruptException:
            rospy.loginfo("GPS Localization Node shutting down")


def main():
    try:
        node = GPSLocalization()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# coding=utf-8

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse
from scipy.optimize import minimize

def calculate_rotation_matrix(xy_points, utm_points):
    """
    x,y座標をUTM座標に可能な限り近づけるような回転行列を計算

    Parameters:
    -----------
    xy_points : np.ndarray
        shape (N, 2) の x,y 座標
    utm_points : np.ndarray
        shape (N, 2) の utm_x, utm_y 座標

    Returns:
    --------
    rotation_matrix : np.ndarray
        2x2 回転行列
    theta : float
        回転角度（ラジアン）
    """

    def rotation_matrix_2d(theta):
        """2D回転行列を生成"""
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        return np.array([[cos_t, -sin_t],
                        [sin_t, cos_t]])

    def objective_function(theta):
        """最小化する目的関数：回転後のx,yとutm_x,utm_yの差の二乗和"""
        R = rotation_matrix_2d(theta[0])
        rotated_points = (R @ xy_points.T).T
        error = np.sum((rotated_points - utm_points) ** 2)
        return error

    # 初期値：0度
    initial_theta = [0.0]

    # 最適化
    result = minimize(objective_function, initial_theta, method='BFGS')
    optimal_theta = result.x[0]

    # 最適な回転行列を計算
    optimal_rotation_matrix = rotation_matrix_2d(optimal_theta)

    return optimal_rotation_matrix, optimal_theta


def main():
    parser = argparse.ArgumentParser(
        description='Calculate rotation matrix to align x,y coordinates with UTM coordinates'
    )
    parser.add_argument(
        '--csv',
        type=str,
        default='/home/nkb/catkin_ws/src/FAST_LIO/PCD/waypoint.csv',
        help='Path to waypoint CSV file'
    )
    parser.add_argument(
        '--start',
        type=int,
        default=None,
        help='Start row index (0-based, default: use all rows)'
    )
    parser.add_argument(
        '--end',
        type=int,
        default=None,
        help='End row index (0-based, exclusive, default: use all rows)'
    )

    args = parser.parse_args()

    # CSVファイルを読み込む
    print(f"Reading CSV file: {args.csv}")
    df = pd.read_csv(args.csv)

    print(f"Total rows in CSV: {len(df)}")

    # 必要なカラムが存在するかチェック
    required_columns = ['x', 'y', 'utm_x', 'utm_y']
    for col in required_columns:
        if col not in df.columns:
            raise ValueError(f"Required column '{col}' not found in CSV file")

    # 回転行列計算用のデータ範囲を決定
    if args.start is not None or args.end is not None:
        start_idx = args.start if args.start is not None else 0
        end_idx = args.end if args.end is not None else len(df)
        print(f"\nUsing rows {start_idx} to {end_idx-1} for rotation matrix calculation")
        df_calc = df.iloc[start_idx:end_idx].copy()
    else:
        print("\nUsing all rows for rotation matrix calculation")
        df_calc = df.copy()

    # x,y座標とUTM座標を抽出（計算用）
    xy_calc = df_calc[['x', 'y']].values
    utm_calc = df_calc[['utm_x', 'utm_y']].values

    # 回転行列を計算
    print("\nCalculating rotation matrix...")
    rotation_matrix, theta = calculate_rotation_matrix(xy_calc, utm_calc)

    print("\n" + "="*60)
    print("ROTATION MATRIX:")
    print("="*60)
    print(rotation_matrix)
    print(f"\nRotation angle: {np.degrees(theta):.6f} degrees ({theta:.6f} radians)")
    print("="*60)

    # 全データに対して回転を適用（描画用）
    xy_all = df[['x', 'y']].values
    utm_all = df[['utm_x', 'utm_y']].values
    xy_rotated = (rotation_matrix @ xy_all.T).T

    # 回転前後の誤差を計算
    error_before = np.sqrt(np.sum((xy_all - utm_all) ** 2, axis=1))
    error_after = np.sqrt(np.sum((xy_rotated - utm_all) ** 2, axis=1))

    print(f"\nError statistics (全{len(df)}行):")
    print(f"  Before rotation - Mean: {np.mean(error_before):.4f} m, Std: {np.std(error_before):.4f} m")
    print(f"  After rotation  - Mean: {np.mean(error_after):.4f} m, Std: {np.std(error_after):.4f} m")
    print(f"  Improvement: {np.mean(error_before) - np.mean(error_after):.4f} m")

    # 描画
    fig, axes = plt.subplots(2, 2, figsize=(16, 14))

    # 1. 回転前のx,yとUTM座標
    ax1 = axes[0, 0]
    ax1.scatter(xy_all[:, 0], xy_all[:, 1], c='blue', s=5, alpha=0.6, label='Original x,y')
    ax1.scatter(utm_all[:, 0], utm_all[:, 1], c='red', s=5, alpha=0.6, label='UTM x,y')
    # 計算に使用した範囲を強調表示
    if args.start is not None or args.end is not None:
        start_idx = args.start if args.start is not None else 0
        end_idx = args.end if args.end is not None else len(df)
        ax1.scatter(xy_calc[:, 0], xy_calc[:, 1], c='cyan', s=20, alpha=0.8,
                   marker='s', edgecolors='black', linewidth=1.0, label=f'Used for calc (rows {start_idx}-{end_idx-1})')
        ax1.scatter(utm_calc[:, 0], utm_calc[:, 1], c='orange', s=20, alpha=0.8,
                   marker='s', edgecolors='black', linewidth=1.0, label=f'UTM (calc range)')
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_title('Before Rotation: Original x,y vs UTM', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # 2. 回転後のx',y'とUTM座標
    ax2 = axes[0, 1]
    ax2.scatter(xy_rotated[:, 0], xy_rotated[:, 1], c='green', s=5, alpha=0.6, label="Rotated x',y'")
    ax2.scatter(utm_all[:, 0], utm_all[:, 1], c='red', s=5, alpha=0.6, label='UTM x,y')
    # 計算に使用した範囲を強調表示
    if args.start is not None or args.end is not None:
        xy_rotated_calc = (rotation_matrix @ xy_calc.T).T
        ax2.scatter(xy_rotated_calc[:, 0], xy_rotated_calc[:, 1], c='lightgreen', s=20, alpha=0.8,
                   marker='s', edgecolors='black', linewidth=1.0, label=f"Rotated (calc range)")
        ax2.scatter(utm_calc[:, 0], utm_calc[:, 1], c='orange', s=20, alpha=0.8,
                   marker='s', edgecolors='black', linewidth=1.0, label=f'UTM (calc range)')
    ax2.set_xlabel('X (m)', fontsize=12)
    ax2.set_ylabel('Y (m)', fontsize=12)
    ax2.set_title(f"After Rotation: Rotated x',y' vs UTM (θ={np.degrees(theta):.2f}°)",
                 fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')

    # 3. 誤差ベクトル（回転前）
    ax3 = axes[1, 0]
    for i in range(len(xy_all)):
        ax3.arrow(xy_all[i, 0], xy_all[i, 1],
                 utm_all[i, 0] - xy_all[i, 0], utm_all[i, 1] - xy_all[i, 1],
                 head_width=0.3, head_length=0.3, fc='blue', ec='blue', alpha=0.4, width=0.05)
    ax3.scatter(xy_all[:, 0], xy_all[:, 1], c='blue', s=5, alpha=0.8, label='Original x,y')
    ax3.scatter(utm_all[:, 0], utm_all[:, 1], c='red', s=5, alpha=0.8, label='UTM x,y')
    ax3.set_xlabel('X (m)', fontsize=12)
    ax3.set_ylabel('Y (m)', fontsize=12)
    ax3.set_title('Error Vectors: Before Rotation', fontsize=14, fontweight='bold')
    ax3.legend(fontsize=10)
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')

    # 4. 誤差ベクトル（回転後）
    ax4 = axes[1, 1]
    for i in range(len(xy_rotated)):
        ax4.arrow(xy_rotated[i, 0], xy_rotated[i, 1],
                 utm_all[i, 0] - xy_rotated[i, 0], utm_all[i, 1] - xy_rotated[i, 1],
                 head_width=0.3, head_length=0.3, fc='green', ec='green', alpha=0.4, width=0.05)
    ax4.scatter(xy_rotated[:, 0], xy_rotated[:, 1], c='green', s=5, alpha=0.8, label="Rotated x',y'")
    ax4.scatter(utm_all[:, 0], utm_all[:, 1], c='red', s=5, alpha=0.8, label='UTM x,y')
    ax4.set_xlabel('X (m)', fontsize=12)
    ax4.set_ylabel('Y (m)', fontsize=12)
    ax4.set_title('Error Vectors: After Rotation', fontsize=14, fontweight='bold')
    ax4.legend(fontsize=10)
    ax4.grid(True, alpha=0.3)
    ax4.axis('equal')

    plt.tight_layout()
    plt.savefig('/home/nkb/catkin_ws/src/tc2025/scripts/rotation_comparison.png', dpi=150, bbox_inches='tight')
    print("\nPlot saved to: /home/nkb/catkin_ws/src/tc2025/scripts/rotation_comparison.png")
    plt.show()

    # 結果をCSVに保存
    df_result = df.copy()
    df_result['x_rotated'] = xy_rotated[:, 0]
    df_result['y_rotated'] = xy_rotated[:, 1]
    df_result['error_before'] = error_before
    df_result['error_after'] = error_after

    output_csv = '/home/nkb/catkin_ws/src/tc2025/scripts/waypoint_with_rotation.csv'
    df_result.to_csv(output_csv, index=False)
    print(f"\nResults saved to: {output_csv}")

    # 回転行列をNumPy配列として保存
    output_npy = '/home/nkb/catkin_ws/src/tc2025/scripts/rotation_matrix.npy'
    np.save(output_npy, rotation_matrix)
    print(f"Rotation matrix saved to: {output_npy}")

    # 回転行列をテキストファイルにも保存
    output_txt = '/home/nkb/catkin_ws/src/tc2025/scripts/rotation_matrix.txt'
    with open(output_txt, 'w') as f:
        f.write("Rotation Matrix (2x2):\n")
        f.write(f"{rotation_matrix[0,0]:.10f} {rotation_matrix[0,1]:.10f}\n")
        f.write(f"{rotation_matrix[1,0]:.10f} {rotation_matrix[1,1]:.10f}\n")
        f.write(f"\nRotation Angle:\n")
        f.write(f"{np.degrees(theta):.10f} degrees\n")
        f.write(f"{theta:.10f} radians\n")
    print(f"Rotation matrix saved to: {output_txt}")


if __name__ == '__main__':
    main()

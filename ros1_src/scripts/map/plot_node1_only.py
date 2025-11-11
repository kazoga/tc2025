#!/usr/bin/env python3
# coding=utf-8

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse

def main():
    parser = argparse.ArgumentParser(
        description='Plot only node=1 waypoints with enlarged view'
    )
    parser.add_argument(
        '--csv',
        type=str,
        default='/home/nkb/catkin_ws/src/tc2025/scripts/waypoint_with_rotation_filtered.csv',
        help='Path to waypoint CSV file'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='/home/nkb/catkin_ws/src/tc2025/scripts/map/waypoint_node1_only.png',
        help='Output plot file path'
    )
    parser.add_argument(
        '--margin',
        type=float,
        default=20.0,
        help='Margin around node=1 points in meters (default: 20.0)'
    )

    args = parser.parse_args()

    # CSVファイルを読み込む
    print(f"Reading CSV file: {args.csv}")
    df = pd.read_csv(args.csv)

    print(f"Total waypoints: {len(df)}")

    # 必要なカラムが存在するかチェック
    required_columns = ['x_rotated', 'y_rotated', 'utm_x', 'utm_y', 'label', 'node']
    for col in required_columns:
        if col not in df.columns:
            raise ValueError(f"Required column '{col}' not found in CSV file")

    # データを抽出
    rotated_x = df['x_rotated'].values
    rotated_y = df['y_rotated'].values
    utm_x = df['utm_x'].values
    utm_y = df['utm_y'].values
    labels = df['label'].values
    nodes = df['node'].values

    # nodeが1の箇所のインデックスを取得
    node_1_indices = np.where(nodes == 1)[0]
    print(f"Waypoints with node=1: {len(node_1_indices)}")

    if len(node_1_indices) == 0:
        print("No node=1 waypoints found!")
        return

    # 誤差を計算
    error = np.sqrt((rotated_x - utm_x)**2 + (rotated_y - utm_y)**2)
    node_1_errors = error[node_1_indices]
    node_1_mean_error = np.mean(node_1_errors)
    node_1_max_error = np.max(node_1_errors)

    print(f"\nNode=1 Error statistics:")
    print(f"  Mean error: {node_1_mean_error:.4f} m")
    print(f"  Max error:  {node_1_max_error:.4f} m")
    print(f"  Min error:  {np.min(node_1_errors):.4f} m")

    # node=1の点の範囲を計算
    node1_rotated_x = rotated_x[node_1_indices]
    node1_rotated_y = rotated_y[node_1_indices]
    node1_utm_x = utm_x[node_1_indices]
    node1_utm_y = utm_y[node_1_indices]

    # 両方の座標系を含む範囲を計算
    all_x = np.concatenate([node1_rotated_x, node1_utm_x])
    all_y = np.concatenate([node1_rotated_y, node1_utm_y])

    x_min = np.min(all_x) - args.margin
    x_max = np.max(all_x) + args.margin
    y_min = np.min(all_y) - args.margin
    y_max = np.max(all_y) + args.margin

    print(f"\nPlot range: x=[{x_min:.1f}, {x_max:.1f}], y=[{y_min:.1f}, {y_max:.1f}]")

    # グラフを作成
    fig, ax = plt.subplots(1, 1, figsize=(16, 12))

    # 全軌跡を薄く表示（背景として）
    ax.plot(rotated_x, rotated_y, 'lightgray', linewidth=1.0, alpha=0.3, label='All trajectory (Rotated)')
    ax.plot(utm_x, utm_y, 'lightcoral', linewidth=1.0, alpha=0.3, label='All trajectory (UTM)')

    # 全ての点を薄くプロット
    ax.scatter(rotated_x, rotated_y, c='lightgreen', s=2, alpha=0.2, zorder=2)
    ax.scatter(utm_x, utm_y, c='lightpink', s=2, alpha=0.2, zorder=2)

    # Rotated側のnode=1の点を強調
    ax.scatter(node1_rotated_x, node1_rotated_y,
               c='cyan', s=150, marker='D', edgecolors='blue', linewidth=3,
               label='Node=1 (Rotated)', zorder=4)

    # UTM側のnode=1の点を強調
    ax.scatter(node1_utm_x, node1_utm_y,
               c='magenta', s=150, marker='D', edgecolors='purple', linewidth=3,
               label='Node=1 (UTM)', zorder=4)

    # node=1の箇所の誤差ベクトルを表示
    for idx in node_1_indices:
        ax.arrow(rotated_x[idx], rotated_y[idx],
                 utm_x[idx] - rotated_x[idx], utm_y[idx] - rotated_y[idx],
                 head_width=0.5, head_length=0.5, fc='orange', ec='orange',
                 alpha=0.7, width=0.15, zorder=3)

    # node=1の箇所にラベルを追加（Rotated側）
    for idx in node_1_indices:
        ax.annotate(f'{labels[idx]}',
                    xy=(rotated_x[idx], rotated_y[idx]),
                    xytext=(8, 8),
                    textcoords='offset points',
                    fontsize=10,
                    color='blue',
                    fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.4', facecolor='cyan', edgecolor='blue', alpha=0.9, linewidth=1.5),
                    arrowprops=dict(arrowstyle='->', color='blue', lw=1.2))

        # 誤差の値も表示
        ax.annotate(f'{error[idx]:.2f}m',
                    xy=(utm_x[idx], utm_y[idx]),
                    xytext=(8, -12),
                    textcoords='offset points',
                    fontsize=8,
                    color='red',
                    fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', edgecolor='red', alpha=0.8, linewidth=1.0))

    # 始点を強調
    ax.scatter(rotated_x[0], rotated_y[0], c='darkblue', s=200, marker='*',
               edgecolors='black', linewidth=2, label='Start', zorder=5)

    ax.set_xlabel('X (m)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=14, fontweight='bold')
    ax.set_title(f'Node=1 Waypoints (Enlarged View)\n(Count: {len(node_1_indices)}, Mean Error: {node_1_mean_error:.3f}m, Max Error: {node_1_max_error:.3f}m)',
                 fontsize=16, fontweight='bold')
    ax.legend(fontsize=11, loc='best')
    ax.grid(True, alpha=0.4, linestyle='--')
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(args.output, dpi=200, bbox_inches='tight')
    print(f"\nPlot saved to: {args.output}")
    # plt.show()  # コメントアウト（CLIで実行するため）


if __name__ == '__main__':
    main()

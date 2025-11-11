#!/usr/bin/env python3
# coding=utf-8

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse

def main():
    parser = argparse.ArgumentParser(
        description='Plot rotated and UTM waypoints from CSV file'
    )
    parser.add_argument(
        '--csv',
        type=str,
        default='/home/nkb/catkin_ws/src/tc2025/scripts/waypoint_with_rotation_filtered.csv',
        help='Path to waypoint CSV file'
    )
    parser.add_argument(
        '--label-interval',
        type=int,
        default=20,
        help='Label interval (default: 20 waypoints)'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='/home/nkb/catkin_ws/src/tc2025/scripts/map/waypoint_plot.png',
        help='Output plot file path'
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

    # 誤差を計算
    error = np.sqrt((rotated_x - utm_x)**2 + (rotated_y - utm_y)**2)
    mean_error = np.mean(error)
    max_error = np.max(error)
    min_error = np.min(error)

    print(f"\nError statistics:")
    print(f"  Mean error: {mean_error:.4f} m")
    print(f"  Max error:  {max_error:.4f} m")
    print(f"  Min error:  {min_error:.4f} m")

    # グラフを作成（3つ）
    fig, axes = plt.subplots(1, 3, figsize=(28, 9))

    # 1. Rotated座標とUTM座標の比較
    ax1 = axes[0]

    # 軌跡をプロット
    ax1.plot(rotated_x, rotated_y, 'g-', linewidth=1.5, alpha=0.5, label='Rotated trajectory')
    ax1.plot(utm_x, utm_y, 'r-', linewidth=1.5, alpha=0.5, label='UTM trajectory')

    # 点をプロット
    ax1.scatter(rotated_x, rotated_y, c='green', s=8, alpha=0.7, label='Rotated x,y', zorder=3)
    ax1.scatter(utm_x, utm_y, c='red', s=8, alpha=0.7, label='UTM x,y', zorder=3)

    # ラベルを追加（指定間隔ごと）
    for i in range(0, len(df), args.label_interval):
        # Rotated側にラベル
        ax1.annotate(f'{labels[i]}',
                    xy=(rotated_x[i], rotated_y[i]),
                    xytext=(5, 5),
                    textcoords='offset points',
                    fontsize=9,
                    color='darkgreen',
                    fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', edgecolor='green', alpha=0.7))

        # UTM側にラベル
        ax1.annotate(f'{labels[i]}',
                    xy=(utm_x[i], utm_y[i]),
                    xytext=(5, -15),
                    textcoords='offset points',
                    fontsize=9,
                    color='darkred',
                    fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow', edgecolor='red', alpha=0.7))

    # 始点と終点を強調
    ax1.scatter(rotated_x[0], rotated_y[0], c='blue', s=100, marker='*',
               edgecolors='black', linewidth=2, label='Start (Rotated)', zorder=5)
    ax1.scatter(utm_x[0], utm_y[0], c='purple', s=100, marker='*',
               edgecolors='black', linewidth=2, label='Start (UTM)', zorder=5)
    ax1.scatter(rotated_x[-1], rotated_y[-1], c='orange', s=100, marker='s',
               edgecolors='black', linewidth=2, label='End (Rotated)', zorder=5)
    ax1.scatter(utm_x[-1], utm_y[-1], c='brown', s=100, marker='s',
               edgecolors='black', linewidth=2, label='End (UTM)', zorder=5)

    ax1.set_xlabel('X (m)', fontsize=13, fontweight='bold')
    ax1.set_ylabel('Y (m)', fontsize=13, fontweight='bold')
    ax1.set_title(f'Rotated vs UTM Waypoints\n(Mean Error: {mean_error:.3f}m)',
                 fontsize=15, fontweight='bold')
    ax1.legend(fontsize=10, loc='best')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.set_xlim(-100, 600)
    ax1.set_ylim(-400, 150)
    ax1.set_aspect('equal')

    # 2. 誤差ベクトルの可視化
    ax2 = axes[1]

    # 軌跡をプロット
    ax2.plot(rotated_x, rotated_y, 'g-', linewidth=1.5, alpha=0.5, label='Rotated trajectory')
    ax2.plot(utm_x, utm_y, 'r-', linewidth=1.5, alpha=0.5, label='UTM trajectory')

    # 誤差ベクトルをプロット
    for i in range(len(df)):
        ax2.arrow(rotated_x[i], rotated_y[i],
                 utm_x[i] - rotated_x[i], utm_y[i] - rotated_y[i],
                 head_width=0.2, head_length=0.2, fc='blue', ec='blue',
                 alpha=0.3, width=0.03, zorder=2)

    # 点をプロット
    ax2.scatter(rotated_x, rotated_y, c='green', s=8, alpha=0.7, label='Rotated x,y', zorder=3)
    ax2.scatter(utm_x, utm_y, c='red', s=8, alpha=0.7, label='UTM x,y', zorder=3)

    # 誤差が大きい点を強調（上位5%）
    error_threshold = np.percentile(error, 95)
    large_error_indices = np.where(error > error_threshold)[0]

    if len(large_error_indices) > 0:
        ax2.scatter(rotated_x[large_error_indices], rotated_y[large_error_indices],
                   c='orange', s=50, marker='o', edgecolors='red', linewidth=2,
                   label=f'Large error (>{error_threshold:.2f}m)', zorder=4)

        # 大きな誤差の点にラベルを表示
        for idx in large_error_indices:
            ax2.annotate(f'{labels[idx]}\n({error[idx]:.2f}m)',
                        xy=(rotated_x[idx], rotated_y[idx]),
                        xytext=(8, 8),
                        textcoords='offset points',
                        fontsize=8,
                        color='red',
                        fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', edgecolor='red', alpha=0.8),
                        arrowprops=dict(arrowstyle='->', color='red', lw=1))

    # 始点を強調
    ax2.scatter(rotated_x[0], rotated_y[0], c='blue', s=100, marker='*',
               edgecolors='black', linewidth=2, label='Start', zorder=5)

    ax2.set_xlabel('X (m)', fontsize=13, fontweight='bold')
    ax2.set_ylabel('Y (m)', fontsize=13, fontweight='bold')
    ax2.set_title(f'Error Vectors: Rotated → UTM\n(Max Error: {max_error:.3f}m)',
                 fontsize=15, fontweight='bold')
    ax2.legend(fontsize=10, loc='best')
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_xlim(-100, 600)
    ax2.set_ylim(-400, 150)
    ax2.set_aspect('equal')

    # 3. Node=1の箇所の可視化
    ax3 = axes[2]

    # 軌跡をプロット（全体）
    ax3.plot(rotated_x, rotated_y, 'lightgray', linewidth=1.0, alpha=0.4, label='All trajectory (Rotated)')
    ax3.plot(utm_x, utm_y, 'lightcoral', linewidth=1.0, alpha=0.4, label='All trajectory (UTM)')

    # 全ての点を薄くプロット
    ax3.scatter(rotated_x, rotated_y, c='lightgreen', s=3, alpha=0.3, zorder=2)
    ax3.scatter(utm_x, utm_y, c='lightpink', s=3, alpha=0.3, zorder=2)

    if len(node_1_indices) > 0:
        # Rotated側のnode=1の点を強調
        ax3.scatter(rotated_x[node_1_indices], rotated_y[node_1_indices],
                   c='cyan', s=80, marker='D', edgecolors='blue', linewidth=2.5,
                   label='Node=1 (Rotated)', zorder=4)

        # UTM側のnode=1の点を強調
        ax3.scatter(utm_x[node_1_indices], utm_y[node_1_indices],
                   c='magenta', s=80, marker='D', edgecolors='purple', linewidth=2.5,
                   label='Node=1 (UTM)', zorder=4)

        # node=1の箇所の誤差ベクトルを表示
        for idx in node_1_indices:
            ax3.arrow(rotated_x[idx], rotated_y[idx],
                     utm_x[idx] - rotated_x[idx], utm_y[idx] - rotated_y[idx],
                     head_width=0.3, head_length=0.3, fc='orange', ec='orange',
                     alpha=0.7, width=0.1, zorder=3)

        # node=1の箇所にラベルを追加（Rotated側のみ）
        for idx in node_1_indices:
            ax3.annotate(f'N{labels[idx]}',
                        xy=(rotated_x[idx], rotated_y[idx]),
                        xytext=(6, 6),
                        textcoords='offset points',
                        fontsize=7,
                        color='blue',
                        fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.25', facecolor='cyan', edgecolor='blue', alpha=0.85, linewidth=1.0),
                        arrowprops=dict(arrowstyle='->', color='blue', lw=0.8))

        # node=1の平均誤差を計算
        node_1_errors = error[node_1_indices]
        node_1_mean_error = np.mean(node_1_errors)
        node_1_max_error = np.max(node_1_errors)

        ax3.set_title(f'Node=1 Waypoints\n(Count: {len(node_1_indices)}, Mean Error: {node_1_mean_error:.3f}m, Max Error: {node_1_max_error:.3f}m)',
                     fontsize=15, fontweight='bold')
    else:
        ax3.text(0.5, 0.5, 'No Node=1 waypoints found',
                ha='center', va='center', transform=ax3.transAxes,
                fontsize=16, color='gray', fontweight='bold')
        ax3.set_title('Node=1 Waypoints\n(No data)', fontsize=15, fontweight='bold')

    # 始点を強調
    ax3.scatter(rotated_x[0], rotated_y[0], c='blue', s=120, marker='*',
               edgecolors='black', linewidth=2, label='Start', zorder=5)

    ax3.set_xlabel('X (m)', fontsize=13, fontweight='bold')
    ax3.set_ylabel('Y (m)', fontsize=13, fontweight='bold')
    ax3.legend(fontsize=10, loc='best')
    ax3.grid(True, alpha=0.3, linestyle='--')
    ax3.set_xlim(-100, 600)
    ax3.set_ylim(-400, 150)
    ax3.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(args.output, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {args.output}")
    plt.show()

    # 統計情報をテキストファイルに保存
    stats_file = args.output.replace('.png', '_stats.txt')
    with open(stats_file, 'w') as f:
        f.write("Waypoint Statistics\n")
        f.write("="*50 + "\n\n")
        f.write(f"Total waypoints: {len(df)}\n")
        f.write(f"Label interval: {args.label_interval}\n")
        f.write(f"Waypoints with node=1: {len(node_1_indices)}\n\n")

        f.write(f"Error Statistics:\n")
        f.write(f"  Mean error:   {mean_error:.6f} m\n")
        f.write(f"  Max error:    {max_error:.6f} m\n")
        f.write(f"  Min error:    {min_error:.6f} m\n")
        f.write(f"  Std deviation: {np.std(error):.6f} m\n\n")

        if len(node_1_indices) > 0:
            f.write(f"Waypoints with node=1:\n")
            for idx in node_1_indices:
                f.write(f"  Label {labels[idx]}: error = {error[idx]:.6f} m\n")
            f.write("\n")

        f.write(f"Waypoints with largest errors (top 5):\n")
        largest_error_indices = np.argsort(error)[-5:][::-1]
        for idx in largest_error_indices:
            node_status = " (node=1)" if idx in node_1_indices else ""
            f.write(f"  Label {labels[idx]}: {error[idx]:.6f} m{node_status}\n")

    print(f"Statistics saved to: {stats_file}")


if __name__ == '__main__':
    main()

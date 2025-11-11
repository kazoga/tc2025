#!/usr/bin/env python3
# coding=utf-8

import pandas as pd
import argparse
from pathlib import Path


def remove_consecutive_node1(input_csv, output_csv=None):
    """
    連続して node=1 になっている行のうち、最初の行を残して後続の行を削除する

    Parameters:
    -----------
    input_csv : str
        入力CSVファイルのパス
    output_csv : str, optional
        出力CSVファイルのパス（指定しない場合は元のファイル名に _filtered を付加）

    Returns:
    --------
    filtered_df : pd.DataFrame
        フィルタリング後のDataFrame
    """
    # CSVファイルを読み込む
    print(f"Reading CSV file: {input_csv}")
    df = pd.read_csv(input_csv)

    print(f"Total rows before filtering: {len(df)}")

    # node列が存在するか確認
    if 'node' not in df.columns:
        raise ValueError("Column 'node' not found in CSV file")

    # フィルタリング用のマスクを作成
    keep_mask = []

    # 各行について、削除するかどうかを判定
    for i in range(len(df)):
        # 現在の行のnodeが1でない場合は残す
        if df.iloc[i]['node'] != 1:
            keep_mask.append(True)
            continue

        # 現在の行のnodeが1の場合
        # 前の行もnodeが1であれば削除、そうでなければ残す
        if i > 0 and df.iloc[i-1]['node'] == 1:
            keep_mask.append(False)  # 連続する2番目以降のnode=1行は削除
        else:
            keep_mask.append(True)   # 連続の最初のnode=1行は残す

    # フィルタリングを適用
    filtered_df = df[keep_mask].reset_index(drop=True)

    print(f"Total rows after filtering: {len(filtered_df)}")
    print(f"Removed {len(df) - len(filtered_df)} rows")

    # 削除された行の統計情報を表示
    removed_indices = [i for i, keep in enumerate(keep_mask) if not keep]
    if removed_indices:
        print(f"\nRemoved row indices (0-based): {removed_indices[:10]}{'...' if len(removed_indices) > 10 else ''}")
        print(f"Total removed: {len(removed_indices)} rows")

    # 出力ファイル名を決定
    if output_csv is None:
        input_path = Path(input_csv)
        output_csv = str(input_path.parent / (input_path.stem + '_filtered' + input_path.suffix))

    # 結果を保存
    filtered_df.to_csv(output_csv, index=False)
    print(f"\nFiltered CSV saved to: {output_csv}")

    return filtered_df


def main():
    parser = argparse.ArgumentParser(
        description='Remove consecutive node=1 rows, keeping only the first occurrence in each sequence'
    )
    parser.add_argument(
        '--input',
        type=str,
        default='/home/nkb/ros/tc2025/ros1_src/scripts/waypoint_with_rotation.csv',
        help='Path to input CSV file'
    )
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='Path to output CSV file (default: input_filename_filtered.csv)'
    )

    args = parser.parse_args()

    print("="*60)
    print("Remove Consecutive node=1 Rows")
    print("="*60)

    # フィルタリングを実行
    filtered_df = remove_consecutive_node1(args.input, args.output)

    # 統計情報を表示
    print("\n" + "="*60)
    print("Node value distribution (after filtering):")
    print("="*60)
    print(filtered_df['node'].value_counts().sort_index())

    print("\n" + "="*60)
    print("Done!")
    print("="*60)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# coding=utf-8

import numpy as np
import argparse
import os
import struct
from pathlib import Path

def load_pcd(filepath):
    """
    PCDファイルを読み込む (ASCII/Binary両対応)

    Parameters:
    -----------
    filepath : str
        PCDファイルのパス

    Returns:
    --------
    header_lines : list
        ヘッダー行のリスト
    points : np.ndarray
        点群データ (N, num_fields)
    fields : list
        フィールド名のリスト
    field_types : list
        フィールドのデータ型のリスト
    field_sizes : list
        フィールドのサイズのリスト
    is_binary : bool
        バイナリ形式かどうか
    """
    # ヘッダーをASCIIモードで読む
    header_lines = []
    points_count = 0
    fields = []
    field_sizes = []
    field_types = []
    field_counts = []
    is_binary = False
    width = 0
    height = 0

    with open(filepath, 'rb') as f:
        while True:
            line_bytes = f.readline()
            try:
                line = line_bytes.decode('ascii')
            except:
                break

            header_lines.append(line)

            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('SIZE'):
                field_sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('TYPE'):
                field_types = line.split()[1:]
            elif line.startswith('COUNT'):
                field_counts = [int(x) for x in line.split()[1:]]
            elif line.startswith('WIDTH'):
                width = int(line.split()[1])
            elif line.startswith('HEIGHT'):
                height = int(line.split()[1])
            elif line.startswith('POINTS'):
                points_count = int(line.split()[1])
            elif line.startswith('DATA'):
                data_format = line.split()[1].strip()
                is_binary = (data_format == 'binary')
                data_start_pos = f.tell()
                break

    # データ部分を読み込む
    if is_binary:
        # バイナリ形式
        with open(filepath, 'rb') as f:
            f.seek(data_start_pos)
            binary_data = f.read()

        # 各フィールドのフォーマット文字列を作成
        format_chars = []
        for ftype, fsize, fcount in zip(field_types, field_sizes, field_counts):
            for _ in range(fcount):
                if ftype == 'F':
                    if fsize == 4:
                        format_chars.append('f')
                    elif fsize == 8:
                        format_chars.append('d')
                elif ftype == 'U':
                    if fsize == 1:
                        format_chars.append('B')
                    elif fsize == 2:
                        format_chars.append('H')
                    elif fsize == 4:
                        format_chars.append('I')
                elif ftype == 'I':
                    if fsize == 1:
                        format_chars.append('b')
                    elif fsize == 2:
                        format_chars.append('h')
                    elif fsize == 4:
                        format_chars.append('i')

        point_format = '<' + ''.join(format_chars)
        point_size = struct.calcsize(point_format)

        points = []
        for i in range(points_count):
            point_data = struct.unpack_from(point_format, binary_data, i * point_size)
            points.append(point_data)

        points = np.array(points, dtype=np.float64)

    else:
        # ASCII形式
        with open(filepath, 'r') as f:
            lines = f.readlines()

        data_start_idx = 0
        for i, line in enumerate(lines):
            if line.startswith('DATA'):
                data_start_idx = i + 1
                break

        data_lines = lines[data_start_idx:]
        points = []

        for line in data_lines:
            if line.strip():
                values = line.strip().split()
                points.append([float(v) for v in values])

        points = np.array(points, dtype=np.float64)

    print(f"Loaded {len(points)} points from {filepath}")
    print(f"Point dimension: {points.shape[1]}")
    print(f"Fields: {fields}")
    print(f"Data format: {'binary' if is_binary else 'ascii'}")

    return header_lines, points, fields, field_types, field_sizes, is_binary


def save_pcd(filepath, header_lines, points, fields, field_types, field_sizes, is_binary):
    """
    PCDファイルを保存する (ASCII/Binary両対応)

    Parameters:
    -----------
    filepath : str
        保存先のパス
    header_lines : list
        ヘッダー行のリスト
    points : np.ndarray
        点群データ
    fields : list
        フィールド名のリスト
    field_types : list
        フィールドのデータ型のリスト
    field_sizes : list
        フィールドのサイズのリスト
    is_binary : bool
        バイナリ形式で保存するかどうか
    """
    # ヘッダーのPOINTS行とWIDTH行を更新
    for i, line in enumerate(header_lines):
        if line.startswith('POINTS'):
            header_lines[i] = f'POINTS {len(points)}\n'
        elif line.startswith('WIDTH'):
            header_lines[i] = f'WIDTH {len(points)}\n'

    if is_binary:
        # バイナリ形式で保存
        with open(filepath, 'wb') as f:
            # ヘッダーを書き込む
            for line in header_lines:
                f.write(line.encode('ascii'))

            # バイナリデータを書き込む
            # フォーマット文字列を作成
            format_chars = []
            for ftype, fsize in zip(field_types, field_sizes):
                if ftype == 'F':
                    if fsize == 4:
                        format_chars.append('f')
                    elif fsize == 8:
                        format_chars.append('d')
                elif ftype == 'U':
                    if fsize == 1:
                        format_chars.append('B')
                    elif fsize == 2:
                        format_chars.append('H')
                    elif fsize == 4:
                        format_chars.append('I')
                elif ftype == 'I':
                    if fsize == 1:
                        format_chars.append('b')
                    elif fsize == 2:
                        format_chars.append('h')
                    elif fsize == 4:
                        format_chars.append('i')

            point_format = '<' + ''.join(format_chars)

            for point in points:
                # float64からそれぞれのフォーマットに変換
                point_data = struct.pack(point_format, *point)
                f.write(point_data)

    else:
        # ASCII形式で保存
        with open(filepath, 'w') as f:
            # ヘッダーを書き込む
            for line in header_lines:
                f.write(line)

            # データを書き込む
            for point in points:
                f.write(' '.join([str(v) for v in point]) + '\n')

    print(f"Saved {len(points)} points to {filepath}")


def apply_rotation_to_pcd(pcd_path, matrix_path, output_path):
    """
    PCDファイルに回転行列を適用する

    Parameters:
    -----------
    pcd_path : str
        入力PCDファイルのパス
    matrix_path : str
        回転行列ファイル(.npy)のパス
    output_path : str
        出力PCDファイルのパス
    """
    # 回転行列を読み込む
    rotation_matrix = np.load(matrix_path)
    print(f"\nLoaded rotation matrix from {matrix_path}:")
    print(rotation_matrix)

    # 回転角度を計算
    theta = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    print(f"Rotation angle: {np.degrees(theta):.6f} degrees ({theta:.6f} radians)")

    # PCDファイルを読み込む
    header_lines, points, fields, field_types, field_sizes, is_binary = load_pcd(pcd_path)

    # XY座標に回転を適用（Z座標は保持）
    # points[:, 0:2] が X, Y 座標
    xy_original = points[:, 0:2]
    xy_rotated = (rotation_matrix @ xy_original.T).T

    # 回転後の座標で置き換え
    points_rotated = points.copy()
    points_rotated[:, 0:2] = xy_rotated

    # 統計情報を表示
    shift = xy_rotated - xy_original
    print(f"\nTransformation statistics:")
    print(f"  X shift - Mean: {np.mean(shift[:, 0]):.6f} m, Std: {np.std(shift[:, 0]):.6f} m")
    print(f"  Y shift - Mean: {np.mean(shift[:, 1]):.6f} m, Std: {np.std(shift[:, 1]):.6f} m")
    print(f"  Total shift - Mean: {np.mean(np.linalg.norm(shift, axis=1)):.6f} m")

    # 結果を保存
    save_pcd(output_path, header_lines, points_rotated, fields, field_types, field_sizes, is_binary)
    print(f"\nRotated PCD saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Apply rotation matrix to PCD file'
    )
    parser.add_argument(
        '--pcd',
        type=str,
        required=True,
        help='Path to input PCD file'
    )
    parser.add_argument(
        '--matrix',
        type=str,
        required=True,
        help='Path to rotation matrix file (.npy)'
    )
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='Path to output PCD file (default: input_filename_rot.pcd in the same directory as input)'
    )

    args = parser.parse_args()

    # 入力ファイルの存在確認
    if not os.path.exists(args.pcd):
        raise FileNotFoundError(f"PCD file not found: {args.pcd}")

    if not os.path.exists(args.matrix):
        raise FileNotFoundError(f"Matrix file not found: {args.matrix}")

    # 出力ファイル名を決定
    if args.output is None:
        pcd_path = Path(args.pcd)
        output_filename = pcd_path.stem + '_rot' + pcd_path.suffix
        output_path = str(pcd_path.parent / output_filename)
    else:
        output_path = args.output

    print(f"Input PCD: {args.pcd}")
    print(f"Rotation matrix: {args.matrix}")
    print(f"Output PCD: {output_path}")
    print("="*60)

    # 回転を適用
    apply_rotation_to_pcd(args.pcd, args.matrix, output_path)

    print("="*60)
    print("Done!")


if __name__ == '__main__':
    main()

"""2D相似変換の推定と座標変換を提供するモジュール。"""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np


_SCALE = 1.0
_ROT = np.eye(2)
_TRANS = np.zeros((2, 1))
_INITIALIZED = False


def estimate_similarity(
    P_list: Sequence[Tuple[float, float]],
    Q_list: Sequence[Tuple[float, float]],
) -> Tuple[float, np.ndarray, np.ndarray]:
    """対応点列から2D相似変換を推定する。"""
    if len(P_list) != len(Q_list) or len(P_list) < 2:
        raise ValueError("対応点の数が不足しています。2点以上で同数が必要です。")

    P = np.array(P_list, dtype=float).T
    Q = np.array(Q_list, dtype=float).T

    p_mean = np.mean(P, axis=1, keepdims=True)
    q_mean = np.mean(Q, axis=1, keepdims=True)
    P_centered = P - p_mean
    Q_centered = Q - q_mean

    cov = Q_centered @ P_centered.T / P.shape[1]
    U, _, Vt = np.linalg.svd(cov)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt

    var_P = np.sum(P_centered ** 2) / P.shape[1]
    scale = np.trace(np.eye(2) @ (cov.T @ R)) / var_P

    t = q_mean - scale * R @ p_mean

    global _SCALE, _ROT, _TRANS, _INITIALIZED
    _SCALE = float(scale)
    _ROT = R
    _TRANS = t
    _INITIALIZED = True
    return _SCALE, _ROT, _TRANS


def xy_to_en(x: float, y: float) -> Tuple[float, float]:
    """内部で推定した変換を用いて(x, y)を(E, N)へ変換する。"""
    if not _INITIALIZED:
        raise RuntimeError("先にestimate_similarity()で変換を推定してください。")
    vec = np.array([[x], [y]], dtype=float)
    en = _SCALE * _ROT @ vec + _TRANS
    return float(en[0, 0]), float(en[1, 0])


def en_to_xy(E: float, N: float) -> Tuple[float, float]:
    """内部で推定した変換の逆変換で(E, N)を(x, y)へ変換する。"""
    if not _INITIALIZED:
        raise RuntimeError("先にestimate_similarity()で変換を推定してください。")
    rot_inv = np.linalg.inv(_ROT)
    vec = np.array([[E], [N]], dtype=float)
    xy = (1.0 / _SCALE) * rot_inv @ (vec - _TRANS)
    return float(xy[0, 0]), float(xy[1, 0])

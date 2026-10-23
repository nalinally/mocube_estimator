"""
モジュールの面とサーボの幾何.

全モジュールが同じ向きで並ぶので、軸は全体で共通にとる: r 面 = +x, f 面 = +y, u 面 = +z。
面の番号 0..5 (u, r, f, d, l, b) はその面のサーボの ICS ID と同じで、
0..2 (u, r, f) は赤外線受信チャンネルとも同じ。d, l, b 面には赤外線 LED がある。

面 i のサーボを正回転させると、その面に接している隣のモジュールが
自分に対して DRIVES[i] の向きに動く。
この割り当てだと、どの接触面でも「どちらのサーボを正回転させても、
正の側のモジュールが負の側に対して軸の正方向へ動く」。
"""

import numpy as np

FACES = ('u', 'r', 'f', 'd', 'l', 'b')
FACE_INDEX = {face: i for i, face in enumerate(FACES)}

# 面の外向き法線
NORMALS = np.array([
    (0, 0, 1),   # u
    (1, 0, 0),   # r
    (0, 1, 0),   # f
    (0, 0, -1),  # d
    (-1, 0, 0),  # l
    (0, -1, 0),  # b
], dtype=float)

# サーボを正回転させたときに、隣のモジュールが自分に対して動く向き
DRIVES = np.array([
    (1, 0, 0),   # u
    (0, 1, 0),   # r
    (0, 0, 1),   # f
    (0, -1, 0),  # d
    (0, 0, -1),  # l
    (-1, 0, 0),  # b
], dtype=float)

# 軸 (0=x, 1=y, 2=z) の正の側の面と負の側の面
POSITIVE_FACE = (FACE_INDEX['r'], FACE_INDEX['f'], FACE_INDEX['u'])
NEGATIVE_FACE = (FACE_INDEX['l'], FACE_INDEX['b'], FACE_INDEX['d'])

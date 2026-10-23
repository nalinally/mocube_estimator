"""
モジュールの位置推定.

連続位置はサーボ指令から積算し (デッドレコニング)、赤外線で隣接を受けたらその組を
ぴったり 1 マス隣にリセットする。赤外線を受けたときの格子座標も別に持つ。
ROS に依存しないので単体テストできる。

- 位置はモジュール中心 [m]。ID0 は動かさない (固定)
- 一度も置かれていない (赤外線で見えていない・手動設定もされていない) モジュールは
  localized = False で、積算にも接触判定にも参加しない

積算のしかた:
  推定位置から接触している組を探し、組ごとに「正の側 - 負の側」の相対速度を
  両側のサーボ指令から決める (駆動していない向きは 0 = くっついたまま)。
  これを全モジュールの速度についての連立方程式にして、ID0 の速度を 0 に固定して最小二乗で解く。
  ループ状につながっていても、複数のサーボを同時に回しても同じ式で扱える。
"""

from dataclasses import dataclass, field

from mocube_estimator.geometry import (
    DRIVES,
    FACE_INDEX,
    NEGATIVE_FACE,
    NORMALS,
    POSITIVE_FACE,
)
import numpy as np

SERVO_COUNT = 6
SERVO_MAX_DEG = 135.0  # mocube_bridge と同じ。これを超える指令はブリッジが捨てる

FIXED_ID = 0


@dataclass
class ModuleEstimate:
    """1 モジュール分の推定状態."""

    id: int  # noqa: A003  (Module.msg に合わせる)
    online: bool = False
    localized: bool = False
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [m]
    grid: tuple = (0, 0, 0)  # 最後に赤外線か手動設定で確定した格子座標
    grid_t: float = 0.0      # grid を確定した時刻 (UNIX time [s])
    # サーボごとに最後に届いた回転指令 [deg]。None は脱力 (止まっているとみなす)
    commands: list = field(default_factory=lambda: [None] * SERVO_COUNT)


@dataclass(frozen=True)
class Contact:
    """接触している組. upper が lower の axis 正方向側にいる."""

    lower: int
    upper: int
    axis: int


class Estimator:

    def __init__(self, edge_length, speed_gain, deadband=0.0, contact_tolerance=0.1):
        """
        パラメータ.

        edge_length: モジュールの一辺 [m]
        speed_gain: 回転指令 1 deg あたりの相対移動速度 [m/s]
        deadband: この大きさ [deg] 以下の指令は止まっているとみなす
        contact_tolerance: 接触判定の許容 (一辺に対する割合)。
          面どうしの距離のずれがこれ以下で、面内の重なりがこれ以上なら接触とみなす
        """
        self.edge_length = edge_length
        self.speed_gain = speed_gain
        self.deadband = deadband
        self.contact_tolerance = contact_tolerance
        self.modules = {}

    # ------------------------------------------------------------
    # 入力
    # ------------------------------------------------------------

    def _get(self, module_id):
        module = self.modules.get(module_id)
        if module is None:
            module = ModuleEstimate(module_id)
            self.modules[module_id] = module
        return module

    def update_modules(self, modules):
        """(id, online) の一覧で、知っているモジュールとオンライン状態を更新する."""
        for module_id, online in modules:
            self._get(module_id).online = online

    def set_grid_positions(self, positions, stamp):
        """{id: (x, y, z)} の格子座標に置く (手動設定・初期配置). 知らない ID も作る."""
        for module_id, grid in positions.items():
            module = self._get(module_id)
            module.localized = True
            module.grid = tuple(int(v) for v in grid)
            module.grid_t = stamp
            module.position = np.array(module.grid, dtype=float) * self.edge_length

    def set_servo(self, module_id, servo, angle):
        """
        サーボ指令を受ける. angle は回転指令 [deg]、None は脱力.

        実際には届かない指令 (知らない・オフラインのモジュール宛て、範囲外) は無視して False を返す。
        """
        module = self.modules.get(module_id)
        if module is None or not module.online:
            return False
        if not 0 <= servo < SERVO_COUNT:
            return False
        if angle is not None and not -SERVO_MAX_DEG <= angle <= SERVO_MAX_DEG:
            return False
        module.commands[servo] = angle
        return True

    def free_module(self, module_id):
        """モジュールの全サーボの脱力指令を受ける."""
        module = self.modules.get(module_id)
        if module is None or not module.online:
            return False
        module.commands = [None] * SERVO_COUNT
        return True

    def observe(self, observer_id, face, sender_id, stamp):
        """
        observer_id の face 面の受信器が sender_id のビーコンを受けた.

        sender を observer の face 側ぴったり 1 マス隣に置き直す。
        どちらも置かれていないなどで今は反映できないとき False (あとで再試行してよい)。
        """
        if observer_id == sender_id:
            return False
        observer = self.modules.get(observer_id)
        sender = self.modules.get(sender_id)
        if observer is None or sender is None:
            return False
        if not observer.localized and not sender.localized:
            return False

        normal = NORMALS[FACE_INDEX[face]]
        step = normal.astype(int)

        if not sender.localized:
            self._place(sender, observer, step, stamp)
            return True
        if not observer.localized:
            self._place(observer, sender, -step, stamp)
            return True

        # observer から見た sender のずれを消す。ID0 を含まない側の塊を動かす
        delta = observer.position + normal * self.edge_length - sender.position
        sender_side, observer_side = self._split(observer_id, sender_id)
        if sender_side is None:
            # ループの中の組なので塊に分けられない。ID0 でない方だけ動かす
            sender_side, observer_side = {sender_id}, {observer_id}
        if FIXED_ID in sender_side:
            moved, partner, anchor, step, delta = (
                observer_side, observer, sender, -step, -delta)
        else:
            moved, partner, anchor = sender_side, sender, observer

        for module_id in moved:
            self.modules[module_id].position = self.modules[module_id].position + delta
        partner.grid = tuple(int(v) for v in np.add(anchor.grid, step))
        for module_id in moved:
            module = self.modules[module_id]
            offset = np.rint((module.position - partner.position) / self.edge_length)
            module.grid = tuple(int(v) for v in np.add(partner.grid, offset))
            module.grid_t = stamp
        return True

    def _place(self, module, anchor, step, stamp):
        """置かれていない module を anchor から step マスの位置に置く."""
        module.localized = True
        module.position = anchor.position + step * self.edge_length
        module.grid = tuple(int(v) for v in np.add(anchor.grid, step))
        module.grid_t = stamp

    def _split(self, a, b):
        """
        a-b の接触を除いた接触グラフで、b 側と a 側の塊 (ID の集合) を返す.

        除いてもつながっている (ループになっている) ときは (None, None)。
        """
        adjacency = {}
        for contact in self.find_contacts():
            if {contact.lower, contact.upper} == {a, b}:
                continue
            adjacency.setdefault(contact.lower, set()).add(contact.upper)
            adjacency.setdefault(contact.upper, set()).add(contact.lower)

        def component(start):
            seen = {start}
            stack = [start]
            while stack:
                for neighbor in adjacency.get(stack.pop(), ()):
                    if neighbor not in seen:
                        seen.add(neighbor)
                        stack.append(neighbor)
            return seen

        b_side = component(b)
        if a in b_side:
            return None, None
        return b_side, component(a)

    # ------------------------------------------------------------
    # 積算
    # ------------------------------------------------------------

    def find_contacts(self):
        """推定位置から、面どうしが接している組を探す."""
        localized = [m for m in self.modules.values() if m.localized]
        tolerance = self.contact_tolerance * self.edge_length
        contacts = []
        for i, first in enumerate(localized):
            for second in localized[i + 1:]:
                d = second.position - first.position
                for axis in range(3):
                    others = [k for k in range(3) if k != axis]
                    if abs(abs(d[axis]) - self.edge_length) > tolerance:
                        continue
                    if any(abs(d[k]) > self.edge_length - tolerance for k in others):
                        continue
                    if d[axis] > 0:
                        contacts.append(Contact(first.id, second.id, axis))
                    else:
                        contacts.append(Contact(second.id, first.id, axis))
                    break
        return contacts

    def speed(self, module_id, servo):
        """サーボの回転指令から、そのサーボが生む相対移動の速さ [m/s] (符号つき)."""
        module = self.modules[module_id]
        angle = module.commands[servo]
        if angle is None or not module.online or abs(angle) <= self.deadband:
            return 0.0
        return self.speed_gain * angle

    def relative_velocity(self, contact):
        """接触している組の、lower に対する upper の速度 [m/s]."""
        lower_face = POSITIVE_FACE[contact.axis]
        upper_face = NEGATIVE_FACE[contact.axis]
        # upper のサーボは lower を upper に対して動かすので、符号が逆になる
        return (self.speed(contact.lower, lower_face) * DRIVES[lower_face]
                - self.speed(contact.upper, upper_face) * DRIVES[upper_face])

    def velocities(self):
        """{id: 速度 [m/s]} を最小二乗で解く. ID0 は 0 に固定."""
        ids = [m.id for m in self.modules.values() if m.localized]
        if not ids:
            return {}
        contacts = self.find_contacts()
        free_ids = [i for i in ids if i != FIXED_ID]
        result = {i: np.zeros(3) for i in ids}
        if not contacts or not free_ids:
            return result

        column = {module_id: k for k, module_id in enumerate(free_ids)}
        incidence = np.zeros((len(contacts), len(free_ids)))
        target = np.zeros((len(contacts), 3))
        for row, contact in enumerate(contacts):
            if contact.upper in column:
                incidence[row, column[contact.upper]] = 1.0
            if contact.lower in column:
                incidence[row, column[contact.lower]] = -1.0
            target[row] = self.relative_velocity(contact)

        # どこにも接していない塊は解が決まらないので、lstsq の最小ノルム解 (=止まっている) になる
        solution, *_ = np.linalg.lstsq(incidence, target, rcond=None)
        for module_id, k in column.items():
            result[module_id] = solution[k]
        return result

    def step(self, dt):
        """時間 dt [s] だけ位置を進める."""
        for module_id, velocity in self.velocities().items():
            module = self.modules[module_id]
            module.position = module.position + velocity * dt


def parse_grid_positions(data):
    """
    読み込んだ {id: [x, y, z]} (YAML など) を検査して {int: (int, int, int)} にする.

    形式がおかしければ ValueError。
    """
    if not isinstance(data, dict):
        raise ValueError('expected a mapping of id -> [x, y, z]')
    positions = {}
    for key, value in data.items():
        try:
            module_id = int(key)
        except (TypeError, ValueError):
            raise ValueError(f'bad module id: {key!r}') from None
        if (not isinstance(value, (list, tuple)) or len(value) != 3
                or not all(isinstance(v, int) and not isinstance(v, bool) for v in value)):
            raise ValueError(f'ID {module_id}: expected [x, y, z] integers, got {value!r}')
        positions[module_id] = tuple(value)
    return positions

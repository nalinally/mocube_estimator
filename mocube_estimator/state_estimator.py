"""
state_estimator ノード.

/modules の赤外線の受信状態と /servo_commands の回転指令から各モジュールの位置を推定し、
/module_states と TF (world_frame -> <frame_prefix><id>) に流す。推定の中身は estimator.py。

位置の手動設定:
  - 起動時: パラメータ initial_positions_file の YAML ({id: [x, y, z]} の格子座標)
  - 実行中: サービス set_grid_positions
どちらも無ければ ID0 を原点に置き、あとは赤外線で見えた順に置いていく。
"""

import os
import time

from geometry_msgs.msg import TransformStamped
from mocube_estimator.estimator import Estimator, FIXED_ID, parse_grid_positions
from mocube_msgs.msg import Modules, ModuleState, ModuleStates, ServoCommand, ServoCommands
from mocube_msgs.srv import SetGridPositions
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import yaml

# IrReceiver.address のうち、隣のモジュールの ID ではないもの
NO_RECEPTION = -1
UNKNOWN_SENDER = 255


def load_grid_file(path):
    """格子座標の YAML ファイルを {id: (x, y, z)} として読む."""
    with open(path, encoding='utf-8') as f:
        return parse_grid_positions(yaml.safe_load(f) or {})


def to_module_states_msg(modules, stamp):
    """推定状態 (ModuleEstimate) の一覧を ModuleStates メッセージにする."""
    msg = ModuleStates()
    msg.t = stamp
    for module in sorted(modules, key=lambda m: m.id):
        state = ModuleState()
        state.id = int(module.id)
        state.localized = module.localized
        state.position.x, state.position.y, state.position.z = (
            float(v) for v in module.position)
        state.grid_x, state.grid_y, state.grid_z = module.grid
        state.grid_t = float(module.grid_t)
        msg.modules.append(state)
    return msg


class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')

        edge_length_mm = self.declare_parameter('edge_length_mm', 110.3).value
        # 回転指令 1 deg あたりの相対移動速度 [mm/s]。未測定の仮の値
        speed_gain_mm = self.declare_parameter('speed_gain_mm', 0.2).value
        deadband = self.declare_parameter('deadband_deg', 0.0).value
        contact_tolerance = self.declare_parameter('contact_tolerance', 0.1).value
        update_period = self.declare_parameter('update_period', 0.02).value
        initial_positions_file = self.declare_parameter('initial_positions_file', '').value
        self._world_frame = self.declare_parameter('world_frame', 'world').value
        self._frame_prefix = self.declare_parameter('frame_prefix', 'mocube_').value

        self._estimator = Estimator(
            edge_length_mm / 1000.0, speed_gain_mm / 1000.0, deadband, contact_tolerance)

        positions = {FIXED_ID: (0, 0, 0)}
        if initial_positions_file:
            path = os.path.abspath(os.path.expanduser(initial_positions_file))
            positions = load_grid_file(path)
            self.get_logger().info(f'Loaded {len(positions)} positions from {path}')
        self._estimator.set_grid_positions(positions, time.time())

        # (id, 面) -> 反映済みの受信時刻。これより新しい受信だけを反映する
        self._processed = {}

        self._modules_subscription = self.create_subscription(
            Modules, 'modules', self._on_modules, 10)
        self._servo_subscription = self.create_subscription(
            ServoCommands, 'servo_commands', self._on_servo_commands, 10)
        self._service = self.create_service(
            SetGridPositions, 'set_grid_positions', self._on_set_grid_positions)
        self._publisher = self.create_publisher(ModuleStates, 'module_states', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._last_step = time.monotonic()
        self._timer = self.create_timer(update_period, self._on_timer)

    # ------------------------------------------------------------
    # subscription / service
    # ------------------------------------------------------------

    def _on_modules(self, msg):
        self._estimator.update_modules((m.id, m.online) for m in msg.modules)
        for module in msg.modules:
            for receiver in module.receivers:
                if receiver.address in (NO_RECEPTION, UNKNOWN_SENDER):
                    continue
                key = (module.id, receiver.face)
                if receiver.t <= self._processed.get(key, 0.0):
                    continue
                # どちらもまだ置かれていなければ反映できないので、次の /modules で再試行する
                if self._estimator.observe(
                        module.id, receiver.face, receiver.address, receiver.t):
                    self._processed[key] = receiver.t

    def _on_servo_commands(self, msg):
        # mocube_bridge と同じ解釈。届かない指令は Estimator が無視する
        for command in msg.commands:
            if command.mode == ServoCommand.POSITION:
                self._estimator.set_servo(command.module_id, command.servo, command.angle)
            elif command.mode == ServoCommand.FREE:
                if command.servo == ServoCommand.ALL_SERVOS:
                    self._estimator.free_module(command.module_id)
                else:
                    self._estimator.set_servo(command.module_id, command.servo, None)

    def _on_set_grid_positions(self, request, response):
        positions = {p.id: (p.x, p.y, p.z) for p in request.positions}
        self._estimator.set_grid_positions(positions, time.time())
        response.success = True
        response.message = f'set {len(positions)} positions'
        self.get_logger().info(f'Set grid positions: {positions}')
        return response

    # ------------------------------------------------------------
    # timer
    # ------------------------------------------------------------

    def _on_timer(self):
        now = time.monotonic()
        self._estimator.step(now - self._last_step)
        self._last_step = now

        modules = list(self._estimator.modules.values())
        self._publisher.publish(to_module_states_msg(modules, time.time()))
        self._broadcast_tf(modules)

    def _broadcast_tf(self, modules):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for module in modules:
            if not module.localized:
                continue
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._world_frame
            transform.child_frame_id = f'{self._frame_prefix}{module.id}'
            (transform.transform.translation.x,
             transform.transform.translation.y,
             transform.transform.translation.z) = (float(v) for v in module.position)
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        if transforms:
            self._tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)

    state_estimator = StateEstimator()

    try:
        rclpy.spin(state_estimator)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        state_estimator.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

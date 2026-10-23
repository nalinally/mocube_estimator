from mocube_estimator.estimator import Contact, Estimator, parse_grid_positions
import numpy as np
import pytest

L = 0.1


def make_estimator(positions, speed_gain=0.001):
    """与えた格子座標に置き、全台オンラインにする."""
    estimator = Estimator(L, speed_gain)
    estimator.update_modules((i, True) for i in positions)
    estimator.set_grid_positions(positions, 1.0)
    return estimator


def position(estimator, module_id):
    return estimator.modules[module_id].position


def test_find_contacts_orders_by_axis():
    estimator = make_estimator({0: (0, 0, 0), 1: (1, 0, 0), 2: (0, 0, -1), 3: (2, 2, 0)})
    assert set(estimator.find_contacts()) == {Contact(0, 1, 0), Contact(2, 0, 2)}


def test_find_contacts_needs_face_overlap():
    estimator = make_estimator({0: (0, 0, 0), 1: (1, 0, 0)})
    estimator.modules[1].position = np.array([L, 0.95 * L, 0.0])
    assert estimator.find_contacts() == []
    estimator.modules[1].position = np.array([L, 0.5 * L, 0.0])
    assert estimator.find_contacts() == [Contact(0, 1, 0)]


# 正の側のどちらのサーボを正回転させても、正の側のモジュールが正方向へ動く
@pytest.mark.parametrize('upper_grid, lower_servo, upper_servo, lower_dir, upper_dir', [
    ((1, 0, 0), 1, 4, (0, 1, 0), (0, 0, 1)),  # r / l
    ((0, 1, 0), 2, 5, (0, 0, 1), (1, 0, 0)),  # f / b
    ((0, 0, 1), 0, 3, (1, 0, 0), (0, 1, 0)),  # u / d
])
def test_positive_rotation_moves_upper_side_positive(
        upper_grid, lower_servo, upper_servo, lower_dir, upper_dir):
    for module_id, servo, direction in ((0, lower_servo, lower_dir), (1, upper_servo, upper_dir)):
        estimator = make_estimator({0: (0, 0, 0), 1: upper_grid})
        estimator.set_servo(module_id, servo, 100.0)
        estimator.step(0.1)
        expected = np.array(upper_grid) * L + 0.001 * 100.0 * 0.1 * np.array(direction)
        np.testing.assert_allclose(position(estimator, 0), [0, 0, 0])
        np.testing.assert_allclose(position(estimator, 1), expected)


def test_chain_moves_together():
    # 0 - 1 - 2 と x 方向に並び、0 の r サーボで 1 を +y に動かすと 2 も一緒に動く
    estimator = make_estimator({0: (0, 0, 0), 1: (1, 0, 0), 2: (2, 0, 0)})
    estimator.set_servo(0, 1, 50.0)
    estimator.step(0.2)
    np.testing.assert_allclose(position(estimator, 1), [L, 0.01, 0])
    np.testing.assert_allclose(position(estimator, 2), [2 * L, 0.01, 0])


def test_ignores_undeliverable_commands():
    estimator = make_estimator({0: (0, 0, 0), 1: (1, 0, 0)})
    estimator.update_modules([(0, False)])
    assert not estimator.set_servo(0, 1, 50.0)   # オフライン
    assert not estimator.set_servo(1, 4, 200.0)  # 範囲外
    assert not estimator.set_servo(1, 6, 50.0)   # サーボ番号外
    assert not estimator.set_servo(9, 0, 50.0)   # 知らないモジュール
    estimator.step(1.0)
    np.testing.assert_allclose(position(estimator, 1), [L, 0, 0])


def test_free_and_deadband_stop():
    estimator = make_estimator({0: (0, 0, 0), 1: (1, 0, 0)})
    estimator.set_servo(0, 1, 50.0)
    estimator.free_module(0)
    estimator.step(1.0)
    np.testing.assert_allclose(position(estimator, 1), [L, 0, 0])

    estimator.deadband = 10.0
    estimator.set_servo(0, 1, 10.0)
    estimator.step(1.0)
    np.testing.assert_allclose(position(estimator, 1), [L, 0, 0])


def test_observe_places_unlocalized_modules():
    estimator = make_estimator({0: (0, 0, 0)})
    estimator.update_modules([(1, True), (2, True)])

    # 1 と 2 はどちらも置かれていないので、まだ反映できない
    assert not estimator.observe(1, 'u', 2, 5.0)
    assert estimator.observe(0, 'r', 1, 5.0)
    assert estimator.observe(1, 'u', 2, 6.0)

    assert estimator.modules[1].grid == (1, 0, 0)
    assert estimator.modules[2].grid == (1, 0, 1)
    assert estimator.modules[2].grid_t == 6.0
    np.testing.assert_allclose(position(estimator, 2), [L, 0, L])


def test_observe_places_observer_from_sender():
    estimator = make_estimator({0: (0, 0, 0)})
    estimator.update_modules([(1, True)])
    assert estimator.observe(1, 'f', 0, 5.0)  # 1 の f 面の先に 0 がいる
    assert estimator.modules[1].grid == (0, -1, 0)


def test_observe_resets_drifted_group():
    # 0 の r 側に 1、その u 側に 2。1 と 2 は推定がずれている
    estimator = make_estimator({0: (0, 0, 0), 1: (1, 0, 0), 2: (1, 0, 1)})
    drift = np.array([0.0, 0.3 * L, 0.0])
    for module_id in (1, 2):
        estimator.modules[module_id].position = estimator.modules[module_id].position + drift
    estimator.modules[1].grid = (1, 5, 0)

    assert estimator.observe(0, 'r', 1, 7.0)
    np.testing.assert_allclose(position(estimator, 1), [L, 0, 0])
    np.testing.assert_allclose(position(estimator, 2), [L, 0, L])
    assert estimator.modules[1].grid == (1, 0, 0)
    assert estimator.modules[2].grid == (1, 0, 1)
    assert estimator.modules[2].grid_t == 7.0
    np.testing.assert_allclose(position(estimator, 0), [0, 0, 0])


def test_observe_moves_side_without_fixed_module():
    # 1 の r 側に 0 がいるのを 1 が見た。動かすのは ID0 を含まない 1 の側
    estimator = make_estimator({0: (0, 0, 0), 1: (-1, 0, 0)})
    estimator.modules[1].position = np.array([-L, 0.2 * L, 0.0])

    assert estimator.observe(1, 'r', 0, 3.0)
    np.testing.assert_allclose(position(estimator, 0), [0, 0, 0])
    np.testing.assert_allclose(position(estimator, 1), [-L, 0, 0])
    assert estimator.modules[1].grid == (-1, 0, 0)


def test_observe_in_loop_moves_sender_only():
    # 2x2 の正方形 (z = 0): 0 (0,0), 1 (1,0), 2 (0,1), 3 (1,1)
    estimator = make_estimator({0: (0, 0, 0), 1: (1, 0, 0), 2: (0, 1, 0), 3: (1, 1, 0)})
    estimator.modules[3].position = estimator.modules[3].position + [0.05 * L, 0, 0]

    assert estimator.observe(2, 'r', 3, 4.0)
    np.testing.assert_allclose(position(estimator, 3), [L, L, 0])
    np.testing.assert_allclose(position(estimator, 1), [L, 0, 0])


def test_parse_grid_positions():
    assert parse_grid_positions({0: [0, 0, 0], '3': [1, -2, 0]}) == {
        0: (0, 0, 0), 3: (1, -2, 0)}
    with pytest.raises(ValueError):
        parse_grid_positions({0: [0, 0]})
    with pytest.raises(ValueError):
        parse_grid_positions({0: [0.5, 0, 0]})
    with pytest.raises(ValueError):
        parse_grid_positions({'a': [0, 0, 0]})
    with pytest.raises(ValueError):
        parse_grid_positions([[0, 0, 0]])

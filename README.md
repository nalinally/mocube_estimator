# mocube_estimator

mocube のモジュールの位置を推定する ROS 2 パッケージです。
`mocube_bridge` が出す `/modules` (赤外線の受信状態) と、`/servo_commands` (サーボへの回転指令) を購読し、
各モジュールの位置を `/module_states` と TF に流します。可視化はこのパッケージではやりません (RViz などで TF を見る)。

## 座標と面

全モジュールが同じ向きで並ぶので、軸は全体で共通です。ID0 を原点に固定します。

| 面 | 法線 | サーボ ICS ID | 正回転で隣が動く向き | 赤外線 |
|---|---|---|---|---|
| u | +z | 0 | +x | 受信 ch0 |
| r | +x | 1 | +y | 受信 ch1 |
| f | +y | 2 | +z | 受信 ch2 |
| d | −z | 3 | −y | LED |
| l | −x | 4 | −z | LED |
| b | −y | 5 | −x | LED |

1 つの接触面では、向かい合う 2 台のサーボが面内の別々の軸を受け持ちます。
どちらのサーボでも、**正回転で「正の側のモジュールが負の側に対して軸の正方向へ動く」** ようになっています
(例: A の +x 側に B がいるとき、A の r サーボで B が +y、B の l サーボで B が +z に動く)。

隣り合う 2 台のうち、赤外線を受けるのは負の側の 1 台だけです (A の r 面の受信器が B の l 面の LED を受ける)。

## 推定のしかた

- **連続位置** (`position`, [m]): サーボ指令からの積算
  1. 推定位置から、面どうしが接している組を探す (`contact_tolerance` の範囲で、面内の重なりがあるもの)
  2. 組ごとに相対速度を両側の回転指令から決める。駆動していない向きは 0 (くっついたまま)
  3. 全モジュールの速度を、ID0 = 0 に固定して最小二乗で解き、積分する。
     ループ状の配置や複数サーボの同時駆動もこの式で扱う
- **赤外線を受けたとき**: その組をぴったり 1 マス隣にリセットする。
  その組の接触を除いて塊に分け、ID0 を含まない側の塊ごと動かす
  (ループの中の組で分けられないときは、ID0 でない方の 1 台だけ動かす)
- **格子座標** (`grid_*`): 赤外線でリセットしたとき (または手動設定したとき) に確定した値。
  動かした塊の全員が、相手の格子座標 + 1 マスを基準に更新され、`grid_t` にその時刻が入る

赤外線は「今もつながっているか」の判断には使わず、新しく受信したとき (`IrReceiver.t` が進んだとき) だけ反映します。
推定と赤外線が食い違ったら赤外線を信じます。

置かれていない (赤外線で見えていない・手動設定もされていない) モジュールは `localized = false` で、
積算にも接触判定にも参加しません。置かれているモジュールの隣で赤外線に見えた時点で置かれます。

サーボ指令の解釈は `mocube_bridge` と同じです。
次の指令はモジュールに届かないので無視します: オフライン・未登録のモジュール宛て、±135 deg を超える角度、サーボ番号外。
回転指令は次の指令が来るまで続くものとし、脱力 (FREE) は停止とみなします。

## 使い方

```bash
ros2 run mocube_estimator state_estimator
# 初期配置を与える場合
ros2 run mocube_estimator state_estimator --ros-args -p initial_positions_file:=~/mocube_layout.yaml
```

初期配置の YAML は `config/initial_positions.example.yaml` を参照してください (`ID: [x, y, z]` の格子座標)。
与えなければ ID0 を原点に置き、あとは赤外線で見えた順に置いていきます。

実行中に位置を手で直す (赤外線の認識ミスなど):

```bash
ros2 service call /set_grid_positions mocube_msgs/srv/SetGridPositions \
  "{positions: [{id: 1, x: 1, y: 0, z: 0}, {id: 2, x: 1, y: 0, z: 1}]}"
```

## トピック・サービス・TF

| 名前 | 型 | 向き |
|---|---|---|
| `/modules` | `mocube_msgs/Modules` | 購読 (赤外線の受信状態・オンライン状態) |
| `/servo_commands` | `mocube_msgs/ServoCommands` | 購読 (回転指令) |
| `/module_states` | `mocube_msgs/ModuleStates` | 出力 (`update_period` ごと) |
| `/set_grid_positions` | `mocube_msgs/srv/SetGridPositions` | サービス |
| TF `world` → `mocube_<id>` | | 置かれているモジュールのみ。回転は常に単位 |

## パラメータ

| 名前 | 既定値 | 内容 |
|---|---|---|
| `edge_length_mm` | 113.1 | モジュールの一辺 [mm] |
| `speed_gain_mm` | 0.2 | 回転指令 1 deg あたりの相対移動速度 [mm/s]。**未測定の仮の値** |
| `deadband_deg` | 0.0 | この大きさ以下の回転指令は停止とみなす [deg] |
| `contact_tolerance` | 0.1 | 接触判定の許容 (一辺に対する割合) |
| `update_period` | 0.02 | 積算と publish の周期 [s] |
| `initial_positions_file` | `''` | 初期配置の YAML。空なら ID0 だけを原点に置く |
| `world_frame` | `world` | TF の親フレーム |
| `frame_prefix` | `mocube_` | TF の子フレーム名の接頭辞 |

## 構成

| ファイル | 内容 |
|---|---|
| `geometry.py` | 面の法線とサーボの駆動方向の表 |
| `estimator.py` | 推定の本体 (`Estimator`)。ROS に依存しないので単体テストできる |
| `state_estimator.py` | ROS ノード。購読・出力・サービス・TF |

```bash
colcon test --packages-select mocube_estimator && colcon test-result --verbose
```

## 既知の限界

- 赤外線はぴったり揃っている間ずっと受信されるので、揃った状態から動き出した直後は、受信が途切れるまで何度もリセットされる。
  そのぶん (受信できる範囲のずれ) は積算に入らない
- 速度は指令に比例するとしている。`speed_gain_mm` は実測して合わせる必要がある
- 重力や、支えのないモジュールの落下は考えない。どこにも接していない塊は止まっているものとする

# ROS 2 高専ロボコン実践編

### 5S altair

## 環境

- Python 3.10
- ThinkPad L380 Ubuntu 22.04.3 LTS
- ROS2 Humble

# 初めに

まず，高専ロボコンにおける ROS2 の意義について説明する．

## ROS2 を使う理由

高専ロボコンのロボット開発では，以下の要素が求められる．

- **リアルタイム制御**：センサー情報を処理しながらロボットをスムーズに動作させる．
- **モジュール化**：複数の機能（モーター制御，画像処理，位置推定など）を分離し，開発しやすくする．
- **通信の簡素化**：各機能を担当するノード間でのデータのやり取りを簡潔にする．
- **拡張性**：機能追加がしやすい構造にする．

ROS2 は，これらの要素を満たすための優れたフレームワークである．

そこで 2024 年高専ロボコン A チームの Roboware を基に解説をしていく．

https://github.com/SkenHub/2024_A_ROS2_Roboware

# システム構築

## ROS2 を用いたシステムの全体構成

ROS2 を利用したロボコンのシステムでは，以下のような構成を取る．

- **通信層**：WebSocket を用いたスマートフォンとの通信
- **制御層**：ロボットの運動を計算し，適切な制御コマンドを生成
- **ハードウェア層**：シリアル通信を介してマイコンへ制御コマンドを送信
- **センシング層**：カメラや LiDAR を用いた環境認識と自己位置推定など

各層の役割を整理すると次のようになる．

| 層             | 役割                   | 具体的な処理例                         |
| -------------- | ---------------------- | -------------------------------------- |
| 通信層         | スマホ・PC との通信    | WebSocket を使用し，ユーザー入力を取得 |
| 制御層         | 経路計画・制御         | 位置推定を行いながら目標地点へ移動     |
| ハードウェア層 | モータードライバの制御 | 速度・角速度をシリアル通信で送信       |
| センシング層   | ロボットの自己位置推定 | RealSense T265 + マイコンのデータ統合  |

---

## 各ノードの役割

ROS2 の特徴として，機能ごとにノードを分割して管理できる．
以下に例を示す．

### 1. WebSocket 通信ノード (`web_socket_node`)

- **役割**：スマホコントローラーからの指示を受け取り，ROS2 トピックへ変換
- **使用技術**：FastAPI，WebSocket，ROS2 パブリッシャー

### 2. コントローラーノード (`controller_node`)

- **役割**：受け取った指示をもとに目標地点を計算し，適切な速度・方向・角速度を算出
- **重要な処理**：
  - 経路計画（現在位置 → 目標位置）
  - 速度制御（最大速度・加速度制限）
  - 方向制御（回転角度の計算）

### 3. シリアル送信ノード (`serial_send_node`)

- **役割**：計算された速度指示をシリアル通信でマイコンへ送信
- **通信フォーマット**：
  - `0xA5 0xA5 [指示番号] [モード] [Vx] [Vy] [omega]`
  - 115200 bps のシリアル通信で送信

### 4. シリアル受信ノード (`serial_read_node`)

- **役割**：マイコンからのフィードバック情報を取得し，自己位置データを ROS2 に公開
- **通信フォーマット**：
  - `0xA5 0xA5 [動作番号] [モード] [X] [Y] [θ]`
  - 115200 bps のシリアル通信で受信

### 5. 自己位置推定ノード (`position_node`)

- **役割**：
  - マイコンと RealSense T265 からの自己位置データを統合
  - 重み付き平均で誤差補正
  - `estimated_position` トピックを配信

# 通信層

高専ロボコンにおいてロボットに対して指令を送る際は遠隔操作を行わなければならない．
そこで通信モジュールとして ESP32，IM920 などが挙げられるが，これらには問題がある．

- **ESP32 の場合**
  - 2.4GHz 帯域で通信を行うため，混線の可能性が大きい．
  - 安定した通信を確保するには工夫が必要．
- **IM920 の場合**
  - 電波法の関係上，通信速度に制限がある（長距離通信向けであり、リアルタイム性が求められるロボコンには不向き）．

このため，WIFI を用いた通信を選択し，その実装として**WebSocket**を採用する．

### WebSocket のメリット

- **リアルタイム通信**：ロボットの操作に対し，低遅延で反応できる．
- **双方向通信**：ロボットの状態をスマートフォンに送信することも可，
- **軽量**：HTTP リクエストのように毎回ヘッダ情報を送る必要がないため，通信の負荷が軽い．

## FastAPI を利用した WebSocket サーバーの実装

WebSocket 通信を ROS2 と連携させるために**FastAPI**を使用する，FastAPI は軽量かつ高性能な Python の Web フレームワークであり，WebSocket を簡単に扱える．

### FastAPI の特徴

- **非同期処理（async/await）対応**
- **高速な通信処理（Starlette ベース）**
- **簡単な WebSocket 実装**

### WebSocket サーバーの基本実装（FastAPI）

以下のコードは，FastAPI を使用して WebSocket サーバーを作成し，クライアントとの通信を管理する．

#### **`web_socket_node.py`（通信ノード）**

```python
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from fastapi import FastAPI, WebSocket as FastAPIWebSocket
from fastapi.responses import HTMLResponse
import uvicorn
import os

# IPアドレスとポート設定
IP_ADDRESS = '192.168.98.216'
PORT = 8010

# UIファイル（`R1_UI.txt`）のパス
UI_PATH = '/home/altair/2024_A_ROS2_Roboware/src/robot_controller/R1_UI.txt'

# FastAPIのインスタンスを作成
app = FastAPI()

# UIの読み込み
if not os.path.exists(UI_PATH):
    raise FileNotFoundError(f'File not found: {UI_PATH}')
with open(UI_PATH, 'r') as f:
    html = f.read()

class WebSocketNode(Node):
    def __init__(self):
        super().__init__('web_socket_node')
        self.send_data = ''
        self.pub = self.create_publisher(String, 'web_socket_pub', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'estimated_position', self.callback, 10)

        @app.get("/")
        async def get():
            return HTMLResponse(html)

        @app.websocket('/ws')
        async def websocket_endpoint(websocket: FastAPIWebSocket):
            await websocket.accept()
            try:
                while True:
                    receive_data = await websocket.receive_text()
                    msg = String()
                    msg.data = receive_data
                    self.pub.publish(msg)

                    string_send_data = ",".join(map(str, self.send_data))
                    await websocket.send_text(string_send_data)
            except Exception as e:
                print(f'WebSocket error: {str(e)}')

    def callback(self, sub_msg):
        self.send_data = sub_msg.data

def run_ros2():
    rclpy.init()
    node = WebSocketNode()
    rclpy.spin(node)
    rclpy.shutdown()

def run_fastapi():
    uvicorn.run(app, host=IP_ADDRESS, port=PORT)

def main():
    ros2_thread = threading.Thread(target=run_ros2)
    ros2_thread.start()

    fastapi_thread = threading.Thread(target=run_fastapi)
    fastapi_thread.start()

    ros2_thread.join()
    fastapi_thread.join()

if __name__ == '__main__':
    main()
```

### WebSocket の利用例

1. クライアント（スマートフォン）から`ws://<PCのIP>:8000/ws`に接続．
2. JSON 形式のコマンドを送信.
3. WebSocket ノードが ROS2 のトピックにデータをパブリッシュ.
4. ROS2 ノードが処理し,ロボットを制御.
5. ロボットの自己位置データを WebSocket 経由でスマートフォンに送信.

## `R1_UI.txt`（UI ファイル）の作成方法

`R1_UI.txt` は Web ページの UI を記述した HTML ファイルです.このファイルはスマートフォンや PC のブラウザからアクセスし,ボタンやジョイスティックを操作することでロボットを制御するために使用します.

### **UI ファイルの基本構造**

UI は HTML + CSS で作成されており,主要な要素は以下の通りです.

例

```html
<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Robot Controller</title>
    <style>
      body {
        background: black;
        color: white;
        font-family: Arial, sans-serif;
        text-align: center;
      }

      .button {
        display: inline-block;
        width: 100px;
        height: 50px;
        margin: 10px;
        background-color: blue;
        color: white;
        font-size: 20px;
        border: none;
        cursor: pointer;
      }

      .button:hover {
        background-color: darkblue;
      }

      .joystick-container {
        position: absolute;
        bottom: 20px;
        left: 50%;
        transform: translateX(-50%);
      }
    </style>
  </head>
  <body>
    <h1>Robot Controller</h1>
    <button class="button" onclick="sendCommand('1')">Move 1</button>
    <button class="button" onclick="sendCommand('2')">Move 2</button>
    <button class="button" onclick="sendCommand('3')">Move 3</button>

    <div class="joystick-container">
      <input type="range" id="joystickX" min="0" max="200" value="100" />
      <input type="range" id="joystickY" min="0" max="200" value="100" />
      <button class="button" onclick="sendJoystick()">Send Joystick</button>
    </div>

    <script>
      const ws = new WebSocket("ws://192.168.98.216:8010/ws");

      ws.onmessage = function (event) {
        console.log("Received: " + event.data);
      };

      function sendCommand(command) {
        ws.send(command);
      }

      function sendJoystick() {
        let lx = document.getElementById("joystickX").value;
        let ly = document.getElementById("joystickY").value;
        let message = `0,1,0,0,0,0,0,${lx},${ly},100,100`;
        ws.send(message);
      }
    </script>
  </body>
</html>
```

### **UI ファイルの役割**

- **ボタン制御**：ボタンを押すと,特定の命令（例: `Move 1`）が WebSocket 経由で送信されます.
- **ジョイスティック制御**：スライダーで値を変更し,ジョイスティックの X,Y 値を送信することで,ロボットの移動を制御できます.

以下に堀君が作製したリポジトリを示す．詳しくは[こちら](https://github.com/SkenHub/ros2-websocket-link)を参照せよ．

# 制御層
ロボットを自律的に移動させるには，目標地点へ向かうだけでなく，安全かつスムーズに移動するための **経路計画** や **追従制御**，**PID制御** が必要になる．

本資料では，以下のトピックについて詳しく解説する

- **経路計画（現在位置 → 目標位置）**
- **追従制御（経路に対しての追従）**
  - 速度制御（最大速度・加速度制限）
  - 方向制御（回転角度の計算）
- **位置に対するPID制御**
一番実装が容易である．


## **経路計画**

### **(1) 障害物がない場合**

最も基本的な経路計画は，「現在位置から目標位置までの直線移動」 である．

#### **計算方法**

ロボットの現在位置を \((x_0, y_0)\)，目標位置を \((x_t, y_t)\) とすると，目標方向 \(\theta\) は以下の式で計算できる

$$
\theta = \tan^{-1}\left( \frac{y_t - y_0}{x_t - x_0} \right)
$$

移動距離 \(d\) は：

$$
d = \sqrt{(x_t - x_0)^2 + (y_t - y_0)^2}
$$

ロボットの速度 \(v\) は，最大速度 \(v_{max}\) を超えないように制限する：

$$
v = \min(v_{max}, d / t)
$$

ここで \(t\) は移動にかかる時間である．

#### **計算例**

現在位置：\((0, 0)\)，目標位置：\((1000, 500)\) のとき，

$$
\theta = \tan^{-1} \left( \frac{500}{1000} \right) = 26.57°
$$

$$
d = \sqrt{1000^2 + 500^2} = 1118.03 \text{ mm}
$$

### **障害物がある場合**

障害物がある場合，直線移動では衝突するため，障害物を避けながら目標地点へ移動する必要がある．そのための経路計画アルゴリズムとして以下がある

#### **① A**\*（A-star）アルゴリズム\*\*

A\*アルゴリズムは，グリッドマップ上で最短経路を求める代表的な探索アルゴリズムである．

- **手順**

  1. スタート地点から目標地点までのグリッドを生成する．
  2. 障害物のあるグリッドを無効にする．
  3. 各グリッドに対して，スタート地点からの移動コスト \(g\) と，ゴールまでの推定コスト \(h\) を計算し，評価関数 \(f = g + h\) を算出する．
  4. 最小の \(f\) を持つグリッドを選択し，ゴールに到達するまで繰り返す．

- **短所**：

  - 計算コストが高い
  - グリッドの解像度が低いと最適経路にならない

#### **② Dynamic Window Approach（DWA）**

DWAは，ロボットが障害物を回避しながらリアルタイムで動的に経路を生成する手法である．

- **手順**

  1. 現在のロボットの速度と角速度をもとに，次の動作候補（速度・角速度）を生成する．
  2. 各候補に対して，目標への到達度，障害物との距離，滑らかさを評価し，最適なものを選択する．
  3. これを繰り返しながら移動する．

- **短所**：

  - 事前にコスト関数のチューニングが必要
  - 近視的な判断をするため，大局的な最適経路にならないことがある

#### **③ 事前に決めた座標に向かって移動**

地図が既知の場合，事前に安全な中間座標を決めておき，その座標を順番に移動する方法がある．

- **手順**

  1. 事前に障害物を回避できる中間地点を定める（例：\(P_1, P_2, P_3\)）
  2. 現在位置から \(P_1\) に向かって移動する
  3. \(P_1\) に到達したら，次の目的地 \(P_2\) に移動する
  4. 最終目的地に到達するまで繰り返す

- **短所**：

  - 事前に環境を知っている必要がある
  - 障害物の変化に対応しづらい

## **位置に対するPID制御**

PID制御は，目標位置に対する偏差を最小化するために用いられる．

- **比例（P）制御**：位置の偏差に比例した制御入力を出力する
- **積分（I）制御**：過去の偏差を考慮して誤差を補正する
- **微分（D）制御**：未来の誤差変化を予測し，急激な変化を抑制する

PID制御の式：

$$
 u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}
$$

ここで，\(e(t)\) は現在の偏差，\(K_p, K_i, K_d\) はPIDゲインである．

PID制御は，障害物がない移動（直線移動）や，事前に決めた座標への移動に適用される．
以下に、位置制御に対するPID制御の実装例を示す。  
このPythonコードでは、現在のロボットの座標\((X, Y, \Theta)\) と目標座標を入力として、PID制御を適用し、速度\(V_x, V_y, \omega\) を出力する。

### **位置制御のためのPID制御（Python実装例）**
```python
import time
import math

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt=0.1):
        """
        PID制御の初期化
        Kp: 比例ゲイン
        Ki: 積分ゲイン
        Kd: 微分ゲイン
        dt: 制御周期（秒）
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        """
        PID制御計算
        error: 現在の偏差
        """
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class PositionPIDController:
    def __init__(self, Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular, dt=0.1):
        """
        位置制御のためのPID制御器
        Kp_linear, Ki_linear, Kd_linear: 並進方向（X, Y）のPIDゲイン
        Kp_angular, Ki_angular, Kd_angular: 角速度（ω）のPIDゲイン
        dt: 制御周期（秒）
        """
        self.pid_x = PIDController(Kp_linear, Ki_linear, Kd_linear, dt)
        self.pid_y = PIDController(Kp_linear, Ki_linear, Kd_linear, dt)
        self.pid_theta = PIDController(Kp_angular, Ki_angular, Kd_angular, dt)
        self.dt = dt

    def compute_control(self, current_pos, target_pos, max_speed=500, max_omega=30):
        """
        現在位置から目標位置への制御を計算
        current_pos: [x, y, theta] 現在の座標（mm, mm, deg）
        target_pos: [x, y, theta] 目標座標（mm, mm, deg）
        max_speed: 最大速度 [mm/s]
        max_omega: 最大角速度 [deg/s]
        """

        x_error = target_pos[0] - current_pos[0]
        y_error = target_pos[1] - current_pos[1]
        theta_error = (target_pos[2] - current_pos[2] + 360) % 360  # 目標角度との差分
        if theta_error > 180:
            theta_error -= 360  # -180 ~ 180 に正規化

        # PID制御による速度計算
        Vx = self.pid_x.compute(x_error)
        Vy = self.pid_y.compute(y_error)
        omega = self.pid_theta.compute(theta_error)

        # 最大速度制限
        speed = math.sqrt(Vx**2 + Vy**2)
        if speed > max_speed:
            scale = max_speed / speed
            Vx *= scale
            Vy *= scale

        # 最大角速度制限
        omega = max(-max_omega, min(max_omega, omega))

        return Vx, Vy, omega

# テスト用
if __name__ == "__main__":
    # PIDゲイン設定（適宜チューニング）
    Kp_linear = 0.5
    Ki_linear = 0.01
    Kd_linear = 0.1

    Kp_angular = 1.0
    Ki_angular = 0.01
    Kd_angular = 0.2

    controller = PositionPIDController(Kp_linear, Ki_linear, Kd_linear, Kp_angular, Ki_angular, Kd_angular)

    # 現在位置と目標位置
    current_position = [0, 0, 0]  # (X, Y, Theta)
    target_position = [1000, 500, 45]  # (X, Y, Theta)

    for _ in range(50):  # 50回更新
        Vx, Vy, omega = controller.compute_control(current_position, target_position)
        print(f"Vx: {Vx:.2f}, Vy: {Vy:.2f}, Omega: {omega:.2f}")

        # 仮のシミュレーション（現在位置を更新）
        current_position[0] += Vx * 0.1  # 0.1s ごとの移動
        current_position[1] += Vy * 0.1
        current_position[2] += omega * 0.1
        current_position[2] %= 360  # 角度を 0-360 に正規化

        time.sleep(0.1)
```

### **実装のポイント**
- **PID制御をX, Y, θ（角度）それぞれに適用**  
  - X, Y は並進方向の制御
  - θ はロボットの向きを制御する角速度

- **エラー（目標との差分）に対してPID計算を適用**
  - 直線移動時：\(V_x, V_y\) を調整して移動
  - 回転移動時：\(\omega\) を調整して向きを合わせる

- **速度制限の適用**
  - 平均速度が上限を超えないようにスケーリング
  - 角速度も最大値を設定して制限

### **計算例**
#### **初期状態**
- 現在位置：\((0,0,0)\)
- 目標位置：\((1000,500,45)\)

#### **実行後の出力例**
```py
Vx: 250.00, Vy: 125.00, Omega: 10.00
Vx: 200.00, Vy: 100.00, Omega: 8.00
Vx: 150.00, Vy: 75.00, Omega: 6.00
...
Vx: 0.00, Vy: 0.00, Omega: 0.00
```
このように、ロボットは目標地点に向かって減速しながら移動する.


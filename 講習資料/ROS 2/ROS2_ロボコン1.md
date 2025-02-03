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

### 1. システム全体の概要

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

```python
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from fastapi import FastAPI, WebSocket
import uvicorn

app = FastAPI()

class WebSocketNode(Node):
    def __init__(self):
        super().__init__('web_socket_node')
        self.pub = self.create_publisher(String, 'web_socket_pub', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'robot_position', self.callback, 10)
        self.send_data = ""

    def callback(self, msg):
        self.send_data = ','.join(map(str, msg.data))

@app.websocket('/ws')
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    node = WebSocketNode()
    try:
        while True:
            receive_data = await websocket.receive_text()
            msg = String()
            msg.data = receive_data
            node.pub.publish(msg)
            await websocket.send_text(node.send_data)
    except Exception as e:
        print(f'WebSocket error: {str(e)}')

# FastAPIサーバーの実行関数
def run_fastapi():
    uvicorn.run(app, host='0.0.0.0', port=8000)

# メイン関数
def main():
    rclpy.init()
    ros2_thread = threading.Thread(target=rclpy.spin, args=(WebSocketNode(),))
    ros2_thread.start()
    run_fastapi()
    ros2_thread.join()

if __name__ == '__main__':
    main()
```

### WebSocket の利用例

1. クライアント（スマートフォン）から`ws://<PCのIP>:8000/ws`に接続．
2. JSON 形式のコマンドを送信.
3. WebSocket ノードが ROS2 のトピックにデータをパブリッシュ.
4. ROS2 ノードが処理し,ロボットを制御.
5. ロボットの自己位置データを WebSocket 経由でスマートフォンに送信.

以下に堀君が作製したリポジトリを示す．詳しくはこちらを参照せよ．

https://github.com/SkenHub/ros2-websocket-link

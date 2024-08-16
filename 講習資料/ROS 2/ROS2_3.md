# ROS 2講習 第二回
-サービス通信のプログラム-
### 4S altair

参考文献:ROS2とPythonで作って学ぶAIロボット入門

---

## 環境
* python 3.10 
* ThinkPad L380 ubuntu22.04.3tls
* ROS2 humble
---

# サービス通信プログラムの作り方
双方向・同期通信で要求があったときのみメッセージを送受信する．
ソードがサービスを使って通信するときは，`クライアントノード`が`サービスノード`（サーバー）に`リクエストメッセージ`を送り，サービスノードはそれを処理してクライアントノードに`レスポンスメッセージ`を送る
![Alt text](images/image.png)
* 1.サービス型
* 2.サービスノード
* 3.クライアントノード

## 1. サービス型
* ### サービスメッセージ型（サービス型）
サービス通信に使うメッセージ型のこと．　サービス通信もトピック通信と同じくメッセージ型と同じ名前でないと通信できない.
サービス通信の標準サービス型は少ないため，一般的にはカスタムサービス型（自作）を使う．

カスタムサービス型の定義
```py
#リクエストの型と変数名を一行づつ書く
型1　変数名1
---
#レスポンスの型と変数名を一行づつ書く
型2　変数名2
型3　変数名3
```
---
サービス定義ファイル(拡張子 .srv)の例を以下に示す
* StringCommand.srv
```py
string command
---
string answer
```
> この例ではstring（文字）型を定義している

---
## 2. サービスノード
* ### サービスノードの作り方
1. サービスの生成
`create_service`（サービス型，サービス名，コールバック）でサービスを生成する．
サービス型とクライアントと同じでなければ通信できない．
* `サービス型`:サービス通信に使うメッセージ型
* `サービス名`:サービス通信に使うサービス名
* `コールバック`:サービスノードから届いたメッセージの処理を書く．
2. コールバックの定義
クライアントノードから届いたメッセージの処理を書く．

サービスノード例
```py
import time
import rclpy
from rclpy.node import Node
from airobot_interfaces.srv import StringCommand

class BringmeService(Node):  # ハッピーサービスクラス
    def __init__(self):  # コンストラクタ
        super().__init__('bringme_service')
        # サービスの生成（サービス型，サービス名, コールバック関数)
        self.service = self.create_service(
            StringCommand, 'command',self.callback)
        self.food = ['apple', 'banana', 'candy']   

    def callback(self, request, response):  # コールバック関数
        time.sleep(5)
        for item in self.food:
            if item in request.command:
                response.answer = 'はい，これです．'
                return response
        response.answer = '見つけることができませんでした．'
        return response


def main():  # main関数
    rclpy.init()
    node = BringmeService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+CLが押されました．")
    finally:
        rclpy.shutdown()
  
```

## 3. クライアントノード
* ### クライアントノードの作り方
1. クライアントの生成
`create_client`（サービス型，サービス名）でクライアントを生成する．
サービス型とクライアントと同じでなければ通信できない．
* `サービス型`:サービス通信に使うメッセージ型
* `サービス名`:サービス通信に使うサービス名
2. サービスができるようになるまで待機
`wait_for_service`(timeout_sec=秒数)をwhile文で使いサービスが使えるようになるまで待機する．
3. リクエストのインスタンス生成
4. リクエストの値に代入
5. サービスのリクエスト
`call_async`（リクエスト）を呼び出すことで，リクエストをサービスに送り結果を取得する．

クライアントノード例
```py
import rclpy
from rclpy.node import Node
from airobot_interfaces.srv import StringCommand


class BringmeClient(Node):
    def __init__(self):
        super().__init__('bringme_client_node')
        self.client = self.create_client(StringCommand, 'command') # クライアントの生成
        # サービスが利用できるまで待機
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービスは利用できません．待機中...')        
        self.request = StringCommand.Request()  # リクエストのインスタンス生成

    def send_request(self, order):
        self.request.command = order  # リクエストに値の代入   
        self.future = self.client.call_async(self.request) # サービスのリクエスト


def main(args=None):
    rclpy.init(args=args)
    node = BringmeClient()
    order = input('何を取ってきますか：')
    node.send_request(order)

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():  # サービスの処理が終了したら
            try:
                response = node.future.result()  # サービスの結果をレスポンスに代入                  
            except Exception as e:
                node.get_logger().info(f"サービスのよび出しは失敗しました．{e}")
            else:                
                node.get_logger().info( # 結果の表示
                    f"\nリクエスト:{node.request.command} -> レスポンス: {response.answer}")
                break  
    rclpy.shutdown()
```


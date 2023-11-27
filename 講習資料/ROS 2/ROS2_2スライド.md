---
marp: true
---
<br> 
<br> 
<br> 
<br> 
<br> 


# ROS 2講習 第二回
### 4S 野口 史遠
<br> 
<br> 
<br> 
<br> 
参考文献:ROS2とPythonで作って学ぶAIロボット入門

---
# ROS2プログラムの処理の流れ
1. モジュールのインポート
* ROS2でpythonプログラムを作るには`rclpy`をインポートする必要がある．
2. 初期化
* `rclpy.init() `:ROS2通信のための初期化
3. ノードの作成
* nodeクラスのインスタンス化：Nodeクラスを継承してクラスを作り,そのインスタンスをさくせいすることでノードを作成する．
4. ノード処理
* `rclpy.spin()`:繰り返し処理可能　`rclpy.spin_once()`:処理を１回実行
5. 終了処理
* `rclpy.shutdown()`:終了処理します．

---
# ROS2プログラミング
ROS2では基本的にクラスを使ってプログラミングします．
```py
import rclpy #1.ROS2 pythonモジュールの呼び出し
from rclpy.node import Node #rclpy.nodeモジュールからNodeクラスをインポート
class AltairNode(Node): #AltairNodeクラス
 def __init__(self):#コンストラクト
  print('ノードの生成')
  super().__init__('Altair_Node') #基底クラスコンストラクタの呼び出し
  self.get_logger().info('アルタイル参上')#4.ノードの処理
def main():#メイン関数
 print('プログラムスタート')
 rclpy.init() #2.初期化
 node = AltairNode() #3.ノード生成
 rclpy.shutdown() #5.終了処理
 print('プログラム終了')
```

![Alt text](image-7.png)

---

* インポート (1-2行目): 1行目のrcipy は ROS2のPython モジュールなので必ずインポートし なければなりません、2行目はノードをつくるために必要でrclpy node モジュールから Node クラ スをインポートします。この2行は常に必要です
* クラスの定義 (5~9行目): AltairNodeクラスを定義しています.コンストラクタの8行目で基底クラスNodeのコンストラクタを呼び出すことでノードを生成しています.引数 Altair_Node はノード名です。9行目のget_logger().info()はノードのメソッドでログ(log)情報(この例ではアルタイル参上)を端末に表示します．print文とは違い、端末だけでなくROS2のアプリrqt_console でも読むことができます。
* main()関数(12~17行目): このプログラムは main()関数から実行されるので、main()関数が前節で説明したsetup.pyのエントリポイント(開始点)です
* relpy.init()(14行目):rclpy.init()でROS2通信を初期化します.ノードをつくる前に呼び出さなければいけません。
* クラスのインスタンス化 (15行目): AltairNodeクラスのインスタンスnodeを生成しています.
* rclpy.shutdown() (16行目): relpy.shutdown() で終了処理をしています.

---
# コールバックを使ったプログラム
ROS2では`コールバック関数`や`コールバックメソッド`を多用してっプログラムを作る．
* コールバック関数：プログラム中で，呼び出し先の関数の実行中に実行されるように，あらかじめ指定しておく関数

→マウスで線を書くなどの処理を実装するとき

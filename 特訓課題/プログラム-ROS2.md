### pythonについて
一応、講習資料にpythonを上げていますが難易度が少し高いです。

### ROS2(humble)について
資料を講習資料のROS2に上げていますのでご覧ください。これだけでいけます。
なお、環境構築については記述していません。

# 課題1:pythonコードを作成せよ
フィボナッチ数列を羅列させるプログラム書け　
最大値10

```python
# 実行結果
0,1,1,2,3,5,8
```

### 提出方法
自身のGITHUBにあげURLを送ってください。

# 課題2:pythonコードを作成せよ
1から10までの整数をループで表示し、偶数の場合には"偶数"と表示し、奇数の場合には"奇数"と表示するプログラムを書いてください.

```py
# 実行結果
1:奇数,2:偶数,3:奇数,4:偶数,5:奇数,6:偶数,7:奇数,8:偶数,9:奇数,10:偶数
```

### 提出方法
自身のGITHUBにあげURLを送ってください。

# 課題3:pythonコードを作成せよ
pythonでクラス用いたプログラムを作成せよ
継承も行うこと。

例
```PY
# 基本の動物クラス
class Animal:
    def __init__(self, name, age):
        self.name = name
        self.age = age
    
    def make_sound(self):
        raise NotImplementedError("This method should be overridden by subclasses")
    
    def __str__(self):
        return f"{self.name} is {self.age} years old"

# 犬クラス（Animalクラスを継承）
class Dog(Animal):
    def __init__(self, name, age, breed):
        super().__init__(name, age)
        self.breed = breed
    
    def make_sound(self):
        return "Woof"
    
    def __str__(self):
        return f"{self.name} is a {self.age}-year-old {self.breed}"

# 猫クラス（Animalクラスを継承）
class Cat(Animal):
    def __init__(self, name, age, color):
        super().__init__(name, age)
        self.color = color
    
    def make_sound(self):
        return "Meow"
    
    def __str__(self):
        return f"{self.name} is a {self.age}-year-old {self.color} cat"

# 動物のリストを作成
animals = [
    Dog("Buddy", 3, "Golden Retriever"),
    Cat("Whiskers", 2, "black"),
    Dog("Rex", 5, "German Shepherd"),
    Cat("Luna", 1, "white")
]

# 各動物の情報と音を表示
for animal in animals:
    print(animal)
    print(animal.make_sound())
    print()

```
### 提出方法
自身のGITHUBにあげURLを送ってください。
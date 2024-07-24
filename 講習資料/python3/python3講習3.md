---
marp: true
---

# python3講習

４S　Altair

---

# 第3回目
## クラス

---
## 環境
* python 3.9 ,3.10 ,3.11
* ThinkPad L380 ubuntu22.04.3tls
* ThinkPad P1 Gen 3 Windows10

---
# クラスとは

Pythonにおいて、クラスはオブジェクト指向プログラミング（OOP）の基本的な概念です。クラスはデータとそのデータを操作する関数（メソッド）を組み合わせたもので、オブジェクトの設計図として機能します。

---
## クラスの基本構文

```python
class MyClass:
    def __init__(self, param1, param2):
        self.param1 = param1
        self.param2 = param2

    def my_method(self):
        # メソッドの中身

# クラスのインスタンス化
my_instance = MyClass(arg1, arg2)
```
* `__init__`: インスタンスが作成される際に呼び出される特殊なメソッドで、初期化を行います
* `self`: インスタンス自体を指すキーワードです

---
# クラスの使用例
```py
class Car:
    def __init__(self, make, model, year):
        self.make = make
        self.model = model
        self.year = year
        self.odometer_reading = 0

    def get_info(self):
        return f"{self.year} {self.make} {self.model}"

    def read_odometer(self):
        return f"Odometer reading: {self.odometer_reading} miles"

    def update_odometer(self, mileage):
        if mileage >= self.odometer_reading:
            self.odometer_reading = mileage
        else:
            print("You can't roll back an odometer!")

# インスタンスの作成
my_car = Car("Toyota", "Camry", 2022)

# メソッドの呼び出し
print(my_car.get_info())  # 出力: 2022 Toyota Camry
print(my_car.read_odometer())  # 出力: Odometer reading: 0 miles

# 属性の変更
my_car.update_odometer(1000)

# 更新された属性の確認
print(my_car.read_odometer())  # 出力: Odometer reading: 1000 miles
```
---
# 継承
クラスは他のクラスから継承することができる．これにより、既存のクラスの機能を引き継ぎながら、新しい機能を追加可能

---
```py
class ElectricCar(Car):
    def __init__(self, make, model, year, battery_size):
        super().__init__(make, model, year)
        self.battery_size = battery_size

    def describe_battery(self):
        return f"Battery size: {self.battery_size} kWh"

# ElectricCarクラスのインスタンス化
my_electric_car = ElectricCar("Tesla", "Model S", 2023, 100)

# 親クラスのメソッドの呼び出し
print(my_electric_car.get_info())  # 出力: 2023 Tesla Model S

# 拡張されたメソッドの呼び出し
print(my_electric_car.describe_battery())  # 出力: Battery size: 100 kWh

```
ここでは、`ElectricCar`クラスが`Car`クラスを継承しており、新しい機能である`describe_battery`メソッドを追加している

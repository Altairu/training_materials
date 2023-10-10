---
marp: true
---

# python3講習

４S　野口史遠

---

# 第1回目
## 入門編

---
## 環境
* python 3.9 ,3.10 ,3.11
* ThinkPad L380 ubuntu22.04.3tls
* ThinkPad P1 Gen 3 Windows10

---
# インタープリンタの起動
通常`/usr/local/bin/python3.10`としてインストールされている

```py
pyhon3.10
```
と入力すると起動できる

```
Python 3.10.12 (main, Jun 11 2023, 05:26:28) [GCC 11.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>>
```
---

Windowsの場合
![picture 1](1.png)  

---

プライマプロント``>>>``が出てるときにunix[ctrl]+[D],win[ctrl]+[Z]を押すとインタープリターは０を返して終了する

```py
altair@altair-ThinkPad-L380:~$ python3.10
Python 3.10.12 (main, Jun 11 2023, 05:26:28) [GCC 11.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> 
altair@altair-ThinkPad-L380:~$ 
```

```py
altair@altair-ThinkPad-L380:~$ python3
Python 3.10.12 (main, Jun 11 2023, 05:26:28) [GCC 11.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> 
```
でも可能

---
# 対話モード

```py
altair@altair-ThinkPad-L380:~$ python3
Python 3.10.12 (main, Jun 11 2023, 05:26:28) [GCC 11.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> new_world = True
>>> if new_world:
...     print("はじめ")
... 
はじめ
>>> 
```
---

コメントアウト
```py

# 1つ目のコメント
one = 1 #2つ目のコメント
# そしてこれが3つ目!
text = "# これはコメントじゃない。"

```

---

## 数値
```py

>>> 2+2
4
>>> 50-5*6
20
>>> (50-5*6)/4
5.0
>>> 8/5
1.6
>>> 17/3 #float
5.666666666666667
>>> 17//3 #切り下げ除算は小数点以下を捨て
5
>>> 17%3 #あまり
2
>>> 5*3+2
17
>>> 5**2
25
>>> 2**7
128
>>> 4*3.75-1
14.0

```

---

```py

>>> width =20
>>> height =5*9
>>> width*height
900

```
等号は代入に使う
変数は定義されないままであるとエラーが出る

```py
>>> n
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
NameError: name 'n' is not defined
>>> 

```

---


対話モードでは、最後に表示した式を変数(アンダースコア)に代入してある。

```py
>>> tax = 12.5/100 
>>> price = 100.50
>>> price * tax
12.5625
>>> price + -
113.0625
>>> round(_. 2)
113.06
```
---

## 文字列

バックスラッシュ(\)でクォート文字のエスケー プができる 
```py
>>> 'spam eggs'# シングルクォート
'spam eggs'
>>> 'doesn\'t'# シングルクォートは\でエスケープするか...
"doesn't"
>>> "doesn't"#...ダブルクォートを使う
"doesn't"
>>> '"Yes," they said. '
'"Yes," they said.
>>> "\"Yes,\" they said."
'"Yes," they said."
>>> '"Isn\'t," they said.' 
"Isn\'t," they said.'
```
---
raw文字列
```py
>>> print('C:\some\name')# \n は改行なので
C:\some
ame
>> print(r'C:\some\name')#引用符の前のに注目
C:\some\name
```

列挙された文字列リテラル (引用符で囲まれたものたち) は自動的に連結される。
```py
>>> 'Py' 'thon" 
'Python'
```

この機能は長い文字列を分割したい時に便利
```py
>>> text = ('ながいながい文字列を' '入れておいて繋げてやろう。 ')
>>> text
ながいながい文字列を入れておいて繋げてやろう。
```
---

インデックス指定
```py
>>> word = 'Python' 
>>> word [0] # 位置 0のキャラクタ 
'P'
>>> word [5] #位置5のキャラクタ 
'n'
```
インデックスには負の数も使える。 

```py
>>> word[-1]# 最後のキャラクタ
'n'
>>> word [-2]#最後から2番目のキャラクタ
'o'
>>> word [-6]
'P'
```
---



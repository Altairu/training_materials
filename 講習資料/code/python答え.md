
---

### 課題1: 答え

```python
def fibonacci(max_value):
    sequence = []
    a, b = 0, 1
    while a <= max_value:
        sequence.append(a)
        a, b = b, a + b
    return sequence

print(fibonacci(10))
```

---

### 課題2: 答え

```python
def odd_even():
    for i in range(1, 11):
        if i % 2 == 0:
            print(f"{i}:偶数")
        else:
            print(f"{i}:奇数")

odd_even()
```

---

### 課題3: 答え

```python
def is_palindrome(s):
    return s == s[::-1]

word = input("文字列を入力してください: ")
if is_palindrome(word):
    print("回文です")
else:
    print("回文ではありません")
```

---

### 課題4: 答え

```python
def factorial(n):
    if n == 0:
        return 1
    else:
        return n * factorial(n - 1)

number = int(input("数を入力してください: "))
print(factorial(number))
```

---

### 課題5: 答え

```python
def multiples_of_3_and_5():
    result = [i for i in range(101) if i % 3 == 0 or i % 5 == 0]
    print(result)

multiples_of_3_and_5()
```

---

### 課題6: 答え

```python
def remove_duplicates(lst):
    return list(set(lst))

lst = [1, 2, 2, 3, 4, 4, 5]
print(remove_duplicates(lst))
```

---

### 課題7: 答え

```python
def count_vowels(sentence):
    vowels = 'aeiou'
    count = sum(1 for char in sentence if char.lower() in vowels)
    return count

sentence = input("文章を入力してください: ")
print(f"母音の数: {count_vowels(sentence)}")
```

---


# PID制御の説明

---

## 目次

1. PID制御とは
2. PIDの要素
3. PIDコントローラの実装
4. コード例の解説

---

## 1. PID制御とは

* PID制御は、Proportional-Integral-Derivative（比例-積分-微分）制御の略です。
* この制御アルゴリズムは、制御対象の状態を目標値に維持するために使用されます。

---

## 2. PIDの要素

PID制御には3つの要素があります。

1. **P（比例）制御**: 現在の誤差を使って制御信号を調整します。
2. **I（積分）制御**: 過去の誤差の累積を使って制御信号を調整します。
3. **D（微分）制御**: 誤差の変化率を使って制御信号を調整します。

---

## 3. PIDコントローラの実装

* PID制御は一般的に以下の式で表されます：

    ```
    Control Signal = Kp * P + Ki * I + Kd * D
    ```

* ここで、Kp、Ki、およびKdは制御アルゴリズムのパラメータで、P、I、およびDはそれぞれ比例、積分、および微分項です。

---

## 4. コード例の解説

以下は、Arduinoコードの一部を使用してPID制御を実装する例です。

```cpp
#include "motor.h"

const int encoderPinA = 2;  // A相
const int encoderPinB = 3;  // B相
volatile long encoderCount = 0;  // エンコーダーのカウンター
int degrees = 30;  // エンコーダーの角度

double Kp = 20;
double Ki = 0.01; 
double Kd = 0;

// ...（他のコード）

void pid() {
  static int previousError = 0; // 前回の誤差
  static int integral = 0; // 積分項
  int error = target - degrees; // 現在の誤差

  // P制御
  int P = Kp * error;

  // I制御（積分項）
  integral += error;
  int I = Ki * integral;

  // D制御（微分項）
  int derivative = error - previousError;
  int D = Kd * derivative;

  // PID制御の計算
  speed1 = P + I + D;

  // 誤差と積分項を更新
  previousError = error;
}
```
このコードは、エンコーダーの角度を制御対象とし、PID制御アルゴリズムを使用して目標値に制御します。

---

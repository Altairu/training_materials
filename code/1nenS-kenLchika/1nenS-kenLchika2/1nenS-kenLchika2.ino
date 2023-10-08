// 定数宣言
int ledPin = 9;        // LEDピン
int button1 = 3;       // ボタン1ピン
int button2 = 4;       // ボタン2ピン
int button3 = 5;       // ボタン3ピン       
double fadeAmount = 0.001;  // 明るさの増減量

void setup() {
  pinMode(ledPin, OUTPUT); // LEDのピンを出力モードに設定
  pinMode(button1, INPUT_PULLUP);// ボタン１のピンを入力モードに設定
  pinMode(button2, INPUT_PULLUP);// ボタン２のピンを入力モードに設定
  pinMode(button3, INPUT_PULLUP);// ボタン３のピンを入力モードに設定
  Serial.begin(9600);
}

void loop() {
  // ボタン1が押された場合、LED光る
  if (digitalRead(button1) == LOW) {
    digitalWrite(ledPin, HIGH);
  }
  // ボタン2が押された場合、LEDの明るさを徐々に上げる
  if (digitalRead(button2) == LOW) {
    for (double i = 0; i <= 255; i += fadeAmount) {
      analogWrite(ledPin, i);
    }
  }
// ボタン3が押された場合、ボタン2が押された場合、LEDの明るさを徐々に下げる
if (digitalRead(button3) == LOW) {
  digitalWrite(ledPin, LOW);
}


// シリアルモニタに各ボタンの状態を表示する
Serial.print("Button 1: ");
Serial.print(digitalRead(button1));
Serial.print(" | Button 2: ");
Serial.print(digitalRead(button2));
Serial.print(" | Button 3: ");
Serial.print(digitalRead(button3));
}
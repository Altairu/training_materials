// LEDピンの定義
int ledPin = 9;

// ボタンピンの定義
int buttonPin = 3;

void setup() {
  // LEDピンを出力に設定
  pinMode(ledPin, OUTPUT);

  // ボタンピンを入力に設定
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // ボタンが押された場合、LEDを点灯する
  if (digitalRead(buttonPin) == LOW) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

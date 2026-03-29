// Serial0 ループバックテスト
// Serial0のTXとRXを直結してから実行すること

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200, SERIAL_8E1);
  delay(2000);
  Serial.println("=== Serial0 Loopback Test ===");
  Serial.println("Serial0のTXとRXを直結してください");
  Serial.println();
}

void loop() {
  // Serial0から送信
  Serial0.write(0xAA);
  delay(50);

  // Serial0から受信できるか
  if (Serial0.available()) {
    int b = Serial0.read();
    Serial.printf("OK: sent 0xAA, received 0x%02X\n", b);
  } else {
    Serial.println("NG: no response from Serial0");
  }

  delay(1000);
}

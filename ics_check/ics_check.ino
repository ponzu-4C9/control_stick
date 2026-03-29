#include <IcsHardSerialClass.h>

const byte EN_PIN = 8;
const long BAUDRATE = 115200;
const int TIMEOUT = 25;
IcsHardSerialClass krs(&Serial0, EN_PIN, BAUDRATE, TIMEOUT);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== ICS Bus Check ===");

  krs.begin();
}

void loop() {
    // ID0のサーボを検出
  Serial.print("setSpd(0, 127) -> ");
  int r = krs.setSpd(0, 127);
  Serial.println(r);  // -1なら通信失敗

  Serial.print("setPos(0, 7500) -> ");
  r = krs.setPos(0, 7500);
  Serial.println(r);  // -1なら通信失敗

  Serial.print("getPos(0) -> ");
  r = krs.getPos(0);
  Serial.println(r);  // -1なら通信失敗

  Serial.println();

  // ID1のサーボを検出
  Serial.print("setSpd(1, 127) -> ");
  r = krs.setSpd(1, 127);
  Serial.println(r);

  Serial.print("setPos(1, 7500) -> ");
  r = krs.setPos(1, 7500);
  Serial.println(r);

  Serial.print("getPos(1) -> ");
  r = krs.getPos(1);
  Serial.println(r);

  Serial.println();
  Serial.println("=== Done ===");
  Serial.println("-1 = 通信失敗, それ以外 = 通信OK");

  delay(300);
}

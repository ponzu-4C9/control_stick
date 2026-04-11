#include <IcsHardSerialClass.h>

const byte EN_PIN = 8;
const long BAUDRATE = 115200;
const int TIMEOUT = 25;
IcsHardSerialClass krs(&Serial0, EN_PIN, BAUDRATE, TIMEOUT);

const int ELE_POT = 2;  // エレベータ用可変抵抗 (GPIO2) → ch1
const int RUD_POT = 3;  // ラダー用可変抵抗   (GPIO3) → ch0

void setup() {
  Serial.begin(115200);
  krs.begin();
  krs.setSpd(0, 127);  // ラダー
  krs.setSpd(1, 127);  // エレベータ
  pinMode(ELE_POT, INPUT);
  pinMode(RUD_POT, INPUT);
}

void loop() {
  int rawE = analogRead(ELE_POT);
  int rawR = analogRead(RUD_POT);
  int posE = map(rawE, 0, 4095, 3500, 11500);
  int posR = map(rawR, 0, 4095, 3500, 11500);
  int retR = krs.setPos(0, posR);  // ラダー
  int retE = krs.setPos(1, posE);  // エレベータ
  Serial.printf("rawE:%d rawR:%d posE:%d posR:%d retE:%d retR:%d\n",
                rawE, rawR, posE, posR, retE, retR);
  delay(20);
}
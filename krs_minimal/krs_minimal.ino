#include <IcsHardSerialClass.h>

const byte EN_PIN = 8;
const long BAUDRATE = 115200;
const int TIMEOUT = 25;
IcsHardSerialClass krs(&Serial0, EN_PIN, BAUDRATE, TIMEOUT);

const int POT_PIN = 2;  // 可変抵抗(GPIO2)

void setup() {
  Serial.begin(115200);
  krs.begin();
  krs.setSpd(0, 127);
  pinMode(POT_PIN, INPUT);
}

void loop() {
  int raw = analogRead(POT_PIN);
  int pos = map(raw, 0, 4095, 3500, 11500);
  int ret = krs.setPos(0, pos);
  Serial.printf("raw:%d pos:%d ret:%d\n", raw, pos, ret);
  delay(20);
}

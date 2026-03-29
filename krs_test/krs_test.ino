#include <IcsHardSerialClass.h>

const byte EN_PIN = 8;
const long BAUDRATE = 115200;
const int TIMEOUT = 25;
IcsHardSerialClass krs(&Serial0, EN_PIN, BAUDRATE, TIMEOUT);

bool streaming = false;
byte streamId = 0;
unsigned long t0 = 0;
unsigned long lastSample = 0;
const unsigned long SAMPLE_INTERVAL = 10;  // ms

void setup() {
  Serial.begin(115200);
  krs.begin();
  krs.setSpd(0, 127);
  krs.setSpd(1, 127);
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    char cmd = s.charAt(0);

    if (cmd == 's') {
      streaming = false;
      Serial.println("END");
    } else if (cmd == 'e' || cmd == 'r') {
      int val = s.substring(1).toInt();
      byte id = (cmd == 'e') ? 0 : 1;
      krs.setPos(id, val);
      Serial.println("ACK:" + String(cmd) + String(val));
      streamId = id;
      t0 = millis();
      lastSample = 0;
      streaming = true;
    }
  }

  if (streaming) {
    unsigned long elapsed = millis() - t0;
    if (elapsed - lastSample >= SAMPLE_INTERVAL) {
      lastSample = elapsed;
      int pos = krs.getPos(streamId);
      if (pos != -1) {
        Serial.println("D:" + String(elapsed) + "," + String(pos));
      }
    }
  }
}

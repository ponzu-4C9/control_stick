#include <IcsHardSerialClass.h>
#include <esp_now.h>
#include <WiFi.h>

const byte EN_PIN = 8;
const long BAUDRATE = 115200;
const int TIMEOUT = 25;
IcsHardSerialClass krs(&Serial0, EN_PIN, BAUDRATE, TIMEOUT);

// ESP-NOW受信ピッチ
volatile float receivedPitch = 0.0;
volatile bool pitchReceived = false;

struct PitchData {
  float pitch;
};

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
  if (data_len == sizeof(PitchData)) {
    PitchData pd;
    memcpy(&pd, data, sizeof(pd));
    receivedPitch = pd.pitch;
    pitchReceived = true;
  }
}

// ストリーミングモード
bool streaming = false;
byte streamId = 0;
unsigned long t0 = 0;
unsigned long lastSample = 0;
const unsigned long SAMPLE_INTERVAL = 10; // ms

// マッピングモード
bool mapping = false;
int mapStart = 3500;
int mapEnd = 11500;
int mapSteps = 50;
int mapCurrentStep = 0;
int mapCurrentTarget = 0;
byte mapServoId = 1;
unsigned long mapSettleStart = 0;
const int MAP_SETTLE_COUNT = 15;
const int MAP_SETTLE_THRESHOLD = 20;
int settleBuffer[15];
int settleIdx = 0;
bool settling = false;

void setup()
{
  Serial.begin(115200);
  krs.begin();
  krs.setSpd(0, 127);
  krs.setSpd(1, 127);

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  Serial.print("MAC:");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  static String inputBuffer = "";
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (inputBuffer.length() > 0)
      {
        inputBuffer.trim();
        char cmd = inputBuffer.charAt(0);

        if (cmd == 's')
        {
          streaming = false;
          mapping = false;
          Serial.println("END");
        }
        else if (cmd == 'e' || cmd == 'r')
        {
          int val = inputBuffer.substring(1).toInt();
          byte id = (cmd == 'e') ? 1 : 0;
          krs.setPos(id, val);
          Serial.printf("ACK:%c%d\n", cmd, val);
          streamId = id;
          t0 = millis();
          lastSample = 0;
          streaming = true;
          mapping = false;
        }
        else if (cmd == 'm')
        {
          // m<id>,<start>,<end>,<steps>  例: me,3500,11500,50
          char servoChar = inputBuffer.charAt(1);
          mapServoId = (servoChar == 'e') ? 1 : 0;

          // カンマ区切りでパラメータ取得
          int idx1 = inputBuffer.indexOf(',');
          int idx2 = inputBuffer.indexOf(',', idx1 + 1);
          int idx3 = inputBuffer.indexOf(',', idx2 + 1);

          if (idx1 > 0 && idx2 > 0 && idx3 > 0) {
            mapStart = inputBuffer.substring(idx1 + 1, idx2).toInt();
            mapEnd = inputBuffer.substring(idx2 + 1, idx3).toInt();
            mapSteps = inputBuffer.substring(idx3 + 1).toInt();
          }

          Serial.printf("ACK:MAP,%d,%d,%d,%d\n", mapServoId, mapStart, mapEnd, mapSteps);
          mapping = true;
          streaming = false;
          mapCurrentStep = 0;
          startMapStep();
        }
        inputBuffer = "";
      }
    }
    else
    {
      inputBuffer += c;
    }
  }

  // 通常ストリーミング
  if (streaming)
  {
    unsigned long elapsed = millis() - t0;
    if (elapsed >= lastSample)
    {
      lastSample += SAMPLE_INTERVAL;
      int pos = krs.getPos(streamId);
      if (pos != -1)
      {
        float pitch = receivedPitch;
        Serial.printf("D:%lu,%d,%.2f\n", elapsed, pos, pitch);
      }
    }
  }

  // マッピングモード
  if (mapping && settling)
  {
    unsigned long elapsed = millis() - mapSettleStart;
    if (elapsed >= (unsigned long)(settleIdx * SAMPLE_INTERVAL))
    {
      int pos = krs.getPos(mapServoId);
      if (pos != -1)
      {
        if (settleIdx < MAP_SETTLE_COUNT) {
          settleBuffer[settleIdx] = pos;
          settleIdx++;
        }

        if (settleIdx >= MAP_SETTLE_COUNT) {
          // 安定判定
          int minVal = settleBuffer[0], maxVal = settleBuffer[0];
          for (int i = 1; i < MAP_SETTLE_COUNT; i++) {
            if (settleBuffer[i] < minVal) minVal = settleBuffer[i];
            if (settleBuffer[i] > maxVal) maxVal = settleBuffer[i];
          }

          if (maxVal - minVal < MAP_SETTLE_THRESHOLD) {
            // 安定した: ピッチを記録
            float pitch = receivedPitch;
            Serial.printf("MAP:%d,%.2f\n", mapCurrentTarget, pitch);

            mapCurrentStep++;
            if (mapCurrentStep >= mapSteps) {
              mapping = false;
              settling = false;
              Serial.println("MAP_DONE");
            } else {
              startMapStep();
            }
          } else {
            // まだ安定していない: リセットして再チェック
            settleIdx = 0;
            mapSettleStart = millis();
          }
        }
      }
    }

    // 10秒タイムアウト
    if (millis() - mapSettleStart > 10000) {
      float pitch = receivedPitch;
      Serial.printf("MAP:%d,%.2f\n", mapCurrentTarget, pitch);
      mapCurrentStep++;
      if (mapCurrentStep >= mapSteps) {
        mapping = false;
        settling = false;
        Serial.println("MAP_DONE");
      } else {
        startMapStep();
      }
    }
  }
}

void startMapStep()
{
  float stepSize = (float)(mapEnd - mapStart) / (float)(mapSteps - 1);
  mapCurrentTarget = mapStart + (int)(stepSize * mapCurrentStep);
  krs.setPos(mapServoId, mapCurrentTarget);
  settling = true;
  settleIdx = 0;
  mapSettleStart = millis();
}

#include <IcsBaseClass.h>
#include <IcsHardSerialClass.h>
#include <freertos/FreeRTOS.h>
#include <Preferences.h>

Preferences preferences;
TaskHandle_t nvmTaskHandle = NULL;
bool settingsChanged = false;

const byte EN_PIN = 8; //サーボ
const long BAUDRATE = 115200;
const int TIMEOUT = 25;
const int r_elevator = 2; //可変抵抗エレベーター
const int r_rudder = 3;   //可変抵抗ラダー
const int trimE = 4; //エレベータートリムスイッチ
const int LED = 5;
const int resetPin = 6; // NVMリセット用ピンを変更 (GPIO 0番ピンをGNDとショートでリセット)
const int trimR1 = 9;  //トリムラダー
const int trimR2 = 10; //トリムラダー
int ServoDegreeElevatorMax = 11500;
int ServoDegreeElevatorMin = 3500;
int ServoDegreeRudderMax = 11500;
int ServoDegreeRudderMin = 3500;
IcsHardSerialClass krs(&Serial0,EN_PIN,BAUDRATE,TIMEOUT); //インスタンス＋ENピン(8番ピン)およびUARTの指定

// 変数の宣言
int Trimelevetor = 0;
int neutralTrimeEle = 0;
int Trimrudder = 0;
int is_pid = 0;//今PID制御をONにするかどうか(0か1)

// PID制御の状態
struct PidState {
  float kp, ki, kd;
  float integral;
  float lastError;
  unsigned long lastTime;
};
PidState pidElevator;
PidState pidRudder;
float lastOutputE = 7500;
float lastOutputR = 7500;

void pidInit(PidState* state, float kp, float ki, float kd) {
  state->kp = kp;
  state->ki = ki;
  state->kd = kd;
  state->integral = 0;
  state->lastError = 0;
  state->lastTime = millis();
}

void pidReset(PidState* state) {
  state->integral = 0;
  state->lastError = 0;
  state->lastTime = millis();
}

float pidCompute(PidState* state, float setpoint, float measured) {
  unsigned long now = millis();
  float dt = (now - state->lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  float error = setpoint - measured;
  state->integral += error * dt;
  float derivative = (error - state->lastError) / dt;

  float output = state->kp * error + state->ki * state->integral + state->kd * derivative;

  state->lastError = error;
  state->lastTime = now;

  return measured + output;
}

// 設定を読み込む関数
void loadSettings() {
  preferences.begin("trim-data", true);
  Trimelevetor = preferences.getInt("trimE", 0);
  neutralTrimeEle = preferences.getInt("neutE", 0);
  Trimrudder = preferences.getInt("trimR", 0);
  preferences.end();
  Serial.printf("Settings Loaded: E=%d, neutE=%d, R=%d\n", Trimelevetor, neutralTrimeEle, Trimrudder);
}

// 非同期でFlashに保存するタスク
void nvmTask(void *pvParameters) {
  while(1) {
    // 通知が来るまで待機
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    preferences.begin("trim-data", false);
    preferences.putInt("trimE", Trimelevetor);
    preferences.putInt("neutE", neutralTrimeEle);
    preferences.putInt("trimR", Trimrudder);
    preferences.end();
    Serial.println(">> Settings saved to NVM");
  }
}

// リセットピンを常時監視するタスク
void resetMonitorTask(void *pvParameters) {
  
  while(1) {
    if (digitalRead(resetPin) == HIGH) {
      // チャタリング防止の少しの待機
      vTaskDelay(50 / portTICK_PERIOD_MS);
      
      if (digitalRead(resetPin) == HIGH) {
        Serial.println("!!! NVM Reset Triggered !!!");
        preferences.begin("trim-data", false);
        preferences.clear();
        preferences.end();
        
        // メモリ上の値も初期化
        Trimelevetor = 0;
        neutralTrimeEle = 0;
        Trimrudder = 0;

        // リセット成功の合図としてLEDを高速点滅(5回)
        for(int i=0; i<5; i++) {
          digitalWrite(LED, HIGH); 
          vTaskDelay(100 / portTICK_PERIOD_MS);
          digitalWrite(LED, LOW);  
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
        // ボタンが離されるまで待機（無限リセットループを防止）
        while(digitalRead(resetPin) == HIGH) {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
      }
    }
    // 100msごとにボタンの状態をチェック
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void Ltika(void *pvParameters){
  for(int i = 0;i < 3;i++){
    digitalWrite(LED, HIGH);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

int elevetor = 0;
int rudder = 0;
void Potentiometer(){
  int R_elevetor = analogRead(r_elevator);
  int R_rudder = analogRead(r_rudder);
  elevetor = map(R_elevetor,0,4095,ServoDegreeElevatorMin,ServoDegreeElevatorMax);     //neutral:7500
  rudder = map(R_rudder,0,4095,ServoDegreeRudderMin,ServoDegreeRudderMax);         //neutral:7500
}

unsigned long lastPushed = millis();//ボタン4チャタリング防止用
int is_buttun3 = 0;//ボタン3長押し検知用
int lastTrimE = 0;

void trimElevetor(){
  int TrimE = analogRead(trimE);
  
  if(0 <= TrimE && TrimE <=100){    //優先度1
    Trimelevetor += 3;
    settingsChanged = true;

  }else if(1000 <= TrimE && TrimE <=1200){  // 優先度2
    Trimelevetor -= 3;
    settingsChanged = true;

  }else if(2000 <= TrimE && TrimE <= 2300){  //優先度3
    is_buttun3 += 1;
    if(is_buttun3 > 20){
      neutralTrimeEle = Trimelevetor;
      is_buttun3 = 0;
      settingsChanged = true;
      xTaskCreate(Ltika, "Ltika", 1024, NULL, 9, NULL);
    }
    
  }else if(!(2000 <= TrimE && TrimE <= 2300) && (2000 <= lastTrimE && lastTrimE <= 2300)){
    Trimelevetor = neutralTrimeEle;
    settingsChanged = true;
    
  }else if(3300 <= TrimE && TrimE <= 3500 && millis() - lastPushed > 250){   //優先度4
    is_pid = !is_pid;
    lastPushed = millis();
    settingsChanged = true;
  }else{
    is_buttun3 = 0;
    // ボタンが離された瞬間に変更があれば保存
    if (settingsChanged && nvmTaskHandle != NULL) {
      xTaskNotifyGive(nvmTaskHandle);
      settingsChanged = false;
    }
  }
  lastTrimE = TrimE;
}

int lastTrimR1 = LOW;
int lastTrimR2 = LOW;
void trimRudder(){
    int nowTrimR1 = digitalRead(trimR1);
    int nowTrimR2 = digitalRead(trimR2);

    if (nowTrimR1 == HIGH && lastTrimR1 == LOW) {
      Trimrudder += 10;
      if (nvmTaskHandle != NULL) xTaskNotifyGive(nvmTaskHandle);
    }
    if (nowTrimR2 == HIGH && lastTrimR2 == LOW) {
      Trimrudder -= 10;
      if (nvmTaskHandle != NULL) xTaskNotifyGive(nvmTaskHandle);
    }
    lastTrimR1 = nowTrimR1;
    lastTrimR2 = nowTrimR2;
}

const long frequency = 30; 
void mainloop(void *pvParameters) {
  while(1){
    Potentiometer();
    trimElevetor();
    trimRudder();
    int ServoDegreeE = elevetor + Trimelevetor;
    int ServoDegreeR = rudder + Trimrudder;

    if (is_pid) {
      digitalWrite(LED, HIGH);
      ServoDegreeE = (int)pidCompute(&pidElevator, ServoDegreeE, lastOutputE);
      ServoDegreeR = (int)pidCompute(&pidRudder, ServoDegreeR, lastOutputR);
    } else {
      digitalWrite(LED, LOW);
    }
    lastOutputE = ServoDegreeE;
    lastOutputR = ServoDegreeR;

    krs.setPos(0, ServoDegreeE);
    krs.setPos(1, ServoDegreeR);

    Serial.printf("ele:%d(%d), rud:%d(%d)\n",ServoDegreeE,elevetor,ServoDegreeR,rudder);
    

    vTaskDelay((1000.0/frequency) / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);

  // 設定の読み込み
  loadSettings();

  krs.begin();
  krs.setPos(0,7500);
  pinMode(r_elevator,INPUT);
  pinMode(r_rudder,INPUT);
  pinMode(trimE,INPUT);
  pinMode(LED,OUTPUT);
  pinMode(trimR1,INPUT);
  pinMode(trimR2,INPUT);
  pinMode(resetPin, INPUT);

  pidInit(&pidElevator, 1.0, 0.1, 0.05);
  pidInit(&pidRudder, 1.0, 0.1, 0.05);

  // 各タスクの生成
  xTaskCreate(nvmTask, "nvmTask", 4096, NULL, 2, &nvmTaskHandle);
  xTaskCreate(resetMonitorTask, "resetMonitor", 2048, NULL, 5, NULL); // リセット監視タスクを追加
  xTaskCreate(mainloop, "mainloop", 10000, NULL, 10, NULL);
}

void loop(){}
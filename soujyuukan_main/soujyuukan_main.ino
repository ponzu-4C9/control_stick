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
  int ServoElevatorMax = 11500; //+10°
  int ServoElevatorMin = 3500; //-10°
  int ServoRudderMax = 11500; //+10°
  int ServoRudderMin = 3500; //-10°
  int TrimElevatorMax = 2000;
  int TrimElevatorMin = -2000;
  int TrimRudderMax = 2000;
  int TrimRudderMin = -2000;
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
    float integralMax;
    float lastError;
    unsigned long lastTime;
  };
  PidState pidElevator;
  PidState pidRudder;
  float currentPitch = 7500; // 姿勢角（ピッチ）：IMUから取得する値をここに入れる
  float currentYaw = 7500;   // 姿勢角（ヨー）：IMUから取得する値をここに入れる

  void pidInit(PidState* state, float kp, float ki, float kd, float integralMax) {
    state->kp = kp;
    state->ki = ki;
    state->kd = kd;
    state->integral = 0;
    state->integralMax = integralMax;
    state->lastError = 0;
    state->lastTime = millis();
  }

  void pidReset(PidState* state) {
    state->integral = 0;
    state->lastError = 0;
    state->lastTime = millis();
  }

  float pidCompute(PidState* state, float error) {
    unsigned long now = millis();
    float dt = (now - state->lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;

    state->integral += error * dt;
    state->integral = constrain(state->integral, -state->integralMax, state->integralMax);
    float derivative = (error - state->lastError) / dt;

    float output = state->kp * error + state->ki * state->integral + state->kd * derivative;

    state->lastError = error;
    state->lastTime = now;

    return output;
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
  int rawEle = 0;
  int rawRud = 0;
  int elergs[4] = {1120, 1590, 1930, 2460}; // エレベーター: 前限界, 前戻り, 後戻り, 後限界
  int rudrgs[4] = {1550,2120,2520,3100};             

  void Potentiometer(){
    rawEle = analogRead(r_elevator);
    rawRud = analogRead(r_rudder);
    int ELE = rawEle;
    int RUD = rawRud;

    // エレベーターのデッドゾーン + リマッピング
    if (ELE < elergs[0]) {
      elevetor = ServoElevatorMax;
    } else if (ELE < elergs[1]) {
      elevetor = map(ELE, elergs[0], elergs[1], ServoElevatorMax, 7500);
    } else if (ELE < elergs[2]) {
      elevetor = 7500;
    } else if (ELE < elergs[3]) {
      elevetor = map(ELE, elergs[2], elergs[3], 7500, ServoElevatorMin);
    } else {
      elevetor = ServoElevatorMin;
    }

    // ラダーのデッドゾーン + リマッピング
    if (RUD < rudrgs[0]) {
      rudder = ServoRudderMax;
    } else if (RUD < rudrgs[1]) {
      rudder = map(RUD, rudrgs[0], rudrgs[1], ServoRudderMax, 7500);
    } else if (RUD < rudrgs[2]) {
      rudder = 7500;
    } else if (RUD < rudrgs[3]) {
      rudder = map(RUD, rudrgs[2], rudrgs[3], 7500, ServoRudderMin);
    } else {
      rudder = ServoRudderMin;
    }
  }

  unsigned long lastPushed = millis();//ボタン4チャタリング防止用
  int is_buttun3 = 0;//ボタン3長押し検知用
  int lastTrimE = 0;

  void trimElevetor(){
    int TrimE = analogRead(trimE);
    
    if(0 <= TrimE && TrimE <=100){    //優先度1
      Trimelevetor = constrain(Trimelevetor + 3, TrimElevatorMin, TrimElevatorMax);
      settingsChanged = true;

    }else if(1000 <= TrimE && TrimE <=1200){  // 優先度2
      Trimelevetor = constrain(Trimelevetor - 3, TrimElevatorMin, TrimElevatorMax);
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
        Trimrudder = constrain(Trimrudder + 10, TrimRudderMin, TrimRudderMax);
        if (nvmTaskHandle != NULL) xTaskNotifyGive(nvmTaskHandle);
      }
      if (nowTrimR2 == HIGH && lastTrimR2 == LOW) {
        Trimrudder = constrain(Trimrudder - 10, TrimRudderMin, TrimRudderMax);
        if (nvmTaskHandle != NULL) xTaskNotifyGive(nvmTaskHandle);
      }
      lastTrimR1 = nowTrimR1;
      lastTrimR2 = nowTrimR2;
  }

  const long frequency = 30; 
  void mainloop(void *pvParameters) {
    while(1){
      const TickType_t xFrequency = (1000.0 / frequency) / portTICK_PERIOD_MS;
      TickType_t xLastWakeTime = xTaskGetTickCount();

      Potentiometer();
      trimElevetor();
      trimRudder();
      int ServoDegreeE = elevetor + Trimelevetor;
      int ServoDegreeR = rudder + Trimrudder;

      if (is_pid) {
        digitalWrite(LED, HIGH);
        float errorE = ServoDegreeE - currentPitch;
        float errorR = ServoDegreeR - currentYaw;
        ServoDegreeE = ServoDegreeE + (int)pidCompute(&pidElevator, errorE);
        ServoDegreeR = ServoDegreeR + (int)pidCompute(&pidRudder, errorR);
      } else {
        digitalWrite(LED, LOW);
      }

      ServoDegreeE = constrain(ServoDegreeE, ServoElevatorMin, ServoElevatorMax);
      ServoDegreeR = constrain(ServoDegreeR, ServoRudderMin, ServoRudderMax);

      krs.setPos(0, ServoDegreeE);
      krs.setPos(1, ServoDegreeR);

      Serial.printf("E:%d(%d) R:%d(%d) raw:%d,%d\n", ServoDegreeE, elevetor, ServoDegreeR, rudder, rawEle, rawRud);
      

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

    pidInit(&pidElevator, 1.0, 0.1, 0.05, 1000.0);
    pidInit(&pidRudder, 1.0, 0.1, 0.05, 1000.0);

    // 各タスクの生成
    xTaskCreate(nvmTask, "nvmTask", 4096, NULL, 2, &nvmTaskHandle);
    xTaskCreate(resetMonitorTask, "resetMonitor", 2048, NULL, 5, NULL); // リセット監視タスクを追加
    xTaskCreate(mainloop, "mainloop", 10000, NULL, 10, NULL);
  }

  void loop(){}
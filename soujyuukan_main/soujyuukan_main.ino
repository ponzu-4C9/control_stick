#include <IcsBaseClass.h>
#include <IcsHardSerialClass.h>
#include <freertos/FreeRTOS.h>
const byte EN_PIN = 8; //サーボ
const long BAUDRATE = 115200;
const int TIMEOUT = 25;
const int r_elevator = 2; //可変抵抗エレベーター
const int r_rudder = 3;   //可変抵抗ラダー
const int trimE = 4; //エレベータートリムスイッチ
const int LED = 5;
const int trimR1 = 9;  //トリムラダー
const int trimR2 = 10; //トリムラダー
int ServoDegreeElevatorMax = 11500;
int ServoDegreeElevatorMin = 3500;
int ServoDegreeRudderMax = 11500;
int ServoDegreeRudderMin = 3500;
IcsHardSerialClass krs(&Serial0,EN_PIN,BAUDRATE,TIMEOUT); //インスタンス＋ENピン(8番ピン)およびUARTの指定

void Ltika(void *pvParameters){
  for(int i = 0;i < 3;i++){
    digitalWrite(LED, HIGH);
    vTaskDelay(300);
    digitalWrite(LED, LOW);
    vTaskDelay(100);
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


int Trimelevetor = 0;
int neutralTrimeEle = 0;
int is_pid = 0;//今PID制御をONにするかどうか(0か1)
unsigned long lastPushed = millis();//ボタン4チャタリング防止用
int is_buttun3 = 0;//ボタン3長押し検知用
int lastTrimE = 0;
void trimElevetor(){
  int TrimE = analogRead(trimE);
  
  if(0 <= TrimE && TrimE <=100){    //優先度1 ,0
    Trimelevetor += 3;

  }else if(1000 <= TrimE && TrimE <=1200){  // 優先度2, 1100
    Trimelevetor -= 3;

  }else if(2000 <= TrimE && TrimE <= 2300){  //優先度3, 2180
    is_buttun3 += 4;
    if(is_buttun3 > 20){
      neutralTrimeEle = Trimelevetor;
      is_buttun3 = 0;
      xTaskCreate(
        Ltika,
        "Ltika",
        1024,
        NULL,
        9,
        NULL
      );
    }
    
  }else if(!(2000 <= TrimE && TrimE <= 2300) && (2000 <= lastTrimE && lastTrimE <= 2300)){
    Trimelevetor = neutralTrimeEle;
  }else if(3300 <= TrimE && TrimE <= 3500 && millis() - lastPushed > 250){   //優先度4 ,3100,　何ms間隔で検出
    if(is_pid){
      is_pid = 0;
    }else{
      is_pid = 1;
    }
    lastPushed = millis();
  }else{
    is_buttun3 = 0;
  }
  lastTrimE = TrimE;
}

int senterEle = 0;
int senterRud = 0;

int lastTrimR1 = LOW;
int lastTrimR2 = LOW;
int Trimrudder = 0;
void trimRudder(){
    int TrimR1 = digitalRead(trimR1);
    int TrimR2 = digitalRead(trimR2);
    int nowTrimR1 = digitalRead(trimR1);
    int nowTrimR2 = digitalRead(trimR2);

    if (nowTrimR1 == HIGH && lastTrimR1 == LOW) {
     Trimrudder += 10;
    }
    if (nowTrimR2 == HIGH && lastTrimR2 == LOW) {
      Trimrudder -= 10;
    }
    lastTrimR1 = nowTrimR1;
    lastTrimR2 = nowTrimR2;
}

const long frequency = 30; // 一秒に何回実行するか
void mainloop(void *pvParameters) {
  while(1){
    Potentiometer();  //可変抵抗
    trimElevetor();   //トリムスイッチ（エレベーター）
    trimRudder();     //トリムスイッチ（ラダー）
    int ServoDegreeE = elevetor + Trimelevetor;
    int ServoDegreeR = rudder + Trimrudder;
    ServoDegreeE = constrain(ServoDegreeE, ServoDegreeElevatorMin, ServoDegreeElevatorMax);  //サーボの角度、最小値、最大値
    ServoDegreeR = constrain(ServoDegreeR, ServoDegreeRudderMin, ServoDegreeRudderMax);
    //krs.setSpd(1,sp);
    if(is_pid){
      digitalWrite(LED,HIGH);
    }else{
      digitalWrite(LED,LOW);
    }
    Serial.printf(" elevator: %d(%d), rudder: %d(%d),is_buttun3: %d\n", ServoDegreeE,elevetor,ServoDegreeR,rudder,is_buttun3);
    krs.setPos(0,ServoDegreeE);      //位置指令　
    krs.setPos(1,ServoDegreeR);

    // 1000ms / 80回
    vTaskDelay((1000.0/frequency) / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}
//サーボでトリムスイッチをおすとサーボの角度の範囲が変わる(最小、最大を超えることはない)

void setup() {
  Serial.begin(115200);  //これがないと次書き込む時にESP32-C3が書き込めなくなる→boot押しながらリセット一回押す→bootを押しながら書き込み
  krs.begin(); //サーボモータの通信初期設定,Serial.beginと同時に使えない
  krs.setPos(0,7500); //位置指令　ID:0サーボを7500へ 中央  (Servo_ID,degree)
  pinMode(r_elevator,INPUT);
  pinMode(r_rudder,INPUT);
  pinMode(trimE,INPUT);
  pinMode(LED,OUTPUT);
  pinMode(trimR1,INPUT);
  pinMode(trimR2,INPUT);

  xTaskCreate(
    mainloop,
    "mainloop",
    10000,
    NULL,
    10,
    NULL
  );
}
void loop(){}

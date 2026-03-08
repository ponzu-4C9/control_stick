//操縦環の可変抵抗の読み取り
#include <vector>

const int r_elevator = 2; //可変抵抗エレベーター
const int r_rudder = 3;   //可変抵抗ラダー
const int outPinPPM = 8;

int ServoDegreeElevatorMax = 8800;
int ServoDegreeElevatorMin = 6200;
int ServoDegreeRudderMax = 8800;
int ServoDegreeRudderMin = 6200;

int Aeleron_uS = 1225;    
int Elevator_uS = 1225;
int Throttle_uS = 750;    
int Rudder_uS = 1225;
int TI_uS = 750;
int TIsw_uS = 750;

void IRAM_ATTR ppmoutput() {
  int Fixed_uS = 300;
  // Channel 1 - Aeleron
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Aeleron_uS);  // Hold for Aeleron_uS microseconds     
 
 // Channel 2 - Elevator
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Elevator_uS); // Hold for Elevator_uS microseconds     
 
 // Channel 3 - Throttle
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Throttle_uS); // Hold for Throttle_uS microseconds     
 
 // Channel 4 - Rudder
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Rudder_uS);   // Hold for Rudder_uS microseconds
 
 // Channel 5 - TI Switch
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(TIsw_uS);     // Hold for TIsw_uS microseconds       
 
 // Channel 6 - TI pot
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(TI_uS);       // Hold for TI_uS microseconds 
 
 // Synchro pulse
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);  // Start Synchro pulse
}

hw_timer_t * timer = NULL;

void setup() {
  Serial.begin(115200);  //これがないと次書き込む時にESP32-C3が書き込めなくなる→boot押しながらリセット一回押す→bootを押しながら書き込み
  pinMode(r_elevator,INPUT);
  pinMode(r_rudder,INPUT);
  pinMode(outPinPPM,OUTPUT);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &ppmoutput);
  timerAlarm(timer, 22000, true, 0);
}

void loop() {
  int elevetor = analogRead(r_elevator);
  int rudder = analogRead(r_rudder);
  Elevator_uS = map(elevetor,1130,2500,1700,750);
  Rudder_uS = map(rudder,1520,3200,750,1700);


  Serial.printf("e:%d r:%d\n",elevetor,rudder);
}

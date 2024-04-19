/* 라이브러리 */



/* 핀 정보 */

const int DUSTSENSOR_PIN = A0;
const int MQ7_PIN = A1;
const int MQ135_PIN = A2;
const int MQ2_PIN = A3;
const int VRESISOTR_PIN = A4;
const int MIKE_PIN = A5;

const int BATH_LED_PIN = 2;
const int BATH_PIR_PIN = 3;
const int LED_PIN = 4;
const int PIR_PIN = 5;
const int BTN_BELL_PIN = 6;
const int BUZZER_PIN = 7 ;

/* 상수 */



/* 변수 */
int flammableGasLevel =0;
int sensorThres = 200;


/* 함수 종류 */
void buzz_operation();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  buzz_operation();
  Serial.println(flammableGasLevel);
}

/* 함수 세부사항 */ 

// Checks if it has reached the threshold value
void buzz_operation(){
  flammableGasLevel = analogRead(MQ2_PIN);

  if (flammableGasLevel > sensorThres){
    tone(BUZZER_PIN, 1000, 200);
  }
  else{
    noTone(BUZZER_PIN);
  }
}
////////////// 라이브러리 입력란
#include <SPI.h>
#include <MFRC522.h>
#include <List.hpp>
#include <DFRobot_DHT11.h>
DFRobot_DHT11 DHT;
//////////////

/////////// pin 입력란
const int BUTTON = 5;
const int RST_PIN = 9;
const int SS_PIN = 10;
const int PIR = 6;
const int R_LED = 2;
const int B_LED = 3;
const int G_LED = 4;
const int buzzer = 6; // conflict?
const int fan = 7;
const int DHT11_PIN = A1;
const int FAN = A2; // replace air cleaner with fan
const int dustSensor = A3;
const int MQ135 = A4;
const int MQ2 = A5;

/////////// 변수 입력란
int cardDetected = false;
int pirstate = false;
int person_count = 0;
bool flag = true;
const int COLOR[][3] = {
    {200, 0, 0},    // Red
    {200, 150, 0},  // Orange
    {200, 200, 0},  // Yellow
    {0, 200, 0},    // Green
    {0, 0, 20}     // Blue
};
int dustLevel = 0;
int gasLevel =0 ;
String quality = "";
int sensorThres = 170;
int flammableGasLevel =0;
bool fan_on = false; 
unsigned long previousMillis = 0; 
const long interval = 500; 
int auto_temperature;
int humidity; 

typedef enum {
  REGISTRATION = 0,
  VERIFICATION
} RFID_STATUS;
MFRC522 rc522(SS_PIN, RST_PIN);
RFID_STATUS rfid_status;
// 등록된 카드를 저장할 tag_list
List<MFRC522::Uid> tag_list;
///////////


/////////// 함수 입력하쇼
void buzz_operation(){
  flammableGasLevel = analogRead(MQ2);

  Serial.print("Pin A0: ");
  Serial.println(flammableGasLevel);
  
  // Checks if it has reached the threshold value
  if (flammableGasLevel > sensorThres){
    
    tone(buzzer, 1000, 200);
  }
  else{
    noTone(buzzer);
  }
}

void air_sensor(){
   dustLevel = analogRead(dustSensor);
   gasLevel = analogRead(MQ135);

  if(dustLevel<150){
    quality = "Very GOOD!";
     analogWrite(R_LED, COLOR[4][0]);
     analogWrite(G_LED, COLOR[4][1]);
     analogWrite(B_LED, COLOR[4][2]);
     analogWrite(FAN, 0);
  }
   else if(dustLevel<300){
     quality = "GOOD!";
     analogWrite(R_LED, COLOR[3][0]);
     analogWrite(G_LED, COLOR[3][1]);
     analogWrite(B_LED, COLOR[3][2]);
     analogWrite(FAN, 0);
   }
      
   else if (dustLevel<450){
     quality = "Poor!";
     analogWrite(R_LED, COLOR[2][0]);
     analogWrite(G_LED, COLOR[2][1]);
     analogWrite(B_LED, COLOR[2][2]);
     analogWrite(FAN, 150);
   }
   else if (dustLevel<600){
     quality  = "Very bad!";
     analogWrite(R_LED, COLOR[1][0]);
     analogWrite(G_LED, COLOR[1][1]);
     analogWrite(B_LED, COLOR[1][2]);
     analogWrite(FAN, 150);
   }
   else{
     quality = "Toxic";
     analogWrite(R_LED, COLOR[0][0]);
     analogWrite(G_LED, COLOR[0][1]);
     analogWrite(B_LED, COLOR[0][2]);
     analogWrite(FAN, 150);
   }
  Serial.print("dust level : ");
  Serial.println(dustLevel);
  // Serial.println(quality);
  Serial.print("gas level : ");
  Serial.println(gasLevel);
}

int buttonPress() {
  int press;
  int buttonState;
  static int prevButtonState = HIGH;
  buttonState = digitalRead(BUTTON);
  press = (buttonState == LOW && prevButtonState == HIGH);
  prevButtonState = buttonState;
  return press;
}

void printUID(MFRC522::Uid uid) {
  for (byte i = 0; i < 4; i++) {
    Serial.print(uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(uid.uidByte[i], HEX);
  }
  Serial.println();
}

bool checkUID(MFRC522::Uid uid) {
  for (int i = 0; i < tag_list.getSize(); i++) {
    if (memcmp(tag_list.get(i).uidByte, rc522.uid.uidByte, 4) == 0) {
      return true;
    }
  }
  return false;
}

void read_sensor_data()
{
  DHT.read(DHT11_PIN);
  auto_temperature = DHT.temperature;
  humidity = DHT.humidity;
  
  Serial.print("temperature : ");
  Serial.print(auto_temperature);
  Serial.print(" , ");
  Serial.print("humidity : ");
  Serial.println(humidity);
  Serial.println();
}

void control_fan()
{
  if (auto_temperature == 27)
  {
    unsigned long currentMillis = millis(); 
    if (currentMillis - previousMillis >= 3000) 
    { 
      analogWrite(fan, 0); 
      fan_on = false; 
    }
  }
  else 
  {
    analogWrite(fan, 150);
    fan_on = true; 
  }
}

void control_fan_gui()
{
  if (Serial.available() > 0) 
  {
    char command = Serial.read();
    if (command == '1') 
    {
      analogWrite(fan, 150);
      fan_on = true; 
    } 
    else if (command == '0') 
    {
      analogWrite(fan, 0); 
      fan_on = false; 
    }
  }
}
//////////

////////// setup문
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  rc522.PCD_Init();
  rfid_status = RFID_STATUS::REGISTRATION;
  Serial.println("[STATUS] Registration");
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(PIR, INPUT);
  pinMode(dustSensor, INPUT);
  pinMode(MQ2, INPUT);
  pinMode(MQ135, INPUT);
  
  pinMode(buzzer, OUTPUT);
  pinMode(fan, OUTPUT);

}
/////////////


///////////// loop문
void loop() {
  // put your main code here, to run repeatedly:
  int value = digitalRead(PIR);
  if (value) {
    pirstate = true;
    flag = true;
  }
  if (buttonPress() == true) {
    if (rfid_status == RFID_STATUS::REGISTRATION) {
      rfid_status = RFID_STATUS::VERIFICATION;
      Serial.println("[STATUS] Verification");
    } else {
      rfid_status = RFID_STATUS::REGISTRATION;
      Serial.println("[STATUS] Registration");
    }
  }
  if (!rc522.PICC_IsNewCardPresent()) {
    return;
  }
  if (!rc522.PICC_ReadCardSerial()) {
    return;
  }
  if (rfid_status == RFID_STATUS::REGISTRATION) {
    if (checkUID(rc522.uid) == false) {
      tag_list.addLast(rc522.uid);
      Serial.print("Registration Tag : ");
      tag_list.addLast(rc522.uid);
      printUID(rc522.uid);
    }
  }
  if (checkUID(rc522.uid)) {
    cardDetected = true;
  } else {
    cardDetected = false;
  }
  if (rfid_status == RFID_STATUS::VERIFICATION && cardDetected == true) {  // rfid_status == VERIFICATION
    if (pirstate) {
      person_count--;
      cardDetected = false;
      pirstate = false;
      flag = false;
      // Serial.print("감소: ");
      // Serial.println(person_count);
      if (person_count < 0) {
        person_count = 0;
      }
      delay(200);
    }
    // Serial.println(flag);
    while (rfid_status == RFID_STATUS::VERIFICATION && cardDetected == true && pirstate == false && flag == true) {
      // Serial.println("while문 입장");
      if (value) {
        person_count++;
        cardDetected = false;
        pirstate = false;
        flag = false;
        // Serial.print("증가: ");
        // Serial.println(person_count);
        break;
      }
    }
  }
  buzz_operation();
  air_sensor();
  delay(100);
  Serial.println(person_count);
  flag = true;

  unsigned long currentMillis = millis();  // 현재 시간을 가져옵니다.
  
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;  // 이전 시간을 업데이트합니다.
    read_sensor_data();
    control_fan();
  }
  control_fan_gui();
}

//////////////////

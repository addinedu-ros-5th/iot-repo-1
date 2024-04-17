////////////// 라이브러리 입력란
#include <SPI.h>
#include <MFRC522.h>
#include <List.hpp>
#include <Servo.h>
#include <DFRobot_DHT11.h>
//////////////

/////////// 변수 입력란
const int BUTTON = 5;
const int RST_PIN = 9;
const int SS_PIN = 10;
const int PIR = 6;
const int LED = 4;
int cardDetected = false;
int pirstate = false;
int person_count = 0;
bool flag = true;
unsigned long previousMillis = 0; 
const long interval = 500; 
int auto_temperature;
int humidity; 
DFRobot_DHT11 DHT;
const int DHT11_PIN = A1;
const int fan = 7;
const int btn = A2;
bool fan_on = false;

int cutain_flag = 1;
Servo servo;
unsigned long lightstart = 0;
unsigned long lightclose = 0;
const unsigned long lightdelay = 3000;

typedef enum {
  REGISTRATION = 0,
  VERIFICATION
} RFID_STATUS;
MFRC522 rc522(SS_PIN, RST_PIN);
RFID_STATUS rfid_status;
// 등록된 카드를 저장할 tag_list
List<MFRC522::Uid> tag_list;
///////////


/////////// 입력하쇼
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
  pinMode(LED, OUTPUT);

  pinMode(btn, INPUT);
  pinMode(fan, OUTPUT);
  servo.attach(8);
  servo.write(10);
}
/////////////


///////////// loop문
void loop() {










  unsigned long currentMillis = millis();  // 현재 시간을 가져옵니다.

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // 이전 시간을 업데이트합니다.
    read_sensor_data();
    control_fan();
  }
  control_fan_gui();










  int light = analogRead(A0);
  int scaledLight = map(light, 0, 1023, 0, 100);
  Serial.println(scaledLight);
  Serial.print(",");
  if (scaledLight < 50) {
    if (cutain_flag == 0) {
      if (lightstart == 0) {
        lightstart = millis();
      }
      if (millis() - lightstart >= lightdelay) {
        servo.write(100);
        cutain_flag = 1;
        lightclose = 0;
      }
    }
  } else {
    if (cutain_flag == 1) {
      if (lightclose == 0) {
        lightclose = millis();
      }
      if (millis() - lightclose >= lightdelay) {
        servo.write(10);
        cutain_flag = 0;
        lightstart = 0;
      }
    }
    if (Serial.available() > 0) {
      int angle = Serial.parseInt();  // 시리얼 포트로부터 각도를 읽음
      servo.write(angle);             // 서보모터를 해당 각도로 회전
    }
  }
  ///////////////////////////////////////////////////////////////////////////////////
  Serial.print("1번");
  DHT.read(DHT11_PIN);  // 만악의 근원
  Serial.print("2번");
  int temperature = DHT.temperature;
  int humidity = DHT.humidity;
  Serial.print(temperature);
  Serial.print(",");
  Serial.println(humidity);

  // if (DHT.temperature <= 26 || DHT.temperature >= 28)
  if (DHT.humidity <= 40 || DHT.humidity >= 50) {
    digitalWrite(fan, HIGH);
    fan_on = true;
  } else if (fan_on) {
    digitalWrite(fan, LOW);
    fan_on = false;
  }
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '1') {
      digitalWrite(fan, HIGH);
      fan_on = true;
    } else if (command == '0') {
      digitalWrite(fan, LOW);
      fan_on = false;
    }
  }
  control_fan_based_on_humidity();














  // put your main code here, to run repeatedly:
  int value = digitalRead(PIR);

  // Serial.println(value);

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
  if (value) {
    pirstate = true;
    flag = true;
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
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
      delay(100);
    }
    // Serial.println(flag);
    while (rfid_status == RFID_STATUS::VERIFICATION && cardDetected == true && pirstate == false && flag == true) {
      Serial.println("while문 입장");
      int value = digitalRead(PIR);
      if (value) {
        Serial.print("pir 값: ");
        Serial.println(pirstate);
        person_count++;
        cardDetected = false;
        pirstate = false;
        flag = false;
        // Serial.print("증가: ");
        // Serial.println(person_count);
        delay(100);
        break;
      }
    }
    Serial.println("탈출");
  }
  delay(100);
  Serial.print("인원 수 :");
  Serial.println(person_count);
  flag = true;
}

//////////////////

void control_fan_based_on_humidity() {
  if (DHT.humidity <= 40 || DHT.humidity >= 50) {
    digitalWrite(fan, HIGH);
    fan_on = true;
  } else if (fan_on) {
    digitalWrite(fan, LOW);
    fan_on = false;
  }
}

void read_sensor_data() {
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

void control_fan() {
  if (auto_temperature == 27) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 3000) {
      analogWrite(fan, 0);
      fan_on = false;
    }
  } else {
    analogWrite(fan, 150);
    fan_on = true;
  }
}

void control_fan_gui() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '1') {
      analogWrite(fan, 150);
      fan_on = true;
    } else if (command == '0') {
      analogWrite(fan, 0);
      fan_on = false;
    }
  }
}

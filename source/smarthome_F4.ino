/* 라이브러리 */
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <DFRobot_DHT11.h>
#include <List.hpp>

/* 핀 정보 */
const int R_LED = 2;
const int G_LED = 3;
const int buzzer = 4;
const int dustFan = A2;
const int dustSensor = A3;
const int MQ135 = A4;
const int MQ2 = A5;
const int BUTTON = 5;
const int PIR = 6;
const int RST_PIN = 9;
const int SS_PIN = 10;
const int airFan = 7;
const int DHT11_PIN = A1;
const int servo_pin = 8;

/* 변수 */
int cardDetected = false;
int pirstate = false;
int person_count = 0;
bool flag = true;
unsigned long previousMillis = 0;
const long interval = 500;
int auto_temperature;
int humidity;
DFRobot_DHT11 DHT;

bool fan_on = false;
int cutain_flag = 1;
Servo servo;
unsigned long lightstart = 0;
unsigned long lightclose = 0;
const unsigned long lightdelay = 3000;
const int COLOR[][3] = {
  { 200, 0, 0 },    // Red
  { 200, 150, 0 },  // Orange
  { 200, 200, 0 },  // Yellow
  { 0, 200, 0 },    // Green
  { 0, 0, 200 }     // Blue
};
int dustLevel = 0;
int gasLevel = 0;
int flammableGasLevel = 0;
String quality = "";
int sensorThres = 200;

typedef enum {
  REGISTRATION = 0,
  VERIFICATION
} RFID_STATUS;

MFRC522 rc522(SS_PIN, RST_PIN);
RFID_STATUS rfid_status;
List<MFRC522::Uid> tag_list;

/* 함수 선언 */
void detectMotion();
void countPeople();
void controlVentilation();
void controlCurtain();
void readTemperatureHumidity();
void manualControlAirconditioner();

/* 설정 */
void setup() {
  Serial.begin(9600);
  SPI.begin();
  rc522.PCD_Init();
  rfid_status = RFID_STATUS::REGISTRATION;
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(PIR, INPUT);
  pinMode(airFan, OUTPUT);
  servo.attach(servo_pin);
  servo.write(10);
  pinMode(dustSensor, INPUT);
  pinMode(MQ2, INPUT);
  pinMode(MQ135, INPUT);
  pinMode(buzzer, OUTPUT);
}

/* 루프 */
void loop() {

  detectMotion();
  Serial.print("PIR : ");
  Serial.print(pirstate);

  countPeople();
  controlVentilation();
  buzz_operation();
  controlCurtain();
  readTemperatureHumidity();
  manualControlAirconditioner();
  Serial.println();
}

/* 함수 구현 */

// 모션 감지 센서 함수
void detectMotion() {
  int value = digitalRead(PIR);
  if (value) {
    pirstate = true;
    flag = true;
    lightstart = millis();
  } else {
    if (millis() - lightstart >= lightdelay) {
      pirstate = false;
    }
  }
}

// 유동 인구 체크 함수
void countPeople() {
  int value = digitalRead(PIR);
  // if (value) {
  //   pirstate = true;
  //   flag = true;
  // }
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
  } else {
  }
  if (pirstate) {
    if (rfid_status == RFID_STATUS::VERIFICATION && cardDetected == true)  // rfid_status == VERIFICATION
    {

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
      int value = digitalRead(PIR);
      if (value) {
        Serial.print("pir 값: ");
        Serial.print(pirstate);
        person_count++;
        cardDetected = false;
        pirstate = false;
        flag = false;
        // Serial.print("증가: ");
        // Serial.println(person_count);
        delay(100);
      }
    }
    Serial.println("탈출");
  }
  delay(100);
  Serial.print("인원 수 :");
  Serial.print(person_count);
  flag = true;
}

// 먼지 센서에 따른 LED 제어 함수
void controlVentilation() {
  dustLevel = analogRead(dustSensor);
  gasLevel = analogRead(MQ135);
  if (dustLevel < 150) {
    quality = "Very GOOD!";
    analogWrite(R_LED, COLOR[4][0]);
    analogWrite(G_LED, COLOR[4][1]);
    analogWrite(dustFan, 0);
  } else if (dustLevel < 300) {
    quality = "GOOD!";
    analogWrite(R_LED, COLOR[3][0]);
    analogWrite(G_LED, COLOR[3][1]);
    analogWrite(dustFan, 0);
  } else if (dustLevel < 450) {
    quality = "Poor!";
    analogWrite(R_LED, COLOR[2][0]);
    analogWrite(G_LED, COLOR[2][1]);
    analogWrite(dustFan, 150);
  } else if (dustLevel < 600) {
    quality = "Very bad!";
    analogWrite(R_LED, COLOR[1][0]);
    analogWrite(G_LED, COLOR[1][1]);
    analogWrite(dustFan, 150);
  } else {
    quality = "Toxic";
    analogWrite(R_LED, COLOR[0][0]);
    analogWrite(G_LED, COLOR[0][1]);
    analogWrite(dustFan, 150);
  }
  Serial.print(", dust level : ");
  Serial.print(dustLevel);
  Serial.print(", gas level : ");
  Serial.print(gasLevel);
}

// 부저 알림 함수
void buzz_operation() {
  flammableGasLevel = analogRead(MQ2);
  Serial.print(", flammableGasLevel : ");
  Serial.print(flammableGasLevel);
  // Checks if it has reached the threshold value
  if (flammableGasLevel > sensorThres) {
    tone(buzzer, 1000, 200);
  } else {
    noTone(buzzer);
  }
}

// 조도센서 값에 따른 커튼 제어 함수
void controlCurtain() {
  int light = analogRead(A0);
  int scaledLight = map(light, 0, 1023, 0, 100);
  Serial.print(scaledLight);
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
      int angle = Serial.parseInt();
      servo.write(angle);
    }
  }
}

// 온습도 값 출력 함수
void readTemperatureHumidity() {
  DHT.read(DHT11_PIN);
  auto_temperature = DHT.temperature;
  humidity = DHT.humidity;
  Serial.print(" temperature : ");
  Serial.print(auto_temperature);
  Serial.print(" , ");
  Serial.print("humidity : ");
  Serial.print(humidity);
}

// 입력값에 따른 에어컨 제어 함수
void manualControlAirconditioner() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '1') {
      analogWrite(airFan, 150);
      fan_on = true;
    } else if (command == '0') {
      analogWrite(airFan, 0);
      fan_on = false;
    }
  }
}

// RFID 모드 변경 버튼 함수
int buttonPress() {
  int press;
  int buttonState;
  static int prevButtonState = HIGH;
  buttonState = digitalRead(BUTTON);
  press = (buttonState == LOW && prevButtonState == HIGH);
  prevButtonState = buttonState;
  return press;
}

// RFID 함수
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

////////////// 라이브러리 입력란
#include <SPI.h>
#include <MFRC522.h>
#include <List.hpp>
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
}
/////////////


///////////// loop문
void loop() {
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

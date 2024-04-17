#include <SPI.h>
#include <MFRC522.h>
#include <List.hpp>
const int BUTTON1 = 2;
const int BUTTON2 = 3;
const int RST_PIN = 9;
const int SS_PIN = 10;
int cardDetected = false;
int buttonstatus = false;
int person_count = 0;
int count = 0;
int flag = 1;
// Qt의 상태값
typedef enum {
  REGISTRATION = 0,
  VERIFICATION
} RFID_STATUS;
MFRC522 rc522(SS_PIN, RST_PIN);
RFID_STATUS rfid_status;
// 등록된 카드를 저장할 tag_list
List<MFRC522::Uid> tag_list;
int buttonPress1() {
  int press;
  int buttonState;
  static int prevButtonState = HIGH;
  buttonState = digitalRead(BUTTON1);
  press = (buttonState == LOW && prevButtonState == HIGH);
  prevButtonState = buttonState;
  return press;
}
int buttonPress2() {
  int press;
  int buttonState;
  static int prevButtonState = HIGH;
  buttonState = digitalRead(BUTTON2);
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
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  rc522.PCD_Init();
  rfid_status = RFID_STATUS::REGISTRATION;
  Serial.println("[STATUS] Registration");
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
}
void loop() {
  //
  if (buttonPress2()) {
    buttonstatus = true;
    flag = true;
  }
  // Serial.println(cardDetected);
  // Serial.println(buttonstatus);
  if (buttonPress1() == true) {
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
    if (buttonstatus) {
      person_count--;
      cardDetected = false;
      buttonstatus = false;
      flag = false;
      // Serial.print("감소: ");
      // Serial.println(person_count);
      if (person_count < 0) {
        person_count = 0;
      }
      delay(200);
    }
    // Serial.println(flag);
    while (rfid_status == RFID_STATUS::VERIFICATION && cardDetected == true && buttonstatus == false && flag == true) {
      Serial.println("while문 입장");
      if (buttonPress2()) {
        person_count++;
        cardDetected = false;
        buttonstatus = false;
        flag = false;
        // Serial.print("증가: ");
        // Serial.println(person_count);
        break;
      }
    }
  }
  delay(100);
  Serial.println(person_count);
  flag = true;
}
// 버튼과 rfid를 활용한 집 내부 사람 수 체크 완료1

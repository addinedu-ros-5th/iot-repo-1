#include <SPI.h>
#include <MFRC522.h>
#include <List.hpp>
#include <Servo.h>

const int RST_PIN = 9;
const int SS_PIN = 10;

Servo servo;

typedef enum
{
  REGISTRATION = 0,
  VERIFICATION
} RFID_STATUS;
MFRC522 rc522(SS_PIN, RST_PIN);
RFID_STATUS rfid_status;
List<MFRC522::Uid> tag_list;
bool cardPresent = false;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  rc522.PCD_Init();
  rfid_status = REGISTRATION;
  Serial.println("[STATUS] Registration");
  servo.attach(8);
  servo.write(0); // 초기 위치 설정
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'v') {
      rfid_status = VERIFICATION;
      Serial.println("[STATUS] Verification");
    } else if (input == 'r') {
      rfid_status = REGISTRATION;
      Serial.println("[STATUS] Registration");
    }
  }

  if (!rc522.PICC_IsNewCardPresent()) {
    if (cardPresent) {
      servo.write(0); // 카드가 떼어졌을 때 서보 모터를 0도로 돌림
      cardPresent = false;
    }
    return;
  }

  if (!rc522.PICC_ReadCardSerial()) {
    return;
  }

  if (rfid_status == REGISTRATION) {
    // 등록 모드: 새로운 카드 UID를 리스트에 추가
    MFRC522::Uid uid = rc522.uid;
    if (!checkUID(uid)) {
      tag_list.addLast(uid);
      Serial.print("Registration Tag: ");
      printUID(uid);
    }
  } else if (rfid_status == VERIFICATION) {
    // 인증 모드: 스캔된 카드 UID가 리스트에 있는지 확인 후 서보 모터 제어
    MFRC522::Uid uid = rc522.uid;
    if (checkUID(uid)) {
      // 유효한 카드인 경우
      servo.write(90); // 서보 모터를 90도 회전
      
      cardPresent = true;
      delay(3000);
    } else {
      // 유효하지 않은 카드인 경우
      servo.write(0); // 서보 모터를 0도로 회전
      cardPresent = false;
    }

  }

  delay(100);
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
    if (memcmp(tag_list.get(i).uidByte, uid.uidByte, 4) == 0) {
      return true;
    }
  }
  return false;
}

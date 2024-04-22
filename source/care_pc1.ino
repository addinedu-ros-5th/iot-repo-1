#include <SPI.h>
#include <MFRC522.h>
#include <List.hpp>

const int RST_PIN = 9;
const int SS_PIN = 10;
typedef enum {
  REGISTRATION = 0,
} RFID_STATUS;
MFRC522 rfid(SS_PIN, RST_PIN);  // MFRC522 instance 생성
RFID_STATUS rfid_status;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  SPI.begin();
  rfid.PCD_Init();
  rfid_status = RFID_STATUS::REGISTRATION;
  // Serial.println("[STATUS] Registration");
}
void loop() {
  //

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    // Print UID of the card
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i] < 0x10 ? "0" : "");  // leading zero 출력
      Serial.print(rfid.uid.uidByte[i], HEX);
      if (i + 1 < rfid.uid.size) {
      }
    }

    delay(1000);
    Serial.println();
  }
}

#include <SPI.h>
#include <MFRC522.h>
#include <List.hpp>

const int RST_PIN = 9;
const int SS_PIN = 10;
typedef enum
{
  REGISTRATION = 0,
  VERIFICATION
} RFID_STATUS;
MFRC522 rc522(SS_PIN, RST_PIN);
RFID_STATUS rfid_status;
List<MFRC522::Uid> tag_list;

void printUID(MFRC522::Uid uid)
{
  for (byte i = 0; i < 4; i++)
  {
    Serial.print(uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(uid.uidByte[i], HEX);
  }
  Serial.println();
}
bool checkUID(MFRC522::Uid uid)
{
  for (int i = 0; i < tag_list.getSize(); i++)
  {
    if (memcmp(tag_list.get(i).uidByte, rc522.uid.uidByte, 4) == 0)
    {
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
  // Serial.println("[STATUS] Registration");
}
void loop() {
  //
  
  if (!rc522.PICC_IsNewCardPresent()) {
    return;
  }
  if (!rc522.PICC_ReadCardSerial()) {
    return;
  }
  if (rfid_status == RFID_STATUS::REGISTRATION)
  {
    if (checkUID(rc522.uid) == false)
    {
      tag_list.addLast(rc522.uid);
      // tag_list.addLast(rc522.uid);
      printUID(rc522.uid);
    }
  }

  delay(100);
}
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

#define SS_PIN 10
#define RST_PIN 9
#define SERVO_PIN 8
typedef enum {
  REGISTRATION = 0,
  IDLE
} RFID_STATUS;
MFRC522 rfid(SS_PIN, RST_PIN);  // MFRC522 instance 생성
RFID_STATUS rfid_status;
Servo servo;                    // Servo instance 생성


void setup() {
  Serial.begin(9600);       // 시리얼 통신 초기화
  SPI.begin();              // SPI 버스 초기화
  rfid.PCD_Init();          // MFRC522 초기화
  servo.attach(SERVO_PIN);  // Servo 모터 초기화
  Serial.println("RFID reader initialized");
  servo.write(0);
}

void loop() {
  // Look for new cards
  if (rfid.PICC_IsNewCardPresent()) {
    if (rfid.PICC_ReadCardSerial() && rfid_status == RFID_STATUS::REGISTRATION) {
      // Print UID of the card
      for (byte i = 0; i < rfid.uid.size; i++) {
        Serial.print(rfid.uid.uidByte[i] < 0x10 ? "0" : "");  // leading zero 출력
        Serial.print(rfid.uid.uidByte[i], HEX);
        if (i + 1 < rfid.uid.size) {
          Serial.print(" ");
        }
      }
      Serial.println();
      delay(500);                       // 0.5초 동안 대기
      rfid.PICC_HaltA();                // 카드 리더 초기화
      rfid_status = RFID_STATUS::IDLE;  // 상태를 변경하여 다음에 카드가 인식될 때까지 대기
    }
  } else {
    rfid_status = RFID_STATUS::REGISTRATION;  // 카드가 없으면 다시 등록 상태로 변경
  }
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();  // Read the incoming byte
    if (receivedChar == 'y') {
      servo.write(90);  // Servo 모터를 90도 회전
      Serial.println("Servo rotated to 90 degrees");
      delay(1000);
      servo.write(0);
    } else if (receivedChar == 'n') {
      servo.write(180);  // Servo 모터를 180도 회전
      Serial.println("Servo rotated to 180 degrees");
      delay(1000);
      servo.write(0);
    }
  }
}

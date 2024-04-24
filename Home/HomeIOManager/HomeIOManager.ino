/* Library */
#include <MQUnifiedsensor.h>
#include <Servo.h>

/* Board Information */
#define board "Arduino UNO"
#define Voltage_Resolution 5
#define ADC_Bit_Resolution 10  // For Arduino UNO/MEGA/NANO

/* MQ-7 Sensor Information*/
#define MQ7_type "MQ-7"        // MQ7
#define RatioMQ7CleanAir 27.5  // RS / R0 = 27.5 ppm

/* MQ-135 Sensor Information*/
#define MQ135_type "MQ-135"     // MQ135
#define RatioMQ135CleanAir 3.6  // RS / R0 = 3.6 ppm

/* MQ-2 Sensor Information*/

#define MQ2_type "MQ-2"        // MQ2
#define RatioMQ2CleanAir 9.83  // RS / R0 = 9.83 ppm


/* Pin Information */
/* analog input */
const int DUSTSENSOR_PIN = A0;
const int MQ7_PIN = A1;
const int MQ135_PIN = A2;
const int MQ2_PIN = A3;
const int VRESISOTR_PIN = A4;
const int MIC_PIN = A5;
/* digital input */
const int BTN_LED_BATH_PIN = 2;
const int BTN_LED_PIN = 3;
const int BTN_BELL_PIN = 4;
const int BTN_GAS_VALVE_PIN = 5;
const int PIR_BATH_PIN = 6;
const int PIR_PIN = 7;
/* digital output */
const int LED_BATH_PIN = 8;
const int LED_PIN = 9;
const int BUZZER_PIN = 10;
const int SERVO_PIN = 11;
const int GAS_BUZZER = 12;
const int DUST_FAN = 13;

/* Constants */
unsigned long lastUpdateTime = 0;  // Variable to store the last update time
unsigned long samplingInterval = 100;
unsigned long lastDebounceTimeBath = 0;
unsigned long lastDebounceTimeRoom = 0;
unsigned long debounceDelay_BTN = 200;
unsigned long lastDetectionTime1 = 0;
unsigned long lastDetectionTime2 = 0;
unsigned long debounceDelay_PIR = 200;
unsigned long micDangerTimer = 0;  // 마이크 위험 상태 지속 시간을 추적하기 위한 타이머
unsigned long servoStartTime = 0;
const unsigned long servoDuration = 10000;  // 10초
/* Variables */
float calcR0 = 0;
float calcR1 = 0;
float calcR2 = 0;

int vresistorDanger_status = 0;
int previousReading = 0;
int previousMicValue = 0;


bool lastButtonStateBath = LOW;
bool lastButtonStateRoom = LOW;

bool lastBellState = LOW;

Servo servoMotor;
bool lastMotorButtonState = LOW;


int pirState1 = LOW;
int pirState2 = LOW;
int lastPirState1 = LOW;
int lastPirState2 = LOW;


/* Danger Thresholds*/
int vresistorThreshold = 10;
int micThreshold = 100;

/* Datasets to go to DB */
/* analog data*/
float dustDensity = 0;
float MQ7_PPM = 0;
float MQ135_PPM = 0;
float MQ2_PPM = 0;
int resistanceValue = 0;
int micValue = 0;

/* digital data*/
bool buttonStateBath = LOW;
bool buttonStateRoom = LOW;
bool LED_BATH_STATUS = LOW;
bool LED_ROOM_STATUS = LOW;
bool bellState = LOW;

bool buttonState = LOW;
bool servoTurned = false;

int motion1 = 0;
int motion2 = 0;
int micDanger_status = 0;
int danger_status = 0;

/* Function Declaration  */
void dustsensor();
MQUnifiedsensor MQ7(board, Voltage_Resolution, ADC_Bit_Resolution, MQ7_PIN, MQ7_type);
MQUnifiedsensor MQ135(board, Voltage_Resolution, ADC_Bit_Resolution, MQ135_PIN, MQ135_type);
MQUnifiedsensor MQ2(board, Voltage_Resolution, ADC_Bit_Resolution, MQ2_PIN, MQ2_type);
int readVariableResistance();
int readMicValue();
void updateStatus(int resistance);
void updateDangerStatus(int micValue);
void processBathButton();
void processRoomButton();
void checkBellButton();
void ringBuzzer();
void checkButton();
int readPIR(int pirPin, unsigned long &lastDetectionTime, int &pirState, int &lastPirState);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  MQ7.setRegressionMethod(1);  // _PPM =  a*ratio^b
  MQ7.setA(99.042);
  MQ7.setB(-1.518);              // Configure the equation values to get CO concentration
  MQ135.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ135.setA(102.2);
  MQ135.setB(-2.473);          // Configure the equation to to calculate NH4 concentration
  MQ2.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ2.setA(574.25);
  MQ2.setB(-2.222);  // Configure the equation to to calculate LPG concentration

  MQ7.init();
  MQ135.init();
  MQ2.init();

  // Serial.println("Calibrating please wait.");
  for (int i = 1; i <= 10; i++) {
    MQ7.update();    // Update data, the Arduino will read the voltage on the analog pin
    MQ135.update();  // Update data, the arduino will read the voltage from the analog pin
    MQ2.update();

    calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
    calcR1 += MQ135.calibrate(RatioMQ135CleanAir);
    calcR2 += MQ135.calibrate(RatioMQ2CleanAir);
  }
  MQ7.setR0(calcR0 / 10);
  MQ135.setR0(calcR2 / 10);
  MQ2.setR0(calcR2 / 10);

  pinMode(BTN_LED_BATH_PIN, INPUT);
  pinMode(LED_BATH_PIN, OUTPUT);
  pinMode(BTN_LED_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  // pinMode(MIC_PIN, INPUT);

  pinMode(BTN_BELL_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(BTN_GAS_VALVE_PIN, INPUT);
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(0);

  pinMode(PIR_BATH_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(DUST_FAN, OUTPUT);
  pinMode(GAS_BUZZER, OUTPUT);
}

void loop() {

  if (buttonPress()) {
    if (!servoTurned) {
      servoMotor.write(90);  // 서보모터를 90도 회전
      servoTurned = true;
      servoStartTime = millis();  // 서보모터가 시작된 시간 기록
    } else {
      servoMotor.write(0);
      servoTurned = false;
    }
  } else {  // 버튼이 눌리지 않은 경우
    // 서보모터가 켜진 상태이고, 현재 시간이 시작 시간으로부터 10초가 지났을 때
    if (servoTurned && (millis() - servoStartTime >= servoDuration)) {
      servoMotor.write(0);  // 서보모터를 0도로 회전
      servoTurned = false;
    }
  }
  unsigned long currentTime = millis();
  processBathButton();
  processRoomButton();

  checkBellButton();

  // checkButton();

  int motion1 = digitalRead(PIR_BATH_PIN);
  int motion2 = digitalRead(PIR_PIN);

  // motion1 = readPIR(PIR_BATH_PIN, lastDetectionTime1, pirState1, lastPirState1);
  // motion2 = readPIR(PIR_PIN, lastDetectionTime2, pirState2, lastPirState2);

  if (currentTime - lastUpdateTime >= samplingInterval) {
    lastUpdateTime = currentTime;  // Update the last update time

    // Update MQ sensor data
    MQ7.update();
    MQ135.update();
    MQ2.update();

    // Read sensor data
    dustsensor();
    MQ7_PPM = MQ7.readSensor();
    MQ135_PPM = MQ135.readSensor();
    MQ2_PPM = MQ2.readSensor();
    resistanceValue = readVariableResistance();
    micValue = analogRead(MIC_PIN);
    updateDangerStatus(micValue, motion1, motion2);
    // Print sensor data if needed
    // Serial.print("Dust density (mg/m^3) : ");
    Serial.print(dustDensity);
    Serial.print(", ");
    // MQ7.serialDebug();
    //Serial.print("MQ-7 : ");
    Serial.print(MQ7_PPM);
    Serial.print(", ");
    // MQ135.serialDebug();
    //Serial.print("MQ-135 : ");
    Serial.print(MQ135_PPM);
    Serial.print(", ");
    // MQ2.serialDebug();
    //Serial.print("MQ-2 : ");
    Serial.print(MQ2_PPM);
    Serial.print(", ");
    // Serial.print("VResistor : ");
    // Serial.print("GAS : ");
    Serial.print(resistanceValue);
    Serial.print(", ");
    // Serial.print("Sound : ")
    Serial.print(micValue);
    Serial.print("|");

    //Serial.print("btnBath : ");
    Serial.print(buttonStateBath);
    Serial.print(", ");
    //Serial.print("btnRoom : ");
    Serial.print(buttonStateRoom);
    Serial.print(", ");
    //Serial.print("LEDBath : ");
    Serial.print(LED_BATH_STATUS);
    Serial.print(", ");
    //Serial.print("LEDRoom : ");
    Serial.print(LED_ROOM_STATUS);
    Serial.print(", ");
    //Serial.print("bell : ");
    Serial.print(bellState);
    Serial.print("|");

    //Serial.print("servo : ");
    Serial.print(buttonState);
    Serial.print(", ");
    // Serial.print("servostate : ");
    Serial.print(servoTurned);
    Serial.print("|");

    //Serial.print("PIR1 : ");
    Serial.print(motion1);
    Serial.print(", ");
    //Serial.print("PIR2 : ");
    Serial.print(motion2);
    Serial.print(", ");
    //Serial.print("micstate : ");
    Serial.print(micDanger_status);
    Serial.print(", ");
    //Serial.print("위험단계 : ");
    Serial.print(danger_status);
  }
  Serial.println();
  if (MQ2_PPM > 2.9) {
    digitalWrite(GAS_BUZZER, HIGH);

  } else {
    digitalWrite(GAS_BUZZER, LOW);
  }
}


/* Function Definitions */
void dustsensor() {
  float dustMeasured = analogRead(DUSTSENSOR_PIN);  // Read the dust value
  // Convert to voltage
  float calcVoltage = dustMeasured * (5.0 / 1024.0);
  // Calculate dust density
  dustDensity = 170 * calcVoltage - 0.1;
  if (dustDensity > 210) {
    digitalWrite(DUST_FAN, HIGH);

  } else {
    digitalWrite(DUST_FAN, LOW);
  }

  return dustDensity;
}
// 3000 +     = VERY POOR
// 1050-3000  = POOR
// 300-1050   = FAIR
// 150-300    = GOOD
// 75-150     = VERY GOOD
// 0-75       = EXCELLENT

// Function to read the variable resistance value
int readVariableResistance() {
  // Read the analog input
  int sensorValue = analogRead(VRESISOTR_PIN);

  // Map the sensor value (0-1023) to the desired range (0-100)
  int resistanceValue = map(sensorValue, 0, 1023, 0, 100);

  return resistanceValue;
}
// Function to update status based on the change in resistance
void updateStatus(int resistance) {
  // Calculate the change in resistance value
  int resistanceChange = abs(resistance - previousReading);

  if (resistanceChange > vresistorThreshold) {
    vresistorDanger_status = 0;  // Change detected
  } else {
    vresistorDanger_status = 1;  // No change
  }
}


// Function to check the danger of loud noise
void updateDangerStatus(int micValue, int motion1, int motion2) {
  if (micValue > micThreshold && (motion1 == 0 && motion2 == 0)) {
    // 소리 센서 값이 임계값을 초과하고 PIR 센서에서 움직임이 감지되지 않을 때
    danger_status = 2;  // 위험도 2단계
  } else if (micValue > micThreshold && (motion1 == 1 || motion2 == 1)) {
    // 소리 센서 값이 임계값을 초과하거나 PIR 센서에서 움직임이 감지될 때
    danger_status = 1;  // 위험도 1단계
  } else {
    // 소리 센서 값이 임계값 이하이고 PIR 센서에서 움직임이 감지되지 않을 때
    danger_status = 0;  // 위험도 0단계
  }
}

void processBathButton() {
  int readingBath = digitalRead(BTN_LED_BATH_PIN);

  if (readingBath != lastButtonStateBath) {
    lastDebounceTimeBath = millis();
  }

  if ((millis() - lastDebounceTimeBath) > debounceDelay_BTN) {
    if (readingBath != buttonStateBath) {
      buttonStateBath = readingBath;

      if (buttonStateBath == HIGH) {
        LED_BATH_STATUS = !LED_BATH_STATUS;
        digitalWrite(LED_BATH_PIN, LED_BATH_STATUS);
      }
    }
  }

  lastButtonStateBath = readingBath;
}

void processRoomButton() {
  int readingRoom = digitalRead(BTN_LED_PIN);

  if (readingRoom != lastButtonStateRoom) {
    lastDebounceTimeRoom = millis();
  }

  if ((millis() - lastDebounceTimeRoom) > debounceDelay_BTN) {
    if (readingRoom != buttonStateRoom) {
      buttonStateRoom = readingRoom;

      if (buttonStateRoom == HIGH) {
        LED_ROOM_STATUS = !LED_ROOM_STATUS;
        digitalWrite(LED_PIN, LED_ROOM_STATUS);
      }
    }
  }

  lastButtonStateRoom = readingRoom;
}

// A function that checks the status of the bell button and rings the buzzer
void checkBellButton() {
  bellState = digitalRead(BTN_BELL_PIN);

  if (bellState != lastBellState) {
    if (bellState == HIGH) {

      ringBuzzer();
    }
  }
  lastBellState = bellState;
}


void ringBuzzer() {
  // 부저 울리기 함수
  unsigned long lastToneTime = 0;
  unsigned long toneInterval = 100;
  unsigned long currentTime = millis();

  if (currentTime - lastToneTime >= toneInterval) {
    tone(BUZZER_PIN, 262, 500);
    lastToneTime = currentTime;
  } else if (currentTime - lastToneTime >= toneInterval / 2) {
    tone(BUZZER_PIN, 294, 500);
  } else if (currentTime - lastToneTime >= toneInterval / 3) {
    tone(BUZZER_PIN, 330, 500);
  } else if (currentTime - lastToneTime >= toneInterval / 4) {
    tone(BUZZER_PIN, 349, 500);
  } else if (currentTime - lastToneTime >= toneInterval / 5) {
    tone(BUZZER_PIN, 392, 500);
  } else if (currentTime - lastToneTime >= toneInterval / 6) {
    tone(BUZZER_PIN, 440, 500);
  } else if (currentTime - lastToneTime >= toneInterval / 7) {
    tone(BUZZER_PIN, 494, 500);
  } else if (currentTime - lastToneTime >= toneInterval / 8) {
    tone(BUZZER_PIN, 523, 500);
  }
}

int buttonPress() {
  int press;
  int buttonState;
  static int prevButtonState = LOW;
  buttonState = digitalRead(BTN_GAS_VALVE_PIN);
  press = (buttonState == HIGH && prevButtonState == LOW);
  prevButtonState = buttonState;
  return press;
}

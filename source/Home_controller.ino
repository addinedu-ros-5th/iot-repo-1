/* Library */
#include <MQUnifiedsensor.h>


/* Board Information */
#define board "Arduino UNO"
#define Voltage_Resolution 5
#define ADC_Bit_Resolution 10 // For Arduino UNO/MEGA/NANO

/* MQ-7 Sensor Information*/
#define MQ7_type "MQ-7" // MQ7
#define RatioMQ7CleanAir 27.5 // RS / R0 = 27.5 ppm

/* MQ-135 Sensor Information*/
#define MQ135_type "MQ-135" // MQ135
#define RatioMQ135CleanAir 3.6 // RS / R0 = 3.6 ppm  

/* MQ-2 Sensor Information*/

#define MQ2_type "MQ-2" // MQ2
#define RatioMQ2CleanAir 9.83// RS / R0 = 9.83 ppm  

/* Pin Information */
const int DUSTSENSOR_PIN = A0;
const int MQ7_PIN = A1;
const int MQ135_PIN = A2;
const int MQ2_PIN = A3;
const int VRESISOTR_PIN = A4;
const int MIC_PIN = A5;

const int BATH_LED_PIN = 2;
const int BATH_PIR_PIN = 3;
const int LED_PIN = 4;
const int PIR_PIN = 5;
const int BTN_BELL_PIN = 6;
const int BUZZER_PIN = 7 ;

/* Constants */
unsigned long lastUpdateTime = 0; // Variable to store the last update time
unsigned long samplingInterval = 1000;

/* Variables */
float calcR0 = 0;
float calcR1 = 0;
float calcR2 = 0;

int previousReading = 0;
int previousMicValue = 0;
int danger_status = 0;


/* Danger Thresholds*/
int vresistorThreshold = 10;
int micThreshold = 100;

/* Datasets to go to DB */
float dustDensity = 0;
float MQ7_PPM = 0;
float MQ135_PPM = 0;
float MQ2_PPM = 0;

int resistanceValue = 0;
int vresistorDanger_status = 1;
int micValue =0; 
int micDanger_status=0;


/* Function Declaration  */
void dustsensor();
MQUnifiedsensor MQ7 (board, Voltage_Resolution, ADC_Bit_Resolution, MQ7_PIN, MQ7_type);
MQUnifiedsensor MQ135(board, Voltage_Resolution, ADC_Bit_Resolution, MQ135_PIN, MQ135_type);
MQUnifiedsensor MQ2(board, Voltage_Resolution, ADC_Bit_Resolution, MQ2_PIN, MQ2_type);
int readMicValue();
void updateStatus(int resistance);
void updateDangerStatus(int micValue);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);

  MQ7.setRegressionMethod(1); // _PPM =  a*ratio^b
  MQ7.setA(99.042); MQ7.setB(-1.518); // Configure the equation values to get CO concentration
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setA(102.2); MQ135.setB(-2.473); // Configure the equation to to calculate NH4 concentration
  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(574.25); MQ2.setB(-2.222); // Configure the equation to to calculate LPG concentration

  MQ7.init();
  MQ135.init();
  MQ2.init(); 

  Serial.println("Calibrating please wait.");
  for(int i = 1; i <= 10; i++) {
    MQ7.update(); // Update data, the Arduino will read the voltage on the analog pin
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    MQ2.update();
    
    calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
    calcR1 += MQ135.calibrate(RatioMQ135CleanAir);
    calcR2 += MQ135.calibrate(RatioMQ2CleanAir);
  }
  MQ7.setR0(calcR0/10);
  MQ135.setR0(calcR2/10);
  MQ2.setR0(calcR2/10);

}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= samplingInterval) {
    lastUpdateTime = currentTime; // Update the last update time
    
    // Update MQ sensor data
    MQ7.update();
    MQ135.update();
    MQ2.update();

    // Read sensor data
    MQ7_PPM = MQ7.readSensor();
    MQ135_PPM = MQ135.readSensor();
    MQ2_PPM = MQ2.readSensor();
    dustsensor();
    resistanceValue = readVariableResistance();
    micValue = analogRead(MIC_PIN);
    
    // Print sensor data if needed
    Serial.print("Dust density (mg/m^3) : ");
    Serial.println(dustDensity); 
    MQ7.serialDebug();
    MQ135.serialDebug();
    MQ2.serialDebug();
    Serial.print("VResistor : ");
    Serial.println(resistanceValue);
    Serial.print("Sound : ");
    Serial.println(micValue);
  }
}

/* Function Definitions */ 
// 
void dustsensor(){
  float dustMeasured = analogRead(DUSTSENSOR_PIN); // Read the dust value
  // Convert to voltage
  float calcVoltage = dustMeasured * (5.0 / 1024.0);
  // Calculate dust density
  dustDensity = 170 * calcVoltage - 0.1;
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
    vresistorDanger_status = 0; // Change detected
  } else {
    vresistorDanger_status = 1; // No change
  }
}


// Function ot check the danger of loud noise
void updateDangerStatus(int micValue) {
  if (micValue - previousMicValue > micThreshold) {
    micDanger_status = 1;
  } else {
    micDanger_status = 0;
  }
  
  previousMicValue = micValue;
}

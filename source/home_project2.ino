#include <Servo.h>


int flag = 1;
Servo servo;

unsigned long lightstart = 0;
unsigned long lightclose = 0;
const unsigned long lightdelay = 3000;

void setup() {
  Serial.begin(9600);
  servo.attach(8);
  servo.write(10);
}

void loop() {
  int light = analogRead(A0);
  int scaledLight = map(light, 0, 1023, 0, 100); 


  Serial.println(scaledLight);


  if (scaledLight < 50)
  {
    if (flag == 0) 
    {
      if (lightstart == 0) 
      { 
        lightstart = millis();
        
      }
      if ( millis() - lightstart >= lightdelay) 
      {
        servo.write(100);
        flag = 1;
        lightclose = 0;
        
      }
    }
  }
  else
  {
    if (flag == 1)
    {
      if (lightclose == 0)
      {
        lightclose = millis();
      }
      if (millis() - lightclose >= lightdelay)
      {
        servo.write(10);
        flag = 0;
        lightstart = 0;     
      }
    }

    if (Serial.available() > 0) {
    int angle = Serial.parseInt(); // 시리얼 포트로부터 각도를 읽음
    servo.write(angle); // 서보모터를 해당 각도로 회전
  }
  }
  delay(500);
  

}
#include "AS5600.h"

#define IN1 12
#define IN2 13
#define PWM1 11
#define PWM2 10
#define IN3 8
#define IN4 9

AS5600 as5600;
int sense = 0;  
double angle1, angle2;
long previousMillis = 0;

void configMotor (int motor, int direction, float PWM) {
  if (motor == 1) {
    digitalWrite(IN1, !direction);
    digitalWrite(IN2, direction);
    analogWrite(PWM1, PWM);
  } else {
    digitalWrite(IN3, !direction);
    digitalWrite(IN4, direction);
    analogWrite(PWM2, PWM);
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // Config H bridge
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  delay(1000);
}
double lastPos = -1;
void loop() {
  configMotor(2, 1, 0);
  configMotor(1, 0, 0);
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 5) {
    previousMillis = currentMillis;
    //angle2= map(analogRead(A0), 0, 1023, 0, 360); // 12 Bits
    angle1 = 63 - map(as5600.getCumulativePosition(), 0, 4095, 0, 360); // 12 Bits
    angle2 =  -138 + map(analogRead(A0), 0, 1023, 0, 360); // 12 Bits
    Serial.print(angle1);
    Serial.print(",");
    Serial.println(angle2);
    //Serial.print(",");
    //Serial.println(currentMillis);
  }
}

#include "AS5600.h"

#define PWM1 10
#define PWM2 11
#define IN1 8
#define IN2 9
#define IN3 12
#define IN4 13

AS5600 as5600;
int sense = 0;
float angle1, angle2;
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
  as5600.begin();
  as5600.resetPosition(0);
  as5600.setDirection(AS5600_CLOCK_WISE);
  // Config H bridge
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.println("LABEL,Theta_1,t");
  delay(1000);
}
double lastPos = -1;
void loop() {
  configMotor(2, 1, 40);
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1) {
    previousMillis = currentMillis;
    //angle2= map(analogRead(A0), 0, 1023, 0, 360); // 12 Bits
    angle1 = -4-map(as5600.getCumulativePosition(), 0, 4095, 0, 360); // 12 Bits
    double pos = map(analogRead(A0), 0, 1023, 0, 360);
    if (lastPos == 360) pos = 360 + pos;
    if (lastPos == 0) pos = pos - 360;
    lastPos = pos;
    angle2 = -143+map(analogRead(A0), 0, 1023, 0, 360); // 12 Bits
    Serial.print(angle1);
    Serial.print(",");
    Serial.print(angle2);
    Serial.print(",");
    Serial.println(currentMillis);
  }
}

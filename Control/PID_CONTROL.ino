#include "AS5600.h"

#define PWM1 9
#define PWM2 10
#define IN1 13
#define IN2 12
#define IN3 8
#define IN4 7

AS5600 as5600;

// Variables para PID
double kp = 7, kd = 1.308;
double setpoint = 160;
double input, output;
double lastError = 0, integral = 0;

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

void PID() {
  long pos = map(as5600.getCumulativePosition(), 0, 4095, 0, 360)-20; // 12 Bits
  input = pos;
  double error = setpoint - input;
  double Pout = kp * error;
  double derivative = error - lastError;
  double Dout = kd * derivative;
  output = Pout + Dout;
  if (output > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    output = -output;
  }
  if (output > 255) output = 255;
  if (output < 0) output = 0;
  analogWrite(PWM1, output); // Enviar señal PWM al motor
  lastError = error;

  Serial.print(input);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.println(error);
}

void loop() {
  PID();
}

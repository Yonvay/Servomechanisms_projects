#include "AS5600.h"

#define IN1 12
#define IN2 13
#define PWM1 11
#define PWM2 10
#define IN3 8
#define IN4 9

AS5600 as5600;

// Variables para PID
double kp = 2.65, kd = 0.1;
double setpoint = 40;
double input, output;
double lastError = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  as5600.begin();
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
  long pos = 335 - map(analogRead(A0), 0, 1023, 0, 360); // 12 Bits
  input = pos;
  double error = setpoint - input;
  double Pout = kp * error;
  double derivative = error - lastError;
  double Dout = kd * derivative;
  output = Pout + Dout;
  if (output > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    output = -output;
  }

  if (output > 255) output = 255;
  if (output < 0) output = 0;
  analogWrite(PWM2, output); // Enviar señal PWM al motor
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

#include "AS5600.h"
#define ENCODER2 A0
#define IN1 8
#define IN2 9
#define PWM1 10
#define PWM2 11
#define IN3 12
#define IN4 13

AS5600 as5600;

// Variables para PID
double P1 = 7, D1 = 1.308, P2 = 10, D2 = 1.72;
double input1, input2;
double output1, output2;
double lastError1 = 0, lastError2 = 0;
double offsetAngle1 = 23, offsetAngle2 = 40;

void setup() {
  //Config AS5600 I2C PROTOCOL
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

void PID1(double setPoint) {
  long pos = offsetAngle1 - map(as5600.getCumulativePosition(), 0, 4095, 0, 360); // 12 Bits
  input1 = pos;
  double error = setPoint - input1;
  double Pout = P1 * error;
  double Dout = D1 * (error - lastError1);
  output1 = Pout + Dout;
  if (output1 > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    output1 = -output1;
  }
  if (output1 > 255) output1 = 255;
  if (output1 == 255 | pos > 170 | output1 < 0) output1 = 0; //Evita que se rompan los acoples
  analogWrite(PWM1, output1); // Enviar señal PWM al motor
  lastError1 = error;
  /*Serial.print("M1, ");
  Serial.print(input1);
  Serial.print(", ");
  Serial.print(output1);
  Serial.print(", ");
  Serial.println(error);*/
}

void PID2(double setPoint) {
  long pos = offsetAngle2 - map(analogRead(ENCODER2), 0, 1023, 0, 360); // 10 Bits, analog output
  input2 = pos;
  double error = setPoint - input2;
  double Pout = P2 * error;
  double Dout = D2 * (error - lastError2);
  output2 = Pout + Dout;
  if (output2 > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    output2 = -output2;
  }
  if (output2 > 255) output2 = 255;
  if (output2 == 255 | pos > 170 | output2 < 0) output2 = 0; //Evita que se rompan los acoples
  analogWrite(PWM2, output2); // Enviar señal PWM al motor
  lastError2 = error;
  /*Serial.print("M2, ");
  Serial.print(input2);
  Serial.print(", ");
  Serial.print(output2);
  Serial.print(", ");
  Serial.println(error);*/
}

int steps = 600;
double sizes[8] = {1.46, 1.33, 0.8, 0.56, 0.38, 0.36, 0.3, 0.27};
double roundness[8] = {5, 7.5, 8.8, 13.8, 19.1, 20, 24.5, 28.2};
double offsetX = 13;
double offsetY = 24.5;

double* getAngles (int step, double scale, double rotation, int leafts) {
  static double angles[2];
  int index = leafts - 3;
  double t = step * 2 * PI / steps;
  double sinusoidalFunction = scale * sizes[index] * (cos(t * leafts + rotation) + roundness[index]);
  double x = sinusoidalFunction * cos(t) + offsetX;
  double y = sinusoidalFunction * sin(t) + offsetY;
  double beta = 2*atan2(y,x);
  double cos_gamma_2 = x/(2*20*cos(beta/2));
  double sin_gamma_2 = sqrt(1 - cos_gamma_2*cos_gamma_2);
  double gamma = 2*atan2(sin_gamma_2, cos_gamma_2);
  angles[0] = (beta + gamma)/2;
  angles[1] = beta - 2*angles[0];
  return angles;
}

double deg2Rad(double angle) {
  return angle * 180.0 / PI;
}

double* angles;
long Dt = 100;
long previousMillis = 0;
unsigned long currentMillis, stepTime;
int currentStep = 0;
double currentScale = 1.0;
double currentRotation = 3*PI/2;//Radians
int currentLeafts = 3;//Leafts > 3


void loop() {
  currentMillis = millis();
  stepTime = currentMillis - previousMillis;
  if (stepTime >= Dt) {//Asigna un nuevo punto pasado un tiempo Dt
    Serial.println(currentStep);
    angles = getAngles(currentStep, currentScale, currentRotation, currentLeafts);
    previousMillis = currentMillis;
    currentStep++;
    if (currentStep == steps - 1) currentStep = 0;//Vuelve a empezar
    Serial.print("ANGLE 1: ");
    Serial.println(angles[0]);
    Serial.print("ANGLE 2: ");
    Serial.println(angles[1]);
  }
  PID1(deg2Rad(angles[0]));
  PID2(deg2Rad(angles[1]));
}

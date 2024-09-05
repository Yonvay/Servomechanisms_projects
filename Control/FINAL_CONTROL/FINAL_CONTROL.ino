#include "AS5600.h"
#define ENCODER2 A0
#define STOP 2
#define HOME 3
#define RUN 4
//H Brigde
#define IN1 8
#define IN2 9
#define PWM1 10
#define PWM2 11
#define IN3 12
#define IN4 13

AS5600 as5600;

// Variables para PID
double P1 = 6, D1 = 1.03, P2 = 7, D2 = 1.36;
double input1, input2;
double output1, output2;
double lastError1 = 0, lastError2 = 0;
bool STOP_UP = false;

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

double offsetAngle1 = 119;

void PID1(double setPoint) {
  long pos = offsetAngle1 - map(as5600.getCumulativePosition(), 0, 4095, 0, 360);  // 12 Bits
  input1 = pos;
  double error = setPoint - input1;
  double Pout = P1 * error;
  double Dout = D1 * (error - lastError1);
  output1 = Pout + Dout;
  if (output1 > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    output1 = -output1;
  }
  if (output1 > 255) output1 = 255;
  if (output1 < 0) output1 = 0;  //Evita que se rompan los acoples
  analogWrite(PWM1, output1);                            // Enviar señal PWM al motor
  lastError1 = error;
}

double offsetAngle2 = -125;
double lastPos = -1;

void PID2(double setPoint) {
  long pos = offsetAngle2 + map(analogRead(ENCODER2), 0, 1023, 0, 360);
  input2 = pos;
  double error = setPoint - input2;
  double Pout = P2 * error;
  double Dout = D2 * (error - lastError2);
  output2 = Pout + Dout;
  if (output2 > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    output2 = -output2;
  }
  if (output2 > 255) output2 = 255;
  if (output2 < 0) output2 = 0;  //Evita que se rompan los acoples
  analogWrite(PWM2, output2);                // Enviar señal PWM al motor
  lastError2 = error;
}

int steps = 40;
double sizes[8] = { 1.46, 1.33, 0.8, 0.56, 0.38, 0.36, 0.3, 0.27 };  //10 petalos
double roundness[8] = { 5, 7.5, 8.8, 13.8, 19.1, 20, 24.5, 28.2 };
double offsetX = 13;
double offsetY = 24.5;

double* getAngles(int step, double scale, double rotation, int leafts) {
  static double angles[2];
  int index = leafts - 3;
  double t = step * 2 * PI / steps;
  double sinusoidalFunction = scale * sizes[index] * (cos(t * leafts + rotation) + roundness[index]);
  double x = sinusoidalFunction * cos(t) + offsetX;
  double y = sinusoidalFunction * sin(t) + offsetY;
  double beta = 2 * atan2(y, x);
  double cos_gamma_2 = x / (2 * 20 * cos(beta / 2));
  double sin_gamma_2 = sqrt(1 - cos_gamma_2 * cos_gamma_2);
  double gamma = 2 * atan2(sin_gamma_2, cos_gamma_2);
  angles[0] = (beta + gamma) / 2;
  angles[1] = beta - 2 * angles[0];
  return angles;
}

double rad2Deg(double angle) {
  return angle * 180.0 / PI;
}

double* angles;
long Dt = 500;
long previousMillis = 0;
unsigned long currentMillis, stepTime;
int currentStep = 0;
double currentScale = 1.0;
double currentRotation = 0;  //Radians
int currentLeafts = 3;       //Leafts > 3

void loop() {
  currentMillis = millis();
  if (digitalRead(STOP) == 1 || STOP_UP) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    Serial.println("PARADA DE EMERGENCIA");
    STOP_UP = true;
  }
  if (!STOP_UP && digitalRead(HOME) == 1) {
    Serial.println("HOME");
    currentStep = 0;
    PID1(90);
    PID2(-90);
  } else if (!STOP_UP && digitalRead(RUN) == 1) {
    stepTime = currentMillis - previousMillis;
    if (stepTime >= Dt) {  //Asigna un nuevo punto pasado un tiempo Dt
      angles = getAngles(currentStep, currentScale, currentRotation, currentLeafts);
      previousMillis = currentMillis;
      currentStep++;
      if (currentStep == steps + 1) {//Termina y se detiene
        STOP_UP = true;
        currentStep = 0;
      };
      Serial.print("Theta_1T: ");
      Serial.print(rad2Deg(angles[0]));
      Serial.print(", Theta_1R: ");
      Serial.println(input1);
      Serial.print("Theta_2T: ");
      Serial.print(rad2Deg(angles[1]));
      Serial.print(", Theta_2R: ");
      Serial.print(input2);
    }
    PID1(rad2Deg(angles[0]));
    PID2(rad2Deg(angles[1]));
  }
}

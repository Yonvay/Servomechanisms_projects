#include "AS5600.h"
#include <ArduinoJson.h>
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

// Tamaño máximo del buffer JSON
const size_t capacity = JSON_OBJECT_SIZE(3) + 3*JSON_OBJECT_SIZE(1) + 10;
DynamicJsonDocument doc(capacity);

AS5600 as5600;

// Variables para PID
double P1 = 7, D1 = 1.308, P2 = 10, D2 = 1.72;
double input1, input2;
double output1, output2;
double lastError1 = 0, lastError2 = 0;
bool STOP_UP = false;
bool Inicio = false;

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
  // Serial.println("LABEL,Theta_1,t");
  delay(1000);
}

double offsetAngle1 = -4;

void PID1(double setPoint) {
  long pos = offsetAngle1 - map(as5600.getCumulativePosition(), 0, 4095, 0, 360); // 12 Bits
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
  if (pos > 170 || pos < 0 || output1 < 0) output1 = 0; //Evita que se rompan los acoples
  analogWrite(PWM1, output1); // Enviar señal PWM al motor
  lastError1 = error;
  doc["a"] = pos;
  //Serial.print("I1: ");
  //Serial.print(input1);
  //Serial.print(", O1: ");
  //Serial.print(output1);
  //Serial.print(", ");
  //Serial.println(error);
}

double offsetAngle2 = -143;
double lastPos = -1;

void PID2(double setPoint) {
  long pos2 = map(analogRead(ENCODER2), 0, 1023, 0, 360);
  if (lastPos == 360 && pos2 < 10) pos2 = 360 + pos2;
  if (lastPos == 0 && pos2 > 350) pos2 = pos2 - 360;
  long pos = offsetAngle2 + pos2; // 10 Bits, analog output
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
  if (pos > 90 || output2 < 0) output2 = 0; //Evita que se rompan los acoples
  analogWrite(PWM2, output2); // Enviar señal PWM al motor
  lastError2 = error;
  doc["b"] = pos2;
  //Serial.print("I2: ");
  //Serial.print(input2);
 // Serial.print(", O2: ");
  //Serial.print(output2);
  //Serial.print(", ");
  //Serial.println(error);
}

int steps = 180;
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

double rad2Deg(double angle) {
  return angle * 180.0 / PI;
}


double* angles;
long Dt = 100;
long previousMillis = 0;
unsigned long currentMillis, stepTime;
int currentStep = 0;
double currentScale = 1.0;
double currentRotation = 0;//Radians
int currentLeafts = 3;//Leafts > 3

void loop() {
    // Si hay datos disponibles en el puerto serial
  if (Serial.available() > 0) {
    // Lee el JSON completo
    String input = Serial.readStringUntil('\n');
    
    // Deserializa el JSON
    DeserializationError error = deserializeJson(doc, input);

    // Si no hay errores en la deserialización
    if (!error) {
      // Verifica si el valor de "ini" es 1
      if (doc["ini"] == 1) {
        // Actualiza los valores 
        if (doc.containsKey("Escala")) {
          currentScale = doc["Escala"];
        }
        if (doc.containsKey("Rotacion")) {
          currentRotation = doc["Rotacion"].as<double>() * (PI / 180.0);

        }
        // Habilita el inicio
        Inicio = true;
      }
    }
  }

  
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
  }else if (!STOP_UP && Inicio) {
    currentMillis = millis();
    stepTime = currentMillis - previousMillis;
    if (stepTime >= Dt) {//Asigna un nuevo punto pasado un tiempo Dt
      Serial.println(currentStep);
      angles = getAngles(currentStep, currentScale, currentRotation, currentLeafts);
      previousMillis = currentMillis;
      currentStep++;
      if (currentStep == steps - 1) currentStep = 0;//Vuelve a empezar
      Serial.print("ANGLE 1: ");
      Serial.println(rad2Deg(angles[0]));
      Serial.print("ANGLE 2: ");
      Serial.println(rad2Deg(angles[1]));
    }
    PID1(rad2Deg(angles[0]));
    PID2(rad2Deg(angles[1]));
        // Serializa el JSON a un string
    String output;
    serializeJson(doc, output);

    // Imprime el JSON por el monitor serial
    Serial.println(output);
  }
}

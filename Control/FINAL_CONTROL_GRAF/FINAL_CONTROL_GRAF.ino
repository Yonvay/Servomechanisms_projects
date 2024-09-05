#include "AS5600.h"
#include <ArduinoJson.h>
#define ENCODER2 A0
#define STOP 2
#define HOME 3
#define RUN 4
//H Brigde
#define IN1 12
#define IN2 13
#define PWM1 11
#define PWM2 10
#define IN3 8
#define IN4 9

// Tamaño máximo del buffer JSON
const size_t capacity = JSON_OBJECT_SIZE(3) + 3*JSON_OBJECT_SIZE(1) + 10;
DynamicJsonDocument doc(capacity);

AS5600 as5600;

// Variables para PID
double P1 = 7, D1 = 1.16, P2 = 10, D2 = 1.72;
double input1, input2;
double output1, output2;
double lastError1 = 0, lastError2 = 0;
bool STOP_UP = false;
bool Inicio = false;

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
  delay(1000);
}

double offsetAngle1 = 23;
double offsetAngle2 = -406;

void PID(double setPoint1, double setPoint2) {
  input1 = offsetAngle1 - map(as5600.getCumulativePosition(), 0, 4095, 0, 360); // 12 Bits
  input2 = offsetAngle2 + map(analogRead(ENCODER2), 0, 1023, 0, 360);// 10 Bits, analog output
  double error1 = setPoint1 - input1;
  double error2 = setPoint2 - input2;
  output1 = P1 * error1 + D1 * (error1 - lastError1);
  output2 = P2 * error2 + D2 * (error2 - lastError2);;
  
  if (output1 > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    output1 = -output1;
  }

  if (output2 > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    output2 = -output2;
  }
  if (output2 > 255) output2 = 255;
  if (output2 < 0) output2 = 0; //Evita que se rompan los acoples
  
  if (output1 > 255) output1 = 255;
  if (output1 < 0) output1 = 0; //Evita que se rompan los acoples

  analogWrite(PWM1, output1); // Enviar señal PWM al motor
  analogWrite(PWM2, output2); // Enviar señal PWM al motor

  lastError1 = error1;
  lastError2 = error2;
}

int steps = 40;
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
long Dt = 5000;
long previousMillis = 0;
unsigned long currentMillis, stepTime;
int currentStep = 0;
double currentScale = 1.0;
double currentRotation = 0;//Radians
int currentLeafts = 3;//Leafts > 3

void loop() {
  currentMillis = millis();
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
        if (doc.containsKey("Petalos")) {
          currentLeafts = doc["Petalos"];
        }
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
  Inicio = true;
  if (Inicio) {
    if (digitalRead(STOP) == 1 || STOP_UP) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    Serial.println("PARADA DE EMERGENCIA");
    STOP_UP = true;
    }
    if (!STOP_UP && digitalRead(HOME) == 1) {
      currentStep = 0;
      angles = getAngles(currentStep, currentScale, currentRotation, currentLeafts);
      PID(rad2Deg(angles[0]), rad2Deg(angles[1]));
    }else if (!STOP_UP && RUN) {
      stepTime = currentMillis - previousMillis;
      if (stepTime >= Dt) {//Asigna un nuevo punto pasado un tiempo Dt
        Serial.println(currentStep);
        angles = getAngles(currentStep, currentScale, currentRotation, currentLeafts);
        previousMillis = currentMillis;
        currentStep++;
        if (currentStep >= steps) currentStep = steps;//Vuelve a empezar
        doc["a"] = input1;
        doc["b"] = input2;
        Serial.print(rad2Deg(angles[0]));
        Serial.println(rad2Deg(angles[1]));
        //Serializa el JSON a un string
        String output;
        serializeJson(doc, output);
        //Imprime el JSON por el monitor serial
        Serial.println(output);
      }
      PID(rad2Deg(angles[0]), rad2Deg(angles[1]));
    }
  }
}

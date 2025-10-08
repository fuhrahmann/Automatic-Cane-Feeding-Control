#include "arduino_secrets.h"

const byte pwmMotorPin = 5;  // Pin PWM ke inverter AI1
const byte hallSensorPin = 2;
const byte proximityLevelPins[] = { 3, 4, 6, 7, 8 };  // 5 IR sensors for level detection
float rpm = 0;
volatile unsigned long pulseCount = 0;
unsigned long lastTime;
unsigned long lastPulseTime;
float kFactor = 5000.0;
float Kp = 2.0, Ki = 0.5, Kd = 1.0;
const int setpointLevel = 3;  // Target level (middle level)
float error, lastError = 0, integral = 0, derivative = 0;
int outputPWM = 0;

void setup() {
  Serial.begin(115200);

  pinMode(hallSensorPin, INPUT_PULLUP);
  for (int i = 0; i < 5; i++) {
    pinMode(proximityLevelPins[i], INPUT);
  }
  pinMode(pwmMotorPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(hallSensorPin), countPulse, RISING);

  Serial.print("Automatic Cane Feeding Control\n");
  delay(2000);
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // Dalam detik
  // RPM Count
  if (currentTime - lastTime >= 1000) {  // Hitung RPM setiap 1 detik
    noInterrupts();
    rpm = (pulseCount / kFactor) * (60000.0 / (currentTime - lastTime));
    Serial.print("Pulse Count = " + String(pulseCount) + "\t");
    Serial.print("RPM = " + String(rpm) + "\n");
    pulseCount = 0;  // Reset counter dan timer
    lastTime = millis();
    interrupts();
  }

  // 1. Read current cane level from IR sensors
  int currentLevel = readCaneLevel();

  // 2. Manual PID Calculation
  error = setpointLevel - currentLevel;

  // Proportional term
  float proportional = Kp * error;

  // Integral term (with anti-windup)
  integral += error * deltaTime;
  integral = constrain(integral, -100, 100);  // Batas integral windup

  // Derivative term
  derivative = (error - lastError) / deltaTime;
  lastError = error;

  // Calculate PID output
  outputPWM = proportional + Ki * integral + Kd * derivative;
  outputPWM = constrain(outputPWM, 0, 255);  // Batas output PWM

  // outputPWM = 128;
  // Serial.println(outputPWM);
  // 3. Adjust motor speed through VFD
  analogWrite(pwmMotorPin, outputPWM);
  delay(100);
}

void countPulse() {
  if (digitalRead(hallSensorPin) == HIGH) pulseCount++;
  // static unsigned long lastInterruptTime = 0;
  // unsigned long interruptTime = millis();

  // // Debounce check
  // if (interruptTime - lastInterruptTime >= 20) {
  //   pulseCount++;
  //   lastPulseTime = interruptTime;
  // }
  // lastInterruptTime = interruptTime;
  // pulseCount++;
}

int readCaneLevel() {
  // Read all IR sensors and determine current level
  int activeSensors = 0;
  int levelSum = 0;

  for (int i = 0; i < 5; i++) {
    int sensorValue = digitalRead(proximityLevelPins[i]);
    if (sensorValue == LOW) {  // LOW Sensor Trigger Detection
      activeSensors++;
      levelSum += (i + 1);
    }
  }

  if (activeSensors == 0) return 0;  // No cane detected

  // Return average level
  return round((float)levelSum / activeSensors);
}

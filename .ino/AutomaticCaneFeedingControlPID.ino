const byte rpmSensorPin = 2;                            // RPM Sensor Pin
const byte pwmMotorPin = 5;                           // PWM to Inverter AI1
const byte proximityLevelPins[] = { 3, 4, 6, 7, 8 };  // 5 IR sensors for level detection
int proximitySensorValue[5];
unsigned long lastTime;
// double Kp = 920.4888, Ki = 576.1463, Kd = 22.8212;
// double Kp = 50.0, Ti = 0.5955, Td = 0.148875;  // Trial Error Value
double Kp = 60.0, Ki = 1.168, Kd = 770.82;  // Zn2 Value PID Ti=51.389, Td=12.487
// double Kp = 45.0, Ki = 0.525;  // Zn2 Value PI Ti=85.648
// double Kp = 462.8, Ki = 6.25, Kd = 8554.35;  // MATLAB PID Tune
double setpointLevel = 4.0;  // Target level (80% chute)
double error, lastError = 0, integral = 0, derivative = 0;
double pidValue = 0;
int outputPWM = 0;
double dutyCycle = 0;

void setup() {
  Serial.begin(115200);

  pinMode(rpmSensorPin, INPUT_PULLUP);
  for (int i = 0; i < 5; i++) {
    pinMode(proximityLevelPins[i], INPUT);
  }
  pinMode(pwmMotorPin, OUTPUT);
  attachInterrupt(rpmSensorPin, pulseCount, RISING);
  delay(5000);
  Serial.println("Automatic Cane Feeding Control");
  Serial.println("Error,PIDValue,PWM,CurrentLevel,DutyCycle");  // Data Log Format
}

void loop() {
  unsigned long currentTime = millis();
  double deltaTime = 0.1;  // s
  // Read current cane level from IR sensors
  int currentLevel = readCaneLevel();

  // Manual PID Calculation
  error = setpointLevel - (double)currentLevel;

  // Proportional term
  float proportional = Kp * error;

  // Integral term (with anti-windup)
  integral += error * deltaTime;
  integral = constrain(integral, -100, 100);  // Anti-windup windup

  // Derivative term
  derivative = (error - lastError) / deltaTime;
  lastError = error;

  // Calculate PID output
  pidValue = proportional + (Ki * integral) + (Kd * derivative);  // PID
  // pidValue = proportional + (Ki * integral);                      // PI
  outputPWM = constrain((int)pidValue, 0, 255);  // Batas output PWM
  dutyCycle = ((double)outputPWM / 255.0) * 100.0;

  // Adjust motor speed through VFD
  analogWrite(pwmMotorPin, outputPWM);

  // Data Log
  if (currentTime - lastTime >= 100) {
    Serial.print(error);
    Serial.print(",");
    Serial.print(pidValue);
    Serial.print(",");
    Serial.print(outputPWM);
    Serial.print(",");
    Serial.print(currentLevel);
    Serial.print(",");
    Serial.println(dutyCycle);

    lastTime = currentTime;
  }

  delay(10);
}

int readCaneLevel() {
  // Read all IR sensors and determine current level
  for (int i = 0; i < 5; i++) {
    proximitySensorValue[i] = digitalRead(proximityLevelPins[i]);
  }

  if (proximitySensorValue[4] == LOW) return 5;       // All sensors covered (Level 5)
  else if (proximitySensorValue[3] == LOW) return 4;  // Sensors 0-3 covered (Level 4)
  else if (proximitySensorValue[2] == LOW) return 3;  // Sensors 0-2 covered (Level 3)
  else if (proximitySensorValue[1] == LOW) return 2;  // Sensors 0-1 covered (Level 2)
  else if (proximitySensorValue[0] == LOW) return 1;  // Only sensor 0 covered (Level 1)
  else return 0;
}

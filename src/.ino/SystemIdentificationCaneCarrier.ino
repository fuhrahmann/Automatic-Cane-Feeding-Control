const byte pwmMotorPin = 5;  // Pin PWM ke inverter AI1
const byte hallSensorPin = 2;
const byte proximityLevelPins[] = { 3, 4, 6, 7, 8 };  // 5 IR sensors for level detection
int proximitySensorValue[5];
float rpm = 0;
unsigned long lastTime;
unsigned long lastPulseTime;

// Variabel untuk pengujian PWM
unsigned long lastTestTime = 0;
unsigned long lastTestTimeSampling = 0;
const unsigned long testInterval = 1000;   // Interval perubahan PWM (ms)
const unsigned long sampleInterval = 100;  // Interval sample (ms)
int pwmValue = 0;
bool increasing = true;
bool testMode = true;  // Set true untuk mode pengujian, false untuk mode normal

void setup() {
  Serial.begin(115200);

  pinMode(hallSensorPin, INPUT_PULLUP);
  for (int i = 0; i < 5; i++) {
    pinMode(proximityLevelPins[i], INPUT);
  }
  pinMode(pwmMotorPin, OUTPUT);

  delay(20000);
  Serial.println("PWM,Level,S1,S2,S3,S4,S5,RPM");
}

void loop() {
  unsigned long currentTime = millis();
  int currentLevel = readCaneLevel();
  if (testMode) {
    // Ramp Input
    // Mode pengujian - generate PWM dari 1-255 dan kembali
    if (currentTime - lastTestTime >= testInterval) {
      lastTestTime = currentTime;
      if (increasing) {
        pwmValue++;
        // Scaling RPM
        if (pwmValue >= 256) {
          increasing = false;
          Serial.println("Pengujian Selesai.");
        }
      } else {
        pwmValue = 0;
      }
      analogWrite(pwmMotorPin, pwmValue);
    }
    if (currentTime - lastTestTimeSampling >= sampleInterval) {
      if (increasing) {
        lastTestTimeSampling = currentTime;
        float rotPm = (pwmValue / 255.0) * 54.31;

        // Cetak data untuk MATLAB (Format: PWM,RPM,Level)
        Serial.print(pwmValue);
        Serial.print(",");
        Serial.print(currentLevel);
        Serial.print(",");
        Serial.print(proximitySensorValue[0]);
        Serial.print(",");
        Serial.print(proximitySensorValue[1]);
        Serial.print(",");
        Serial.print(proximitySensorValue[2]);
        Serial.print(",");
        Serial.print(proximitySensorValue[3]);
        Serial.print(",");
        Serial.print(proximitySensorValue[4]);
        Serial.print(",");
        Serial.println(rotPm);
      }
    }
  }
}

int readCaneLevel() {
  // Read all IR sensors and determine cu   rrent level
  for (int i = 0; i < 5; i++) {
    proximitySensorValue[i] = digitalRead(proximityLevelPins[i]);
  }

  if (proximitySensorValue[4] == LOW) return 5;       // All sensors covered (Level 6)
  else if (proximitySensorValue[3] == LOW) return 4;  // Sensors 0-3 covered (Level 5)
  else if (proximitySensorValue[2] == LOW) return 3;  // Sensors 0-2 covered (Level 4)
  else if (proximitySensorValue[1] == LOW) return 2;  // Sensors 0-1 covered (Level 3)
  else if (proximitySensorValue[0] == LOW) return 1;  // Only sensor 0 covered (Level 2)
  else return 0;
}

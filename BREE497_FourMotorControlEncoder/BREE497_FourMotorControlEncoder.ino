// ================== ENCODER PINS ==================
const int ENCA[4] = {18, 22, 34, 17};  
const int ENCB[4] = {19, 21, 35, 16}; 

// ================== MOTOR CONTROL PINS ==================
const int IN1[4]  = {26, 13, 5, 4};
const int IN2[4]  = {27, 12, 23, 2};
const int EN[4]   = {25, 14, 33, 15};

// ================== PWM SETTINGS ==================
const int freq = 30000;
const int resolution = 8;
int dutyCycle = 0;

// ================== ENCODER VARIABLES ==================
volatile long pos[4] = {0, 0, 0, 0};  // one per motor

// ================== TIMING & STATE MACHINE ==================
unsigned long previousMillis = 0;
unsigned long actionStartTime = 0;
int motorState = 0;
bool accelerating = false;

// ================== INTERRUPT HANDLERS ==================
void IRAM_ATTR readEncoder0() { int b = digitalRead(ENCB[0]); pos[0] += (b > 0) ? 1 : -1; }
void IRAM_ATTR readEncoder1() { int b = digitalRead(ENCB[1]); pos[1] += (b > 0) ? 1 : -1; }
void IRAM_ATTR readEncoder2() { int b = digitalRead(ENCB[2]); pos[2] += (b > 0) ? 1 : -1; }
void IRAM_ATTR readEncoder3() { int b = digitalRead(ENCB[3]); pos[3] += (b > 0) ? 1 : -1; }

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    ledcWrite(EN[i], 0);
  }
  Serial.println("Motors stopped");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Testing 4 Motors with Encoders...");

  // --- Setup motor pins ---
  for (int i = 0; i < 4; i++) {
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    ledcAttach(EN[i], freq, resolution);
  }

  // --- Setup encoder pins and interrupts ---
  pinMode(ENCA[0], INPUT);
  pinMode(ENCB[0], INPUT);
  pinMode(ENCA[1], INPUT);
  pinMode(ENCB[1], INPUT);
  pinMode(ENCA[2], INPUT);
  pinMode(ENCB[2], INPUT);
  pinMode(ENCA[3], INPUT);
  pinMode(ENCB[3], INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[2]), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[3]), readEncoder3, RISING);

  actionStartTime = millis();
}

void loop() {
  // --- Print encoder positions every 200 ms ---
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    noInterrupts();
    Serial.print("Encoders: ");
    for (int i = 0; i < 4; i++) {
      Serial.print("M"); Serial.print(i+1); Serial.print(": ");
      Serial.print(pos[i]); Serial.print("\t");
    }
    interrupts();
    Serial.println();
    lastPrint = millis();
  }

  // --- Motor control timing ---
  unsigned long currentMillis = millis();

  switch (motorState) {
    case 0: // Forward
      if (currentMillis - actionStartTime < 2000) {
        for (int i = 0; i < 4; i++) {
          digitalWrite(IN1[i], LOW);
          digitalWrite(IN2[i], HIGH);
          ledcWrite(EN[i], 255);
        }
      } else {
        stopAllMotors();
        motorState = 1;
        actionStartTime = currentMillis;
      }
      break;

    case 1: // Stop 1
      if (currentMillis - actionStartTime >= 1000) {
        Serial.println("Moving backward");
        for (int i = 0; i < 4; i++) {
          digitalWrite(IN1[i], HIGH);
          digitalWrite(IN2[i], LOW);
          ledcWrite(EN[i], 255);
        }
        motorState = 2;
        actionStartTime = currentMillis;
      }
      break;

    case 2: // Backward
      if (currentMillis - actionStartTime >= 2000) {
        stopAllMotors();
        motorState = 3;
        actionStartTime = currentMillis;
      }
      break;

    case 3: // Stop 2
      if (currentMillis - actionStartTime >= 1000) {
        Serial.println("Accelerating forward");
        for (int i = 0; i < 4; i++) {
          digitalWrite(IN1[i], LOW);
          digitalWrite(IN2[i], HIGH);
        }
        dutyCycle = 0;
        accelerating = true;
        motorState = 4;
        actionStartTime = currentMillis;
      }
      break;

    case 4: // Accelerate
      if (accelerating && (currentMillis - previousMillis >= 200)) {
        previousMillis = currentMillis;
        if (dutyCycle <= 255) {
          for (int i = 0; i < 4; i++) ledcWrite(EN[i], dutyCycle);
          Serial.print("Speed: "); Serial.println(dutyCycle);
          dutyCycle += 10;
        } else {
          accelerating = false;
          Serial.println("Acceleration complete");
          motorState = 0;
          actionStartTime = currentMillis;
        }
      }
      break;
  }
}



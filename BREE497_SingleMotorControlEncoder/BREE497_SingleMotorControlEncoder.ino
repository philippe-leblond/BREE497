int ENCA = 14;  // YELLOW wire
int ENCB = 12;  // Green wire

// Motor A pins
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 25; 

// PWM properties
const int freq = 30000;
const int resolution = 8;
int dutyCycle = 0;

volatile int posi = 0;  // must be volatile

unsigned long previousMillis = 0;
unsigned long actionStartTime = 0;
int motorState = 0; // 0=fwd, 1=stop1, 2=backward, 3=stop2, 4=accelerate
bool accelerating = false;

void IRAM_ATTR readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  // New PWM setup for ESP32 Core 3.3.x
  ledcAttach(enable1Pin, freq, resolution);

  Serial.println("Testing DC Motor and Encoder...");

  actionStartTime = millis(); // start first action immediately
}

void loop() {
  // --- Encoder reading ---
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    noInterrupts();
    int pos = posi;
    interrupts();
    Serial.print("Encoder pos: ");
    Serial.println(pos);
    lastPrint = millis();
  }

   // --- Motor control state machine ---
  unsigned long currentMillis = millis();

  switch (motorState) {
    case 0: // Moving forward
      if (currentMillis - actionStartTime < 2000) {
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
        ledcWrite(enable1Pin, 255);
      } else {
        Serial.println("Motor stopped");
        ledcWrite(enable1Pin, 0);
        motorState = 1;
        actionStartTime = currentMillis;
      }
      break;

    case 1: // First stop
      if (currentMillis - actionStartTime >= 1000) {
        Serial.println("Moving backward");
        digitalWrite(motor1Pin1, HIGH);
        digitalWrite(motor1Pin2, LOW);
        ledcWrite(enable1Pin, 255);
        motorState = 2;
        actionStartTime = currentMillis;
      }
      break;

    case 2: // Moving backward
      if (currentMillis - actionStartTime >= 2000) {
        Serial.println("Motor stopped");
        ledcWrite(enable1Pin, 0);
        motorState = 3;
        actionStartTime = currentMillis;
      }
      break;

    case 3: // Second stop
      if (currentMillis - actionStartTime >= 1000) {
        Serial.println("Accelerating forward");
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
        dutyCycle = 0;
        accelerating = true;
        motorState = 4;
        actionStartTime = currentMillis;
      }
      break;

    case 4: // Accelerate forward
      if (accelerating && (currentMillis - previousMillis >= 200)) {
        previousMillis = currentMillis;
        if (dutyCycle <= 255) {
          ledcWrite(enable1Pin, dutyCycle);
          Serial.print("Speed: ");
          Serial.println(dutyCycle);
          dutyCycle += 10;
        } else {
          accelerating = false;
          Serial.println("Acceleration complete");
          motorState = 0; // loop back to forward
          actionStartTime = currentMillis;
        }
      }
      break;
  }

}


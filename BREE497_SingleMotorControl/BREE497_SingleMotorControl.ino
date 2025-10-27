// Motor A pins
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 25; 

// PWM properties
const int freq = 30000;
const int resolution = 8;
int dutyCycle = 200;

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  // New PWM setup for ESP32 Core 3.3.x
  ledcAttach(enable1Pin, freq, resolution);

  Serial.begin(115200);
  Serial.println("Testing DC Motor...");
}

void loop() {
  // Move forward
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  ledcWrite(enable1Pin, 255);  // Full speed
  delay(2000);

  // Stop
  Serial.println("Motor stopped");
  ledcWrite(enable1Pin, 0);
  delay(1000);

  // Move backward
  Serial.println("Moving Backward");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  ledcWrite(enable1Pin, 255);
  delay(2000);

  // Stop
  Serial.println("Motor stopped");
  ledcWrite(enable1Pin, 0);
  delay(1000);

  // Accelerate forward
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  for (dutyCycle = 0; dutyCycle <= 255; dutyCycle += 10) {
    ledcWrite(enable1Pin, dutyCycle);
    Serial.print("Speed: ");
    Serial.println(dutyCycle);
    delay(200);
  }
}
#include <Arduino.h>

// === MOTOR PINS ===
// Front Left
#define IN1_FL  26
#define IN2_FL  27
#define EN_FL   25

// Front Right
#define IN1_FR  13
#define IN2_FR  12
#define EN_FR   14

// Rear Left
#define IN1_RL  5
#define IN2_RL  23
#define EN_RL   33

// Rear Right
#define IN1_RR  4
#define IN2_RR  2
#define EN_RR   15

// === ENCODER PINS ===
#define ENC_FL_A 18
#define ENC_FL_B 19
#define ENC_FR_A 22
#define ENC_FR_B 21
#define ENC_RL_A 34
#define ENC_RL_B 35
#define ENC_RR_A 17
#define ENC_RR_B 16

// === PWM CONFIG ===
const int freq = 1000;
const int resolution = 8;
int pwmChannelFL = 0;
int pwmChannelFR = 1;
int pwmChannelRL = 2;
int pwmChannelRR = 3;

// === ENCODER COUNTERS ===
volatile long encoderFL = 0;
volatile long encoderFR = 0;
volatile long encoderRL = 0;
volatile long encoderRR = 0;

// === MOTOR SPEED ===
int speedValue = 255; // 0–255

// === TIMER VARIABLES ===
unsigned long lastEncoderSend = 0;
const unsigned long encoderInterval = 100; // ms

// === ENCODER INTERRUPTS ===
void IRAM_ATTR readEncoderFL() {
  int b = digitalRead(ENC_FL_B);
  encoderFL += (b > 0) ? 1 : -1;
}

void IRAM_ATTR readEncoderFR() {
  int b = digitalRead(ENC_FR_B);
  encoderFR += (b > 0) ? 1 : -1;
}

void IRAM_ATTR readEncoderRL() {
  int b = digitalRead(ENC_RL_B);
  encoderRL += (b > 0) ? 1 : -1;
}

void IRAM_ATTR readEncoderRR() {
  int b = digitalRead(ENC_RR_B);
  encoderRR += (b > 0) ? 1 : -1;
}

// === MOTOR CONTROL ===
void setMotor(int in1, int in2, int channel, bool forward) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  ledcWrite(channel, speedValue);
}

// === MOVEMENT FUNCTIONS ===
void stopAll() {
  ledcWrite(EN_FL, 0);
  ledcWrite(EN_FR, 0);
  ledcWrite(EN_RL, 0);
  ledcWrite(EN_RR, 0);
  Serial.println("Motors stopped");
}

void moveForward() {
  setMotor(IN1_FL, IN2_FL, EN_FL, true);
  setMotor(IN1_FR, IN2_FR, EN_FR, true);
  setMotor(IN1_RL, IN2_RL, EN_RL, true);
  setMotor(IN1_RR, IN2_RR, EN_RR, true);
  Serial.println("Forward");
}

void moveBackward() {
  setMotor(IN1_FL, IN2_FL, EN_FL, false);
  setMotor(IN1_FR, IN2_FR, EN_FR, false);
  setMotor(IN1_RL, IN2_RL, EN_RL, false);
  setMotor(IN1_RR, IN2_RR, EN_RR, false);
  Serial.println("Backward");
}

void moveLeft() {
  setMotor(IN1_FL, IN2_FL, EN_FL, false);
  setMotor(IN1_FR, IN2_FR, EN_FR, true);
  setMotor(IN1_RL, IN2_RL, EN_RL, true);
  setMotor(IN1_RR, IN2_RR, EN_RR, false);
  Serial.println("Left");
}

void moveRight() {
  setMotor(IN1_FL, IN2_FL, EN_FL, true);
  setMotor(IN1_FR, IN2_FR, EN_FR, false);
  setMotor(IN1_RL, IN2_RL, EN_RL, false);
  setMotor(IN1_RR, IN2_RR, EN_RR, true);
  Serial.println("Right");
}

// === SETUP ===
void setup() {
  Serial.begin(115200);

  pinMode(IN1_FL, OUTPUT);
  pinMode(IN2_FL, OUTPUT);
  pinMode(IN1_FR, OUTPUT);
  pinMode(IN2_FR, OUTPUT);
  pinMode(IN1_RL, OUTPUT);
  pinMode(IN2_RL, OUTPUT);
  pinMode(IN1_RR, OUTPUT);
  pinMode(IN2_RR, OUTPUT);

  // ledcSetup(pwmChannelFL, freq, resolution);
  // ledcSetup(pwmChannelFR, freq, resolution);
  // ledcSetup(pwmChannelRL, freq, resolution);
  // ledcSetup(pwmChannelRR, freq, resolution);

  // ledcAttachPin(EN_FL, pwmChannelFL);
  // ledcAttachPin(EN_FR, pwmChannelFR);
  // ledcAttachPin(EN_RL, pwmChannelRL);
  // ledcAttachPin(EN_RR, pwmChannelRR);

  ledcAttach(EN_FL, freq, resolution); 
  ledcAttach(EN_FR, freq, resolution); 
  ledcAttach(EN_RL, freq, resolution); 
  ledcAttach(EN_RR, freq, resolution);

  pinMode(ENC_FL_A, INPUT_PULLUP);
  pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_RL_A, INPUT_PULLUP);
  pinMode(ENC_RL_B, INPUT_PULLUP);
  pinMode(ENC_RR_A, INPUT_PULLUP);
  pinMode(ENC_RR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), readEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), readEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), readEncoderRL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), readEncoderRR, RISING);

  stopAll();
}

// === LOOP ===
void loop() {
  // Handle serial commands instantly
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "forward") moveForward();
    else if (cmd == "backward") moveBackward();
    else if (cmd == "left") moveLeft();
    else if (cmd == "right") moveRight();
    else if (cmd == "stop") stopAll();
  }

  // Send encoder data every 100 ms without delay()
  unsigned long currentMillis = millis();
  if (currentMillis - lastEncoderSend >= encoderInterval) {
    lastEncoderSend = currentMillis;

  Serial.print("<ENC_FL:"); Serial.print(encoderFL);
  Serial.print(",ENC_FR:"); Serial.print(encoderFR);
  Serial.print(",ENC_RL:"); Serial.print(encoderRL);
  Serial.print(",ENC_RR:"); Serial.print(encoderRR);
  Serial.println(">");

  }
}

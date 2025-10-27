// #define ENCA 25 // Yellow wire
// #define ENCB 26 // Green wire

// void setup() {
//   Serial.begin(115200);
//   pinMode(ENCA, INPUT);
//   pinMode(ENCB, INPUT);
// }

// void loop() {
//   int a = digitalRead(ENCA);
//   int b = digitalRead(ENCB);
//   Serial.print(a * 5);
//   Serial.print(" ");
//   Serial.print(b * 5);
//   Serial.println();
//   delay(100); // Optional: slows down output for readability
// }


#define ENCA 25  // YELLOW wire
#define ENCB 26  // Green wire

volatile int posi = 0;  // must be volatile

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
}

void loop() {
  int pos;
  noInterrupts();  // disable interrupts temporarily
  pos = posi;
  interrupts();    // re-enable
  Serial.println(pos);
  delay(100);      // optional: slow down serial output
}

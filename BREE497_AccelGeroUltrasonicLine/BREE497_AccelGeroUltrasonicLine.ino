#include <DFRobot_BMI160.h>

#define NUM_ULTRASONICS 2 //4
#define NUM_LINE_SENSORS 3
#define NUM_READINGS 10
#define UPDATE_INTERVAL 100  // ms

// --- Ultrasonic sensor pins ---
const int trigPins[NUM_ULTRASONICS] = {25, 32};
const int echoPins[NUM_ULTRASONICS] = {26, 33};

// const int trigPins[NUM_ULTRASONICS] = {34, 32, 26, 27};
// const int echoPins[NUM_ULTRASONICS] = {35, 33, 25, 13};

// --- Line sensor pins ---
const int linePins[NUM_LINE_SENSORS] = {5, 19, 18};

// --- Data storage ---
float ultrasonicReadings[NUM_ULTRASONICS][NUM_READINGS];
int lineReadings[NUM_LINE_SENSORS][NUM_READINGS];

// --- BMI160 sensor setup ---
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;
float accelXArr[NUM_READINGS], accelYArr[NUM_READINGS], accelZArr[NUM_READINGS];
float gyroXArr[NUM_READINGS], gyroYArr[NUM_READINGS], gyroZArr[NUM_READINGS];

// --- Timing ---
unsigned long lastUpdate = 0;

// ---------- HELPER FUNCTIONS ----------

// Get median of array
template <typename T>
T getMedian(T *arr, int size) {
  T temp[size];
  memcpy(temp, arr, size * sizeof(T));
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (temp[j] < temp[i]) {
        T t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }
  return temp[size / 2];
}

// Ultrasonic distance in cm
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout
  float distance = duration * 0.0343 / 2;
  if (distance == 0 || distance > 400) distance = 400; // cap outliers
  return distance;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Setup ultrasonics
  for (int i = 0; i < NUM_ULTRASONICS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // Setup line sensors
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    pinMode(linePins[i], INPUT);
  }

  // Setup BMI160
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("BMI160 reset failed!");
    while (1);
  }

  if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
    Serial.println("BMI160 init failed!");
    while (1);
  }

  Serial.println("All sensors initialized!");
  Serial.println("=========================");
}

// ---------- MAIN LOOP ----------
void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentMillis;

    // --- Ultrasonic readings ---
    for (int i = 0; i < NUM_ULTRASONICS; i++) {
      for (int j = 0; j < NUM_READINGS; j++) {
        ultrasonicReadings[i][j] = readUltrasonic(trigPins[i], echoPins[i]);
        delay(3);
      }
    }

    // --- Line sensors ---
    for (int i = 0; i < NUM_LINE_SENSORS; i++) {
      for (int j = 0; j < NUM_READINGS; j++) {
        lineReadings[i][j] = digitalRead(linePins[i]);
        delay(1);
      }
    }

    // --- BMI160 accel + gyro readings ---
    for (int i = 0; i < NUM_READINGS; i++) {
      int16_t data[6] = {0};
      if (bmi160.getAccelGyroData(data) == 0) {
        // Gyroscope raw data
        gyroXArr[i] = data[0] * 3.14 / 180.0;
        gyroYArr[i] = data[1] * 3.14 / 180.0;
        gyroZArr[i] = data[2] * 3.14 / 180.0;

        // Accelerometer raw data
        accelXArr[i] = data[3] / 16384.0;
        accelYArr[i] = data[4] / 16384.0;
        accelZArr[i] = data[5] / 16384.0;
      }
      delay(5);
    }

    // --- Compute medians ---
    float ultrasonicMedians[NUM_ULTRASONICS];
    int lineMedians[NUM_LINE_SENSORS];

    for (int i = 0; i < NUM_ULTRASONICS; i++)
      ultrasonicMedians[i] = getMedian(ultrasonicReadings[i], NUM_READINGS);

    for (int i = 0; i < NUM_LINE_SENSORS; i++)
      lineMedians[i] = getMedian(lineReadings[i], NUM_READINGS);

    float accelX = getMedian(accelXArr, NUM_READINGS);
    float accelY = getMedian(accelYArr, NUM_READINGS);
    float accelZ = getMedian(accelZArr, NUM_READINGS);
    float gyroX = getMedian(gyroXArr, NUM_READINGS);
    float gyroY = getMedian(gyroYArr, NUM_READINGS);
    float gyroZ = getMedian(gyroZArr, NUM_READINGS);

    // --- Print results ---
    Serial.print("<");
    Serial.print("U1:"); Serial.print(ultrasonicMedians[0]);
    Serial.print(",U2:"); Serial.print(ultrasonicMedians[1]);
    Serial.print(",L1:"); Serial.print(lineMedians[0]);
    Serial.print(",L2:"); Serial.print(lineMedians[1]);
    Serial.print(",L3:"); Serial.print(lineMedians[2]);
    Serial.print(",GX:"); Serial.print(gyroX);
    Serial.print(",GY:"); Serial.print(gyroY);
    Serial.print(",GZ:"); Serial.print(gyroZ);
    Serial.print(",AX:"); Serial.print(accelX);
    Serial.print(",AY:"); Serial.print(accelY);
    Serial.print(",AZ:"); Serial.print(accelZ);
    Serial.println(">");
  }
}

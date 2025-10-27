/*
  ESP32 - 4 Ultrasonic Sensors + 3 Digital Line Sensors
  Reads each sensor 10 times, computes the median, and prints results every 100 ms.
  Designed for ROS2 serial bridge or debugging via Serial Monitor.
*/

#define NUM_ULTRASONICS 4
#define NUM_LINE_SENSORS 3
#define NUM_READINGS 10
#define UPDATE_INTERVAL 100  // milliseconds

// --- Ultrasonic sensor pins ---
const int trigPins[NUM_ULTRASONICS] = {34, 32, 26, 27};
const int echoPins[NUM_ULTRASONICS] = {35, 33, 25, 13};

// --- Line sensor pins ---
const int linePins[NUM_LINE_SENSORS] = {21, 19, 18};

// --- Storage arrays ---
float ultrasonicReadings[NUM_ULTRASONICS][NUM_READINGS];
int lineReadings[NUM_LINE_SENSORS][NUM_READINGS];

unsigned long lastUpdate = 0;

// Helper function to get median from an array
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

// Read distance in cm from ultrasonic
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
  float distance = duration * 0.0343 / 2;         // convert to cm
  if (distance == 0 || distance > 400) distance = 400;  // cap outliers
  return distance;
}

void setup() {
  Serial.begin(115200);

  // Ultrasonic setup
  for (int i = 0; i < NUM_ULTRASONICS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // Line sensors setup
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    pinMode(linePins[i], INPUT);
  }

  Serial.println("Starting sensor readings...");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentMillis;

    // --- Ultrasonic readings ---
    for (int i = 0; i < NUM_ULTRASONICS; i++) {
      for (int j = 0; j < NUM_READINGS; j++) {
        ultrasonicReadings[i][j] = readUltrasonic(trigPins[i], echoPins[i]);
        delay(5);  // small delay to stabilize reading
      }
    }

    // --- Line sensor readings ---
    for (int i = 0; i < NUM_LINE_SENSORS; i++) {
      for (int j = 0; j < NUM_READINGS; j++) {
        lineReadings[i][j] = digitalRead(linePins[i]);
        delay(2);
      }
    }

    // --- Compute medians ---
    float ultrasonicMedians[NUM_ULTRASONICS];
    int lineMedians[NUM_LINE_SENSORS];

    for (int i = 0; i < NUM_ULTRASONICS; i++) {
      ultrasonicMedians[i] = getMedian(ultrasonicReadings[i], NUM_READINGS);
    }

    for (int i = 0; i < NUM_LINE_SENSORS; i++) {
      lineMedians[i] = getMedian(lineReadings[i], NUM_READINGS);
    }

    // --- Print results ---
    Serial.println("=== Sensor Data ===");
    for (int i = 0; i < NUM_ULTRASONICS; i++) {
      Serial.print("Ultrasonic ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(ultrasonicMedians[i]);
      Serial.println(" cm");
    }

    for (int i = 0; i < NUM_LINE_SENSORS; i++) {
      Serial.print("Line ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(lineMedians[i] == HIGH ? "HIGH" : "LOW");
    }

    Serial.println("====================");
  }
}

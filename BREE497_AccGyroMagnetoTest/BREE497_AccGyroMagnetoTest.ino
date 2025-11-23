#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

unsigned long lastPrint = 0;
const unsigned long printInterval = 20; // 50Hz print

// Watchdog for IMU stall detection
unsigned long lastIMUEvent = 0;
const unsigned long imuTimeout = 300;   // ms → reset after 300ms of no data

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

void setup() {
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.begin(115200);
  delay(50);

  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("IMU NOT FOUND");
    while(1);
  }

  Serial.println("IMU OK");

  bno08x.enableReport(SH2_ACCELEROMETER, 25);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 25);
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 25);

  lastIMUEvent = millis();
}

// Reinitialize IMU after freeze
void resetIMU() {
  Serial.println("⚠️ IMU stalled -> resetting...");

  // Soft reset
  // bno08x.sh2.reset();
  // delay(100);

  // Re-init I2C
  bno08x.begin_I2C(0x4B);
  delay(50);

  // Re-enable reports
  bno08x.enableReport(SH2_ACCELEROMETER, 25);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 25);
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 25);

  Serial.println("✅ IMU reinitialized.");
}

void loop() {

  bool eventReceived = false;

  // Always drain FIFO
  while (bno08x.getSensorEvent(&sensorValue)) {
    eventReceived = true;
    lastIMUEvent = millis();  // watchdog heartbeat

    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        ax = sensorValue.un.accelerometer.x;
        ay = sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        gx = sensorValue.un.gyroscope.x;
        gy = sensorValue.un.gyroscope.y;
        gz = sensorValue.un.gyroscope.z;
        break;

      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mx = sensorValue.un.magneticField.x;
        my = sensorValue.un.magneticField.y;
        mz = sensorValue.un.magneticField.z;
        break;
    }
  }

  // 🔥 Watchdog: IMU is frozen?
  if (millis() - lastIMUEvent > imuTimeout) {
    resetIMU();
    lastIMUEvent = millis();
  }

  // Printing
  if (millis() - lastPrint >= printInterval) {
    lastPrint = millis();
    Serial.printf("<AX:%.3f,AY:%.3f,AZ:%.3f,"
                  "GX:%.3f,GY:%.3f,GZ:%.3f,"
                  "MX:%.3f,MY:%.3f,MZ:%.3f>\n",
                  ax, ay, az, gx, gy, gz, mx, my, mz);
  }
}

#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // Initialize the hardware BMI160  
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("BMI160 reset failed!");
    while (1);
  }
  
  // Set and initialize the BMI160 I2C address
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
    Serial.println("BMI160 init failed!");
    while (1);
  }

  Serial.println("BMI160 initialized successfully!");
  Serial.println("Reading accelerometer (m/s^2) and gyroscope (°/s) data...");
  Serial.println("-------------------------------------------------------");
}

void loop() {
  int16_t accelGyro[6] = {0}; 
  int rslt = bmi160.getAccelGyroData(accelGyro);

  if (rslt == 0) {
    // Gyroscope data (first three values)
    float gyroX = accelGyro[0] * 3.14 / 180.0;
    float gyroY = accelGyro[1] * 3.14 / 180.0;
    float gyroZ = accelGyro[2] * 3.14 / 180.0;

    // Accelerometer data (last three values)
    float accelX = accelGyro[3] / 16384.0;
    float accelY = accelGyro[4] / 16384.0;
    float accelZ = accelGyro[5] / 16384.0;

    // Print formatted output
    Serial.println("------ SENSOR DATA ------");
    Serial.print("Gyro X (°/s): "); Serial.println(gyroX);
    Serial.print("Gyro Y (°/s): "); Serial.println(gyroY);
    Serial.print("Gyro Z (°/s): "); Serial.println(gyroZ);
    Serial.print("Accel X (g): "); Serial.println(accelX);
    Serial.print("Accel Y (g): "); Serial.println(accelY);
    Serial.print("Accel Z (g): "); Serial.println(accelZ);
    Serial.println("--------------------------\n");
  } 
  else {
    Serial.println("Error reading BMI160 data!");
  }

  delay(100);
}

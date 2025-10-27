/*
 * MPU6050Arduino.c
 *
 * Arduino sketch to read raw MPU6050 IMU data (accelerometer and gyroscope)
 * and transmit it over serial port.
 *
 * Hardware Requirements:
 * - Arduino board (Uno, Nano, Mega, etc.)
 * - MPU6050 IMU module
 *
 * Connections:
 * - MPU6050 VCC  -> Arduino 5V (or 3.3V)
 * - MPU6050 GND  -> Arduino GND
 * - MPU6050 SCL  -> Arduino A5 (SCL)
 * - MPU6050 SDA  -> Arduino A4 (SDA)
 *
 * Dependencies:
 * - Wire library (built-in)
 *
 * Serial Protocol:
 * - Baud Rate: 115200
 * - Format: "AX:value,AY:value,AZ:value,GX:value,GY:value,GZ:value,TEMP:value\n"
 *
 * Debug Mode:
 * - Uncomment "#define DEBUG_MODE" to enable verbose debug output
 * - Debug mode prints human-readable sensor values with physical units
 * - Useful for testing and verification on Arduino Serial Monitor
 * - Comment out "#define DEBUG_MODE" for normal operation with C++ reader
 */

#include <Wire.h>

// Debug mode - uncomment to enable verbose debug output
// #define DEBUG_MODE

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 Register addresses
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

// Raw sensor data
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;

// Timing for data transmission
unsigned long last_time = 0;
const unsigned long sample_interval = 10; // 10ms = 100Hz

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB)
  }

  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400kHz (fast mode)

  // Initialize MPU6050
  if (initMPU6050()) {
    Serial.println("INFO:MPU6050 initialized successfully");
  } else {
    Serial.println("ERROR:Failed to initialize MPU6050");
    while (1) {
      delay(1000);
    }
  }

  delay(100);
}

void loop() {
  unsigned long current_time = millis();

  // Read sensor data at specified interval
  if (current_time - last_time >= sample_interval) {
    last_time = current_time;

    // Read raw sensor data
    readMPU6050Data();

    // Print debug info if enabled
    debugPrint();

    // Transmit data over serial
    transmitData();
  }
}

bool initMPU6050() {
  // Check if MPU6050 is connected
  Wire.beginTransmission(MPU6050_ADDR);
  byte error = Wire.endTransmission();

  if (error != 0) {
    return false;
  }

  // Wake up MPU6050 (disable sleep mode)
  writeRegister(MPU6050_PWR_MGMT_1, 0x00);
  delay(10);

  // Set sample rate divider (1kHz / (1 + SMPLRT_DIV))
  // For 100Hz: 1000 / (1 + 9) = 100Hz
  writeRegister(MPU6050_SMPLRT_DIV, 0x09);

  // Configure DLPF (Digital Low Pass Filter)
  // DLPF_CFG = 3: Accel BW = 44Hz, Gyro BW = 42Hz
  writeRegister(MPU6050_CONFIG, 0x03);

  // Configure gyroscope range
  // FS_SEL = 0: +/- 250 degrees/sec
  writeRegister(MPU6050_GYRO_CONFIG, 0x00);

  // Configure accelerometer range
  // AFS_SEL = 0: +/- 2g
  writeRegister(MPU6050_ACCEL_CONFIG, 0x00);

  delay(10);
  return true;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)1);
  return Wire.read();
}

void readMPU6050Data() {
  // Request 14 bytes from MPU6050 starting at ACCEL_XOUT_H register
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

  // Read accelerometer data (6 bytes)
  accel_x = (Wire.read() << 8) | Wire.read();
  accel_y = (Wire.read() << 8) | Wire.read();
  accel_z = (Wire.read() << 8) | Wire.read();

  // Read temperature data (2 bytes)
  temperature = (Wire.read() << 8) | Wire.read();

  // Read gyroscope data (6 bytes)
  gyro_x = (Wire.read() << 8) | Wire.read();
  gyro_y = (Wire.read() << 8) | Wire.read();
  gyro_z = (Wire.read() << 8) | Wire.read();
}

void debugPrint() {
#ifdef DEBUG_MODE
  // Print human-readable debug information
  Serial.println("=====================================");
  Serial.println("MPU6050 Debug Output");
  Serial.println("=====================================");

  // Print raw values
  Serial.println("Raw Values:");
  Serial.print("  Accel X: "); Serial.println(accel_x);
  Serial.print("  Accel Y: "); Serial.println(accel_y);
  Serial.print("  Accel Z: "); Serial.println(accel_z);
  Serial.print("  Gyro X:  "); Serial.println(gyro_x);
  Serial.print("  Gyro Y:  "); Serial.println(gyro_y);
  Serial.print("  Gyro Z:  "); Serial.println(gyro_z);
  Serial.print("  Temp:    "); Serial.println(temperature);

  // Convert and print physical values
  Serial.println("\nPhysical Values:");
  Serial.print("  Accel X: "); Serial.print(accel_x / 16384.0); Serial.println(" g");
  Serial.print("  Accel Y: "); Serial.print(accel_y / 16384.0); Serial.println(" g");
  Serial.print("  Accel Z: "); Serial.print(accel_z / 16384.0); Serial.println(" g");
  Serial.print("  Gyro X:  "); Serial.print(gyro_x / 131.0); Serial.println(" deg/s");
  Serial.print("  Gyro Y:  "); Serial.print(gyro_y / 131.0); Serial.println(" deg/s");
  Serial.print("  Gyro Z:  "); Serial.print(gyro_z / 131.0); Serial.println(" deg/s");
  Serial.print("  Temp:    "); Serial.print((temperature / 340.0) + 36.53); Serial.println(" C");

  // Print timestamp
  Serial.print("\nUptime: "); Serial.print(millis()); Serial.println(" ms");
  Serial.println("=====================================\n");
#endif
}

void transmitData() {
  // Format: AX:value,AY:value,AZ:value,GX:value,GY:value,GZ:value,TEMP:value
  Serial.print("AX:");
  Serial.print(accel_x);
  Serial.print(",AY:");
  Serial.print(accel_y);
  Serial.print(",AZ:");
  Serial.print(accel_z);
  Serial.print(",GX:");
  Serial.print(gyro_x);
  Serial.print(",GY:");
  Serial.print(gyro_y);
  Serial.print(",GZ:");
  Serial.print(gyro_z);
  Serial.print(",TEMP:");
  Serial.println(temperature);
}

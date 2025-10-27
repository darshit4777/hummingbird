# MPU6050 IMU Reader

Interface for reading MPU6050 IMU sensor data via Arduino over USB serial connection.

## Quick Start

### 1. Upload Arduino Firmware
```bash
# Open MPU6050Arduino.c in Arduino IDE and upload to your board
```

**Optional - Enable Debug Mode:**
To test the MPU6050 sensor directly on Arduino Serial Monitor:
1. Open MPU6050Arduino.c
2. Uncomment the line: `// #define DEBUG_MODE` → `#define DEBUG_MODE`
3. Upload to Arduino
4. Open Serial Monitor (115200 baud)
5. You'll see detailed sensor output with physical units

**Remember:** Comment out `#define DEBUG_MODE` before using with the C++ reader!

### 2. Connect Hardware
- MPU6050 VCC → Arduino 5V
- MPU6050 GND → Arduino GND
- MPU6050 SCL → Arduino A5
- MPU6050 SDA → Arduino A4
- Arduino → Computer via USB

### 3. Run the Reader
```bash
# Build and run with default port
./run_reader.sh

# Or specify port
./run_reader.sh /dev/ttyUSB0
```

## Usage Examples

```bash
# Standard run (builds first)
./run_reader.sh

# Skip rebuild for faster startup
./run_reader.sh --skip-build

# Debug build with symbols
./run_reader.sh --debug

# Optimized release build
./run_reader.sh --release

# Use different port
./run_reader.sh /dev/ttyUSB0

# Show all options
./run_reader.sh --help
```

## Files

- **MPU6050Arduino.c** - Arduino firmware to read sensor and transmit over serial
- **Mpu6050Reader.cpp** - C++ application to receive and parse sensor data
- **run_reader.sh** - Convenience script to build and run
- **BUILD** - Bazel build configuration

## Data Format

Serial output format (115200 baud, 100Hz):
```
AX:value,AY:value,AZ:value,GX:value,GY:value,GZ:value,TEMP:value
```

All values are 16-bit signed integers (raw sensor readings).

## Conversion to Physical Units

- **Acceleration (g):** `raw_value / 16384.0`
- **Gyroscope (°/s):** `raw_value / 131.0`
- **Temperature (°C):** `(raw_value / 340.0) + 36.53`

## Debug Mode Output

When DEBUG_MODE is enabled, you'll see output like this on Arduino Serial Monitor:

```
=====================================
MPU6050 Debug Output
=====================================
Raw Values:
  Accel X: -1024
  Accel Y: 512
  Accel Z: 16384
  Gyro X:  -50
  Gyro Y:  25
  Gyro Z:  0
  Temp:    8400

Physical Values:
  Accel X: -0.06 g
  Accel Y: 0.03 g
  Accel Z: 1.00 g
  Gyro X:  -0.38 deg/s
  Gyro Y:  0.19 deg/s
  Gyro Z:  0.00 deg/s
  Temp:    61.23 C

Uptime: 1234 ms
=====================================
```

This helps verify the sensor is working correctly before using it with the C++ reader.

## Troubleshooting

### Testing Sensor on Arduino
1. Enable DEBUG_MODE in MPU6050Arduino.c
2. Upload to Arduino
3. Open Serial Monitor (115200 baud)
4. Verify you see debug output with reasonable values:
   - Accel Z should be ~1.0g when flat on table
   - Gyro values should be near 0 when stationary
   - Temperature should be room temperature (20-30°C)

### Port Access Denied
```bash
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### Port Not Found
```bash
# List available ports
ls -la /dev/tty{USB,ACM}*
```

### Arduino Not Responding
1. Check USB connection
2. Verify sketch is uploaded
3. Try resetting Arduino
4. Check baud rate matches (115200)

## Dependencies

- **Build:** Bazel (Bazelisk recommended)
- **Runtime:** POSIX-compliant OS (Linux/macOS)
- **Hardware:** Arduino with MPU6050 connected via I2C

No external C++ libraries required!

## See Also

- [current_progress.md](../current_progress.md) - Detailed project documentation
- [MPU6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)

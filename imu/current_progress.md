# IMU Development Progress

## Project Overview
This directory contains IMU (Inertial Measurement Unit) integration code for the Hummingbird quadcopter project. The focus is on interfacing with various IMU sensors for flight control and stabilization.

## Current Status
**Last Updated:** October 26, 2025
**Build Status:** ✅ Successfully building with Bazel

### MPU6050 Integration

#### Completed Tasks
1. **Arduino Interface (MPU6050Arduino.c)**
   - Created Arduino sketch to read raw MPU6050 sensor data
   - Implemented I2C communication at 400kHz
   - Configured sensor ranges:
     - Accelerometer: ±2g
     - Gyroscope: ±250°/s
   - Sample rate: 100Hz (configurable)
   - Serial communication: 115200 baud
   - Data protocol: CSV-style format `AX:value,AY:value,AZ:value,GX:value,GY:value,GZ:value,TEMP:value`
   - Debug mode feature:
     - Enable/disable with `#define DEBUG_MODE` toggle
     - Prints human-readable values with physical units
     - Shows raw values, converted values (g, deg/s, °C), and uptime
     - Useful for Arduino Serial Monitor testing

2. **C++ Reader Application (Mpu6050Reader.cpp)**
   - Created C++ program to interface with Arduino over USB serial
   - Implemented `MPU6050Reader` class with the following features:
     - POSIX serial communication using termios (no external dependencies)
     - Data parsing and validation
     - Timestamping of received data
     - Conversion functions for physical units (g, deg/s, °C)
     - Error handling and connection management
   - Structured data format using `MPU6050Data` struct

3. **Build System Setup**
   - ✅ Successfully building with Bazel
   - Migrated from manual g++ compilation to Bazel build system
   - Created BUILD file for mpu6050 package
   - Simplified WORKSPACE (no external dependencies needed)
   - Created .bazelrc for project-wide build settings
   - Renamed files to CamelCase convention (Mpu6050Reader.cpp)
   - Installed Bazelisk v1.19.0 for automatic Bazel version management
   - Binary size: 33KB (stripped)

4. **Convenience Scripts and Documentation**
   - Created run_reader.sh for easy building and running
   - Features:
     - Automatic building before running
     - Support for different serial ports
     - Debug and release build configurations
     - Skip-build option for faster testing
     - Colored output and helpful error messages
     - Port existence and permission checking
   - Created README.md in mpu6050 directory for quick reference

5. **Python IMU Visualizer** ✨ NEW
   - Created full-featured visualization application
   - Real-time 3D orientation display using quaternions
   - Live graphs for acceleration and gyroscope data
   - AHRS filter integration (Madgwick and Mahony)
   - Configurable filter parameters
   - Components:
     - `SerialReader.py`: Arduino serial communication
     - `AhrsFilter.py`: AHRS filter wrapper with smoothing
     - `OrientationWidget.py`: 3D OpenGL visualization
     - `GraphWidget.py`: Real-time PyQtGraph plotting
     - `ImuVisualizer.py`: Main GUI application
   - Features:
     - Switchable filters (Madgwick/Mahony)
     - Adjustable filter gain
     - Sample rate monitoring
     - Port selection GUI
     - Reset and clear functions
     - Automatic dependency management

#### File Structure
```
imu/
├── current_progress.md           # This document
├── mpu6050/                      # MPU6050 reader package
│   ├── BUILD                     # Bazel build configuration
│   ├── MPU6050Arduino.c          # Arduino firmware
│   ├── Mpu6050Reader.cpp         # C++ reader application
│   ├── run_reader.sh             # Convenience script to build and run
│   └── README.md                 # Quick reference guide
└── imu_visualizer/               # Python visualization application
    ├── ImuVisualizer.py          # Main GUI application
    ├── SerialReader.py           # Serial communication module
    ├── AhrsFilter.py             # AHRS filter wrapper
    ├── OrientationWidget.py      # 3D orientation widget
    ├── GraphWidget.py            # Real-time graph widgets
    ├── requirements.txt          # Python dependencies
    ├── run_visualizer.sh         # Run script
    └── README.md                 # Visualizer documentation
```

#### Hardware Connections
**MPU6050 to Arduino:**
- VCC  → 5V (or 3.3V)
- GND  → GND
- SCL  → A5 (Arduino I2C Clock)
- SDA  → A4 (Arduino I2C Data)

**Arduino to Computer:**
- USB connection (typically /dev/ttyACM0 or /dev/ttyUSB0 on Linux)

#### Building and Running

**Quick Start (Recommended):**
```bash
# Navigate to mpu6050 directory
cd imu/mpu6050

# Build and run with default port (/dev/ttyACM0)
./run_reader.sh

# Use a different port
./run_reader.sh /dev/ttyUSB0

# Skip rebuild, just run
./run_reader.sh --skip-build

# Debug build
./run_reader.sh --debug

# Release build
./run_reader.sh --release

# Show help
./run_reader.sh --help
```

**Manual Build with Bazel:**
```bash
# From project root
bazel build //imu/mpu6050:Mpu6050Reader

# Debug build
bazel build --config=debug //imu/mpu6050:Mpu6050Reader

# Release build
bazel build --config=release //imu/mpu6050:Mpu6050Reader
```

**Manual Run:**
```bash
# Run directly with Bazel
bazel run //imu/mpu6050:Mpu6050Reader -- /dev/ttyACM0

# Or run the compiled binary
./bazel-bin/imu/mpu6050/Mpu6050Reader /dev/ttyACM0
```

**Upload Arduino sketch:**
1. Open MPU6050Arduino.c in Arduino IDE
2. Select your Arduino board and port
3. Upload the sketch

**Run IMU Visualizer:**
```bash
# Navigate to visualizer directory
cd imu/imu_visualizer

# Run (automatically installs dependencies)
./run_visualizer.sh
```

The visualizer will:
1. Create a Python virtual environment (first run only)
2. Install all required packages (first run only)
3. Launch the GUI application

In the GUI:
1. Select your serial port from the dropdown
2. Click "Connect"
3. Adjust filter type (Madgwick/Mahony) and gain as needed
4. Watch the 3D orientation and real-time graphs!

#### Dependencies

**Arduino:**
- Wire library (built-in)

**C++ Reader:**
- C++11 compiler (standard library only)
- POSIX-compliant OS (Linux/macOS)
- Bazel build system (Bazelisk recommended)

**Python Visualizer:**
- Python 3.7+
- PySerial (serial communication)
- PyQt5 (GUI framework)
- PyQtGraph (real-time plotting)
- PyOpenGL (3D graphics)
- AHRS (orientation filters)
- NumPy (numerical computations)

All Python dependencies are automatically installed by `run_visualizer.sh`.

**Install Bazelisk (recommended):**
```bash
wget https://github.com/bazelbuild/bazelisk/releases/download/v1.19.0/bazelisk-linux-amd64
chmod +x bazelisk-linux-amd64
sudo mv bazelisk-linux-amd64 /usr/local/bin/bazel
# Or install to user directory: mv bazelisk-linux-amd64 ~/.local/bin/bazel
```

**No external libraries required!** The C++ reader uses only standard POSIX APIs.

## Next Steps

### Immediate Tasks
- [ ] Test the MPU6050 setup with physical hardware
- [ ] Validate data transmission and parsing
- [ ] Calibrate IMU sensor offsets
- [ ] Implement sensor fusion algorithms (complementary/Kalman filter)

### Future Enhancements
- [ ] Add ROS integration for publishing IMU data
- [ ] Implement madgwick or mahony filter for orientation estimation
- [ ] Add support for other IMU sensors (MPU9250, BNO055, etc.)
- [ ] Create unit tests for data parsing and conversion functions
- [ ] Add data logging capabilities
- [ ] Implement real-time visualization of IMU data
- [ ] Add configuration file support for runtime parameters

### Integration with Flight Controller
- [ ] Interface IMU data with existing xfour_controller package
- [ ] Update flight_control.cpp to use IMU measurements
- [ ] Implement attitude estimation for quadcopter stabilization
- [ ] Add IMU data to debug publisher (if applicable)

## Technical Notes

### Data Format
The serial protocol uses a simple CSV-style format for easy parsing:
```
AX:<accel_x>,AY:<accel_y>,AZ:<accel_z>,GX:<gyro_x>,GY:<gyro_y>,GZ:<gyro_z>,TEMP:<temp>
```

All values are 16-bit signed integers representing raw sensor readings.

### Conversion Formulas
- **Acceleration (g):** `value / 16384.0` (for ±2g range)
- **Gyroscope (deg/s):** `value / 131.0` (for ±250°/s range)
- **Temperature (°C):** `(value / 340.0) + 36.53`

### Known Limitations
1. Current implementation uses blocking I/O for serial communication
2. No error recovery mechanism if Arduino resets
3. No configuration for different MPU6050 settings from C++ side
4. Timestamping is done on PC side (not synchronized with Arduino)

### Build History and Issues Resolved

#### October 26, 2025 - Initial Bazel Build Setup
**Issues Encountered:**
1. **Bazel not installed** - Resolved by installing Bazelisk v1.19.0 to ~/.local/bin
2. **WORKSPACE disabled in Bazel 8+** - Fixed by adding `common --enable_workspace` to .bazelrc
3. **Boost rules_boost SHA256 mismatch** - Initial checksum was incorrect
4. **Boost library glob errors** - rules_boost had issues with Boost locale sources

**Solution:**
- Rewrote Mpu6050Reader.cpp to use POSIX termios APIs instead of Boost.Asio
- Removed all external dependencies from BUILD file
- Simplified WORKSPACE to minimal configuration
- Result: Clean build with no external dependencies, 33KB binary

**Build Command:**
```bash
bazel build //imu/mpu6050:Mpu6050Reader
```

**Build Output:**
```
INFO: Analyzed target //imu/mpu6050:Mpu6050Reader (110 packages loaded, 675 targets configured).
INFO: Found 1 target...
Target //imu/mpu6050:Mpu6050Reader up-to-date:
  bazel-bin/imu/mpu6050/Mpu6050Reader
INFO: Build completed successfully, 6 total actions
```

## References
- MPU6050 Datasheet: [InvenSense MPU-6000/MPU-6050 Product Specification](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- I2C Protocol: Arduino Wire Library Documentation
- POSIX Serial Programming: [Linux Serial Programming Guide](https://tldp.org/HOWTO/Serial-Programming-HOWTO/)
- Bazel Build System: [Bazel C++ Tutorial](https://bazel.build/tutorials/cpp)
- Bazelisk: [Bazelisk Repository](https://github.com/bazelbuild/bazelisk)

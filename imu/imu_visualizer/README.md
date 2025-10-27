# IMU Visualizer

Real-time 3D visualization and data plotting for MPU6050 IMU sensor.

## Features

- **3D Orientation Display**: Real-time 3D visualization of IMU orientation
- **Real-time Graphs**: Live plots for acceleration (X, Y, Z) and angular velocity (X, Y, Z)
- **AHRS Filters**: Madgwick and Mahony filters for smooth orientation estimation
- **Configurable**: Adjustable filter gains and types
- **Serial Interface**: Direct connection to Arduino running MPU6050 firmware

## Quick Start

### 1. Install Dependencies

```bash
./run_visualizer.sh
```

The script will automatically:
- Create a Python virtual environment
- Install all required dependencies
- Launch the visualizer

### 2. Connect Hardware

1. Upload `MPU6050Arduino.c` to your Arduino
2. Connect Arduino to computer via USB
3. Note the serial port (e.g., /dev/ttyACM0)

### 3. Run Visualizer

```bash
# Simple run - select port in GUI
./run_visualizer.sh

# Or specify port directly (not implemented yet in GUI)
./run_visualizer.sh /dev/ttyACM0
```

## Manual Installation

If you prefer manual setup:

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run visualizer
python3 ImuVisualizer.py
```

## Usage

### Control Panel

- **Port Selection**: Choose serial port from dropdown or enter custom port
- **Connect/Disconnect**: Connect to Arduino
- **Filter Selection**: Choose between Madgwick and Mahony filters
- **Gain**: Adjust filter gain (lower = smoother, higher = more responsive)
- **Reset Filter**: Reset orientation estimation
- **Clear Graphs**: Clear all plotted data

### Visualization

**3D Orientation (Left)**:
- Color-coded 3D box showing IMU orientation
- X-axis: Red
- Y-axis: Green
- Z-axis: Blue
- Euler angles displayed below (Roll, Pitch, Yaw)

**Graphs (Right)**:
- Top: Linear Acceleration (g units)
- Bottom: Angular Velocity (°/s units)
- Real-time plotting with automatic scaling
- Mean values displayed below each graph

## Filter Configuration

### Madgwick Filter
- Default gain: 0.033
- Recommended range: 0.01 - 0.1
- Lower gain = smoother but slower response
- Higher gain = faster but more noise

### Mahony Filter
- Default gain (kp): 0.033
- Recommended range: 0.01 - 1.0
- Similar characteristics to Madgwick
- Computationally more efficient

## Architecture

```
imu_visualizer/
├── ImuVisualizer.py        # Main GUI application
├── SerialReader.py         # Serial communication with Arduino
├── AhrsFilter.py           # AHRS filter wrapper (Madgwick/Mahony)
├── OrientationWidget.py    # 3D orientation visualization
├── GraphWidget.py          # Real-time plotting widgets
├── requirements.txt        # Python dependencies
├── run_visualizer.sh       # Convenience run script
└── README.md               # This file
```

## Dependencies

- **PySerial**: Serial communication with Arduino
- **PyQt5**: GUI framework
- **PyQtGraph**: Real-time plotting
- **PyOpenGL**: 3D graphics
- **AHRS**: Orientation filters (Madgwick, Mahony)
- **NumPy**: Numerical computations

## Troubleshooting

### Permission Denied on Serial Port

```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

### Virtual Environment Issues

```bash
# Remove and recreate
rm -rf venv
./run_visualizer.sh
```

### Import Errors

```bash
# Reinstall dependencies
source venv/bin/activate
pip install --upgrade -r requirements.txt
```

### Poor Orientation Tracking

1. Try adjusting filter gain (start with 0.033)
2. Switch between Madgwick and Mahony filters
3. Ensure MPU6050 is properly calibrated
4. Check for magnetic interference if using magnetometer

## Tips

- **Calibration**: Let the IMU sit still for a few seconds after connecting
- **Smooth Motion**: Make slow, smooth movements for best results
- **Filter Tuning**: Experiment with gain values based on your application
- **Data Rate**: Monitor the sample rate display (should be ~100 Hz)

## See Also

- [../mpu6050/README.md](../mpu6050/README.md) - MPU6050 Arduino firmware
- [../current_progress.md](../current_progress.md) - Project progress and documentation

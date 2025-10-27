"""
SerialReader.py

Module to read MPU6050 data from Arduino over serial connection.
Parses incoming data and provides structured IMU measurements.
"""

import serial
import time
import threading
from typing import Optional, Callable
from dataclasses import dataclass
import queue


@dataclass
class ImuData:
    """Structure to hold IMU sensor data"""
    accel_x: float  # Raw accelerometer values
    accel_y: float
    accel_z: float
    gyro_x: float   # Raw gyroscope values
    gyro_y: float
    gyro_z: float
    temperature: float
    timestamp: float  # Timestamp in seconds


class SerialReader:
    """
    Reads MPU6050 data from Arduino over serial port.

    Expected format: AX:value,AY:value,AZ:value,GX:value,GY:value,GZ:value,TEMP:value
    """

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        """
        Initialize serial reader.

        Args:
            port: Serial port path
            baudrate: Communication baud rate
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_port: Optional[serial.Serial] = None
        self.is_running = False
        self.read_thread: Optional[threading.Thread] = None
        self.data_queue = queue.Queue(maxsize=100)
        self.error_callback: Optional[Callable] = None

    def connect(self) -> bool:
        """
        Connect to Arduino over serial port.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )

            # Wait for Arduino to reset
            time.sleep(2.0)

            # Flush initial data
            self.serial_port.reset_input_buffer()

            print(f"Connected to Arduino on {self.port} at {self.baudrate} baud")
            return True

        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            if self.error_callback:
                self.error_callback(f"Serial connection error: {e}")
            return False

    def disconnect(self):
        """Disconnect from serial port"""
        self.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Disconnected from Arduino")

    def start(self):
        """Start reading data in background thread"""
        if not self.serial_port or not self.serial_port.is_open:
            print("Error: Serial port not connected")
            return

        self.is_running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        print("Started reading IMU data")

    def stop(self):
        """Stop reading data"""
        self.is_running = False
        if self.read_thread:
            self.read_thread.join(timeout=2.0)
        print("Stopped reading IMU data")

    def get_data(self, timeout: float = 0.1) -> Optional[ImuData]:
        """
        Get latest IMU data from queue.

        Args:
            timeout: Maximum time to wait for data

        Returns:
            ImuData object or None if no data available
        """
        try:
            return self.data_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def set_error_callback(self, callback: Callable):
        """Set callback function for error reporting"""
        self.error_callback = callback

    def _read_loop(self):
        """Background thread to continuously read serial data"""
        while self.is_running:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8').strip()

                    # Skip INFO and ERROR messages
                    if line.startswith("INFO:") or line.startswith("ERROR:"):
                        print(line)
                        continue

                    # Parse data line
                    data = self._parse_line(line)
                    if data:
                        # Add to queue, drop oldest if full
                        if self.data_queue.full():
                            try:
                                self.data_queue.get_nowait()
                            except queue.Empty:
                                pass
                        self.data_queue.put(data)

            except serial.SerialException as e:
                print(f"Serial read error: {e}")
                if self.error_callback:
                    self.error_callback(f"Serial read error: {e}")
                self.is_running = False
                break
            except Exception as e:
                print(f"Unexpected error: {e}")
                continue

    def _parse_line(self, line: str) -> Optional[ImuData]:
        """
        Parse a data line from Arduino.

        Format: AX:value,AY:value,AZ:value,GX:value,GY:value,GZ:value,TEMP:value

        Args:
            line: Raw serial line

        Returns:
            ImuData object or None if parsing fails
        """
        try:
            tokens = line.split(',')
            if len(tokens) != 7:
                return None

            data = {}
            for token in tokens:
                key, value = token.split(':')
                data[key] = int(value)

            return ImuData(
                accel_x=float(data['AX']),
                accel_y=float(data['AY']),
                accel_z=float(data['AZ']),
                gyro_x=float(data['GX']),
                gyro_y=float(data['GY']),
                gyro_z=float(data['GZ']),
                temperature=float(data['TEMP']),
                timestamp=time.time()
            )

        except (ValueError, KeyError) as e:
            print(f"Parse error: {e} - Line: {line}")
            return None


if __name__ == "__main__":
    # Test the serial reader
    reader = SerialReader("/dev/ttyACM0")

    if reader.connect():
        reader.start()

        print("Reading IMU data... (Press Ctrl+C to stop)")
        try:
            while True:
                data = reader.get_data(timeout=1.0)
                if data:
                    print(f"Accel: ({data.accel_x:6.0f}, {data.accel_y:6.0f}, {data.accel_z:6.0f})  "
                          f"Gyro: ({data.gyro_x:6.0f}, {data.gyro_y:6.0f}, {data.gyro_z:6.0f})  "
                          f"Temp: {data.temperature:5.0f}")
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            reader.disconnect()

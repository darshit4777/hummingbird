"""
AhrsFilter.py

Wrapper for AHRS filters (Madgwick and Mahony) to estimate IMU orientation.
Converts raw accelerometer and gyroscope data to quaternion orientation.
"""

import numpy as np
from ahrs.filters import Madgwick, Mahony
from typing import Tuple
from enum import Enum


class FilterType(Enum):
    """Available AHRS filter types"""
    MADGWICK = "madgwick"
    MAHONY = "mahony"


class AhrsFilter:
    """
    AHRS filter wrapper for IMU orientation estimation.

    Converts raw accelerometer and gyroscope data to smooth orientation (quaternion).
    Supports Madgwick and Mahony filters with configurable parameters.
    """

    # Sensor scaling factors for MPU6050
    ACCEL_SCALE = 16384.0  # LSB/g for ±2g range
    GYRO_SCALE = 131.0     # LSB/(deg/s) for ±250°/s range

    def __init__(self, filter_type: FilterType = FilterType.MADGWICK,
                 sample_rate: float = 100.0, gain: float = 0.033):
        """
        Initialize AHRS filter.

        Args:
            filter_type: Type of filter to use (Madgwick or Mahony)
            sample_rate: Sensor sample rate in Hz
            gain: Filter gain parameter (beta for Madgwick, kp for Mahony)
        """
        self.filter_type = filter_type
        self.sample_rate = sample_rate
        self.gain = gain

        # Initialize filter
        self._create_filter()

        # Quaternion state [w, x, y, z]
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        # Smoothed acceleration and gyroscope values (physical units)
        self.accel_smoothed = np.array([0.0, 0.0, 1.0])  # g
        self.gyro_smoothed = np.array([0.0, 0.0, 0.0])   # rad/s

        # Exponential moving average alpha
        self.ema_alpha = 0.2

    def _create_filter(self):
        """Create the appropriate AHRS filter instance"""
        if self.filter_type == FilterType.MADGWICK:
            self.filter = Madgwick(
                frequency=self.sample_rate,
                gain_imu=self.gain
            )
        elif self.filter_type == FilterType.MAHONY:
            self.filter = Mahony(
                frequency=self.sample_rate,
                k_P=self.gain,
                k_I=0.0
            )
        else:
            raise ValueError(f"Unknown filter type: {self.filter_type}")

    def switch_filter(self, filter_type: FilterType, gain: float = None):
        """
        Switch to a different filter type.

        Args:
            filter_type: New filter type
            gain: New gain value (optional, keeps current if None)
        """
        self.filter_type = filter_type
        if gain is not None:
            self.gain = gain
        self._create_filter()
        print(f"Switched to {filter_type.value} filter with gain={self.gain}")

    def update(self, accel_raw: Tuple[float, float, float],
               gyro_raw: Tuple[float, float, float]) -> np.ndarray:
        """
        Update filter with new sensor data.

        Args:
            accel_raw: Raw accelerometer values (AX, AY, AZ)
            gyro_raw: Raw gyroscope values (GX, GY, GZ)

        Returns:
            Updated quaternion [w, x, y, z]
        """
        # Convert to physical units
        accel_g = np.array([
            accel_raw[0] / self.ACCEL_SCALE,
            accel_raw[1] / self.ACCEL_SCALE,
            accel_raw[2] / self.ACCEL_SCALE
        ])

        gyro_rad_s = np.array([
            np.deg2rad(gyro_raw[0] / self.GYRO_SCALE),
            np.deg2rad(gyro_raw[1] / self.GYRO_SCALE),
            np.deg2rad(gyro_raw[2] / self.GYRO_SCALE)
        ])

        # Apply exponential moving average for smoothing
        self.accel_smoothed = (self.ema_alpha * accel_g +
                               (1 - self.ema_alpha) * self.accel_smoothed)
        self.gyro_smoothed = (self.ema_alpha * gyro_rad_s +
                              (1 - self.ema_alpha) * self.gyro_smoothed)

        # Update AHRS filter
        self.quaternion = self.filter.updateIMU(
            q=self.quaternion,
            gyr=self.gyro_smoothed,
            acc=self.accel_smoothed
        )

        return self.quaternion

    def get_euler_angles(self) -> Tuple[float, float, float]:
        """
        Convert current quaternion to Euler angles.

        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        return self._quaternion_to_euler(self.quaternion)

    def get_accel_g(self) -> np.ndarray:
        """Get smoothed acceleration in g"""
        return self.accel_smoothed.copy()

    def get_gyro_deg_s(self) -> np.ndarray:
        """Get smoothed gyroscope in deg/s"""
        return np.rad2deg(self.gyro_smoothed)

    def reset(self):
        """Reset filter to initial state"""
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.accel_smoothed = np.array([0.0, 0.0, 1.0])
        self.gyro_smoothed = np.array([0.0, 0.0, 0.0])
        self._create_filter()
        print("AHRS filter reset")

    @staticmethod
    def _quaternion_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).

        Args:
            q: Quaternion [w, x, y, z]

        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        w, x, y, z = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return (np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw))


if __name__ == "__main__":
    # Test the AHRS filter
    ahrs = AhrsFilter(FilterType.MADGWICK)

    # Simulate some sensor data (stationary, upright)
    accel_raw = (0, 0, 16384)  # 1g in Z direction
    gyro_raw = (0, 0, 0)        # No rotation

    for i in range(100):
        quat = ahrs.update(accel_raw, gyro_raw)
        roll, pitch, yaw = ahrs.get_euler_angles()

        if i % 20 == 0:
            print(f"Iter {i:3d}: Quaternion: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]")
            print(f"          Roll: {roll:7.2f}°  Pitch: {pitch:7.2f}°  Yaw: {yaw:7.2f}°")

    # Test filter switching
    ahrs.switch_filter(FilterType.MAHONY, gain=0.5)
    quat = ahrs.update(accel_raw, gyro_raw)
    print(f"\nAfter switch to Mahony: {ahrs.get_euler_angles()}")

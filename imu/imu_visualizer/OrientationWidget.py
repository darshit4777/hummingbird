"""
OrientationWidget.py

3D visualization widget for IMU orientation using PyQt and PyQtGraph.
Displays a 3D box representing the IMU orientation in real-time.
"""

import numpy as np
from PyQt5 import QtWidgets
import pyqtgraph.opengl as gl
from typing import Tuple


class OrientationWidget(QtWidgets.QWidget):
    """
    Widget to display 3D IMU orientation.

    Shows a 3D box that rotates according to the IMU quaternion orientation.
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.layout)

        # Create 3D view widget
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setMinimumSize(400, 400)
        self.layout.addWidget(self.gl_widget)

        # Set up camera
        self.gl_widget.setCameraPosition(distance=8, elevation=20, azimuth=45)

        # Create grid
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.gl_widget.addItem(grid)

        # Create coordinate axes
        self._create_axes()

        # Create IMU box
        self._create_imu_box()

        # Create label for Euler angles
        self.euler_label = QtWidgets.QLabel("Roll: 0.0°  Pitch: 0.0°  Yaw: 0.0°")
        self.euler_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.layout.addWidget(self.euler_label)

    def _create_axes(self):
        """Create XYZ coordinate axes"""
        # X axis (red)
        x_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [2, 0, 0]]),
            color=(1, 0, 0, 1),
            width=2,
            antialias=True
        )
        self.gl_widget.addItem(x_axis)

        # Y axis (green)
        y_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 2, 0]]),
            color=(0, 1, 0, 1),
            width=2,
            antialias=True
        )
        self.gl_widget.addItem(y_axis)

        # Z axis (blue)
        z_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 0, 2]]),
            color=(0, 0, 1, 1),
            width=2,
            antialias=True
        )
        self.gl_widget.addItem(z_axis)

    def _create_imu_box(self):
        """Create 3D box representing the IMU"""
        # Define box vertices (2x1x0.5 units)
        verts = np.array([
            [-1, -0.5, -0.25],  # 0
            [ 1, -0.5, -0.25],  # 1
            [ 1,  0.5, -0.25],  # 2
            [-1,  0.5, -0.25],  # 3
            [-1, -0.5,  0.25],  # 4
            [ 1, -0.5,  0.25],  # 5
            [ 1,  0.5,  0.25],  # 6
            [-1,  0.5,  0.25],  # 7
        ])

        # Define faces (12 triangles for 6 faces)
        faces = np.array([
            [0, 1, 2], [0, 2, 3],  # Bottom
            [4, 5, 6], [4, 6, 7],  # Top
            [0, 1, 5], [0, 5, 4],  # Front
            [2, 3, 7], [2, 7, 6],  # Back
            [0, 3, 7], [0, 7, 4],  # Left
            [1, 2, 6], [1, 6, 5],  # Right
        ])

        # Define face colors (different color for each face)
        face_colors = np.array([
            [0.8, 0.2, 0.2, 0.8],  # Bottom - Red
            [0.8, 0.2, 0.2, 0.8],
            [0.2, 0.8, 0.2, 0.8],  # Top - Green
            [0.2, 0.8, 0.2, 0.8],
            [0.2, 0.2, 0.8, 0.8],  # Front - Blue
            [0.2, 0.2, 0.8, 0.8],
            [0.8, 0.8, 0.2, 0.8],  # Back - Yellow
            [0.8, 0.8, 0.2, 0.8],
            [0.8, 0.2, 0.8, 0.8],  # Left - Magenta
            [0.8, 0.2, 0.8, 0.8],
            [0.2, 0.8, 0.8, 0.8],  # Right - Cyan
            [0.2, 0.8, 0.8, 0.8],
        ])

        # Create mesh item
        self.imu_mesh = gl.GLMeshItem(
            vertexes=verts,
            faces=faces,
            faceColors=face_colors,
            smooth=False,
            drawEdges=True,
            edgeColor=(0, 0, 0, 1)
        )

        self.gl_widget.addItem(self.imu_mesh)

    def update_orientation(self, quaternion: np.ndarray):
        """
        Update the 3D box orientation based on quaternion.

        Args:
            quaternion: Orientation quaternion [w, x, y, z]
        """
        # Convert quaternion to rotation matrix
        rotation_matrix = self._quaternion_to_rotation_matrix(quaternion)

        # Apply rotation to mesh
        self.imu_mesh.resetTransform()
        self.imu_mesh.setTransform(rotation_matrix)

    def update_euler_display(self, roll: float, pitch: float, yaw: float):
        """
        Update the Euler angles display label.

        Args:
            roll, pitch, yaw: Euler angles in degrees
        """
        self.euler_label.setText(
            f"Roll: {roll:7.2f}°  Pitch: {pitch:7.2f}°  Yaw: {yaw:7.2f}°"
        )

    @staticmethod
    def _quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """
        Convert quaternion to 4x4 rotation matrix for OpenGL.

        Args:
            q: Quaternion [w, x, y, z]

        Returns:
            4x4 rotation matrix
        """
        w, x, y, z = q

        # Compute rotation matrix elements
        r00 = 1 - 2 * (y**2 + z**2)
        r01 = 2 * (x*y - w*z)
        r02 = 2 * (x*z + w*y)

        r10 = 2 * (x*y + w*z)
        r11 = 1 - 2 * (x**2 + z**2)
        r12 = 2 * (y*z - w*x)

        r20 = 2 * (x*z - w*y)
        r21 = 2 * (y*z + w*x)
        r22 = 1 - 2 * (x**2 + y**2)

        # Create 4x4 matrix (OpenGL format)
        matrix = np.array([
            [r00, r01, r02, 0],
            [r10, r11, r12, 0],
            [r20, r21, r22, 0],
            [0,   0,   0,   1]
        ], dtype=np.float32)

        return matrix


if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication

    app = QApplication(sys.argv)

    # Create and show widget
    widget = OrientationWidget()
    widget.setWindowTitle("IMU Orientation Visualizer")
    widget.resize(600, 600)
    widget.show()

    # Test rotation
    import time
    angle = 0

    def update_test():
        global angle
        angle += 2
        # Create test quaternion (rotation around Z axis)
        qw = np.cos(np.deg2rad(angle) / 2)
        qz = np.sin(np.deg2rad(angle) / 2)
        quat = np.array([qw, 0, 0, qz])
        widget.update_orientation(quat)
        widget.update_euler_display(0, 0, angle % 360)

    from PyQt5.QtCore import QTimer
    timer = QTimer()
    timer.timeout.connect(update_test)
    timer.start(50)  # 20 Hz

    sys.exit(app.exec_())

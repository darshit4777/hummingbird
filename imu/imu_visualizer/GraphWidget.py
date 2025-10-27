"""
GraphWidget.py

Real-time plotting widgets for IMU acceleration and gyroscope data.
Uses PyQtGraph for efficient real-time plotting.
"""

import numpy as np
from collections import deque
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


class RealtimeGraphWidget(QtWidgets.QWidget):
    """
    Widget for real-time plotting of 3-axis sensor data.

    Displays X, Y, Z axes on a single plot with different colors.
    """

    def __init__(self, title: str, ylabel: str, max_points: int = 500, parent=None):
        """
        Initialize graph widget.

        Args:
            title: Graph title
            ylabel: Y-axis label
            max_points: Maximum number of points to display
            parent: Parent widget
        """
        super().__init__(parent)

        self.max_points = max_points

        # Data buffers for X, Y, Z axes
        self.time_data = deque(maxlen=max_points)
        self.x_data = deque(maxlen=max_points)
        self.y_data = deque(maxlen=max_points)
        self.z_data = deque(maxlen=max_points)

        # Start time reference
        self.start_time = None

        # Create layout
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        # Create plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setTitle(title, color='k', size='12pt')
        self.plot_widget.setLabel('left', ylabel, color='k')
        self.plot_widget.setLabel('bottom', 'Time (s)', color='k')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.addLegend()

        layout.addWidget(self.plot_widget)

        # Create plot curves
        self.curve_x = self.plot_widget.plot(
            pen=pg.mkPen(color='r', width=2),
            name='X'
        )
        self.curve_y = self.plot_widget.plot(
            pen=pg.mkPen(color='g', width=2),
            name='Y'
        )
        self.curve_z = self.plot_widget.plot(
            pen=pg.mkPen(color='b', width=2),
            name='Z'
        )

        # Stats label
        self.stats_label = QtWidgets.QLabel("")
        self.stats_label.setStyleSheet("font-size: 10px;")
        layout.addWidget(self.stats_label)

    def add_data_point(self, x: float, y: float, z: float, timestamp: float = None):
        """
        Add a new data point to the graph.

        Args:
            x, y, z: Data values for each axis
            timestamp: Timestamp (uses current time if None)
        """
        if timestamp is None:
            timestamp = QtCore.QTime.currentTime().msecsSinceStartOfDay() / 1000.0

        if self.start_time is None:
            self.start_time = timestamp

        # Relative time in seconds
        rel_time = timestamp - self.start_time

        # Add to buffers
        self.time_data.append(rel_time)
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

    def update_plot(self):
        """Update the plot with current data"""
        if len(self.time_data) == 0:
            return

        time_array = np.array(self.time_data)
        x_array = np.array(self.x_data)
        y_array = np.array(self.y_data)
        z_array = np.array(self.z_data)

        # Update curves
        self.curve_x.setData(time_array, x_array)
        self.curve_y.setData(time_array, y_array)
        self.curve_z.setData(time_array, z_array)

        # Update stats
        if len(x_array) > 0:
            x_mean = np.mean(x_array[-50:])  # Last 50 points
            y_mean = np.mean(y_array[-50:])
            z_mean = np.mean(z_array[-50:])

            self.stats_label.setText(
                f"Mean (last 50): X={x_mean:7.3f}  Y={y_mean:7.3f}  Z={z_mean:7.3f}"
            )

    def clear(self):
        """Clear all data"""
        self.time_data.clear()
        self.x_data.clear()
        self.y_data.clear()
        self.z_data.clear()
        self.start_time = None
        self.update_plot()


class ImuGraphsWidget(QtWidgets.QWidget):
    """
    Container widget for both acceleration and gyroscope graphs.

    Displays two graphs side-by-side or stacked.
    """

    def __init__(self, layout_mode: str = "horizontal", parent=None):
        """
        Initialize IMU graphs widget.

        Args:
            layout_mode: "horizontal" or "vertical" layout
            parent: Parent widget
        """
        super().__init__(parent)

        # Create layout
        if layout_mode == "horizontal":
            self.layout = QtWidgets.QHBoxLayout()
        else:
            self.layout = QtWidgets.QVBoxLayout()

        self.setLayout(self.layout)

        # Create acceleration graph
        self.accel_graph = RealtimeGraphWidget(
            title="Linear Acceleration",
            ylabel="Acceleration (g)"
        )
        self.layout.addWidget(self.accel_graph)

        # Create gyroscope graph
        self.gyro_graph = RealtimeGraphWidget(
            title="Angular Velocity",
            ylabel="Angular Velocity (°/s)"
        )
        self.layout.addWidget(self.gyro_graph)

    def add_accel_data(self, ax: float, ay: float, az: float, timestamp: float = None):
        """Add acceleration data point"""
        self.accel_graph.add_data_point(ax, ay, az, timestamp)

    def add_gyro_data(self, gx: float, gy: float, gz: float, timestamp: float = None):
        """Add gyroscope data point"""
        self.gyro_graph.add_data_point(gx, gy, gz, timestamp)

    def update_plots(self):
        """Update both plots"""
        self.accel_graph.update_plot()
        self.gyro_graph.update_plot()

    def clear_all(self):
        """Clear all graphs"""
        self.accel_graph.clear()
        self.gyro_graph.clear()


if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer
    import time

    app = QApplication(sys.argv)

    # Create widget
    widget = ImuGraphsWidget(layout_mode="vertical")
    widget.setWindowTitle("IMU Graphs Test")
    widget.resize(800, 600)
    widget.show()

    # Test data generation
    t = 0

    def generate_test_data():
        global t
        t += 0.05

        # Generate sine wave test data
        ax = np.sin(t)
        ay = np.cos(t)
        az = 1.0 + 0.1 * np.sin(2 * t)

        gx = 10 * np.sin(0.5 * t)
        gy = 15 * np.cos(0.7 * t)
        gz = 5 * np.sin(0.3 * t)

        widget.add_accel_data(ax, ay, az)
        widget.add_gyro_data(gx, gy, gz)
        widget.update_plots()

    # Update at 20 Hz
    timer = QTimer()
    timer.timeout.connect(generate_test_data)
    timer.start(50)

    sys.exit(app.exec_())

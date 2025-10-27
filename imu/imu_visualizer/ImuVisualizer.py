"""
ImuVisualizer.py

Main GUI application for IMU visualization.
Combines 3D orientation display with real-time sensor data graphs.
"""

import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
from SerialReader import SerialReader, ImuData
from AhrsFilter import AhrsFilter, FilterType
from OrientationWidget import OrientationWidget
from GraphWidget import ImuGraphsWidget


class ImuVisualizer(QtWidgets.QMainWindow):
    """
    Main IMU Visualizer application.

    Features:
    - 3D orientation display
    - Real-time acceleration and gyroscope graphs
    - Filter selection (Madgwick/Mahony)
    - Serial port configuration
    - Start/stop controls
    """

    def __init__(self):
        super().__init__()

        self.serial_reader = None
        self.ahrs_filter = None
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update_visualization)

        self.init_ui()
        self.init_filter()

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("IMU Visualizer - MPU6050")
        self.setGeometry(100, 100, 1400, 900)

        # Central widget
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QtWidgets.QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Top: Control panel
        main_layout.addWidget(self.create_control_panel())

        # Middle: Content area (3D view and graphs)
        content_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        # Left: 3D Orientation
        self.orientation_widget = OrientationWidget()
        content_splitter.addWidget(self.orientation_widget)

        # Right: Graphs
        graphs_container = QtWidgets.QWidget()
        graphs_layout = QtWidgets.QVBoxLayout()
        graphs_container.setLayout(graphs_layout)

        self.graphs_widget = ImuGraphsWidget(layout_mode="vertical")
        graphs_layout.addWidget(self.graphs_widget)

        content_splitter.addWidget(graphs_container)
        content_splitter.setSizes([600, 800])

        main_layout.addWidget(content_splitter)

        # Bottom: Status bar
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.status_label.setStyleSheet(
            "padding: 5px; background-color: #ffcccc; font-weight: bold;"
        )
        main_layout.addWidget(self.status_label)

    def create_control_panel(self):
        """Create the control panel widget"""
        panel = QtWidgets.QGroupBox("Controls")
        layout = QtWidgets.QHBoxLayout()
        panel.setLayout(layout)

        # Serial port selection
        layout.addWidget(QtWidgets.QLabel("Port:"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.addItems([
            "/dev/ttyACM0",
            "/dev/ttyACM1",
            "/dev/ttyUSB0",
            "/dev/ttyUSB1"
        ])
        self.port_combo.setEditable(True)
        layout.addWidget(self.port_combo)

        # Connect/Disconnect button
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_button)

        layout.addWidget(QtWidgets.QLabel("|"))

        # Filter selection
        layout.addWidget(QtWidgets.QLabel("Filter:"))
        self.filter_combo = QtWidgets.QComboBox()
        self.filter_combo.addItems(["Madgwick", "Mahony"])
        self.filter_combo.currentTextChanged.connect(self.on_filter_changed)
        layout.addWidget(self.filter_combo)

        # Filter gain
        layout.addWidget(QtWidgets.QLabel("Gain:"))
        self.gain_spinbox = QtWidgets.QDoubleSpinBox()
        self.gain_spinbox.setRange(0.001, 2.0)
        self.gain_spinbox.setSingleStep(0.01)
        self.gain_spinbox.setValue(0.033)
        self.gain_spinbox.setDecimals(3)
        self.gain_spinbox.valueChanged.connect(self.on_gain_changed)
        layout.addWidget(self.gain_spinbox)

        layout.addWidget(QtWidgets.QLabel("|"))

        # Reset button
        reset_button = QtWidgets.QPushButton("Reset Filter")
        reset_button.clicked.connect(self.reset_filter)
        layout.addWidget(reset_button)

        # Clear graphs button
        clear_button = QtWidgets.QPushButton("Clear Graphs")
        clear_button.clicked.connect(self.clear_graphs)
        layout.addWidget(clear_button)

        layout.addStretch()

        # Sample rate display
        self.rate_label = QtWidgets.QLabel("Rate: 0 Hz")
        layout.addWidget(self.rate_label)

        return panel

    def init_filter(self):
        """Initialize the AHRS filter"""
        self.ahrs_filter = AhrsFilter(
            filter_type=FilterType.MADGWICK,
            sample_rate=100.0,
            gain=self.gain_spinbox.value()
        )

    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if self.serial_reader is None:
            self.connect_serial()
        else:
            self.disconnect_serial()

    def connect_serial(self):
        """Connect to Arduino"""
        port = self.port_combo.currentText()

        self.serial_reader = SerialReader(port=port, baudrate=115200)
        self.serial_reader.set_error_callback(self.on_serial_error)

        if self.serial_reader.connect():
            self.serial_reader.start()
            self.update_timer.start(20)  # Update UI at ~50 Hz

            self.connect_button.setText("Disconnect")
            self.port_combo.setEnabled(False)
            self.status_label.setText(f"Connected to {port}")
            self.status_label.setStyleSheet(
                "padding: 5px; background-color: #ccffcc; font-weight: bold;"
            )

            # Sample rate tracking
            self.sample_count = 0
            self.last_rate_update = QtCore.QTime.currentTime()

        else:
            self.serial_reader = None
            QtWidgets.QMessageBox.critical(
                self,
                "Connection Error",
                f"Failed to connect to {port}\n\n"
                "Please check:\n"
                "1. Arduino is connected\n"
                "2. Correct port is selected\n"
                "3. You have permission to access the port"
            )

    def disconnect_serial(self):
        """Disconnect from Arduino"""
        if self.serial_reader:
            self.update_timer.stop()
            self.serial_reader.disconnect()
            self.serial_reader = None

        self.connect_button.setText("Connect")
        self.port_combo.setEnabled(True)
        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet(
            "padding: 5px; background-color: #ffcccc; font-weight: bold;"
        )

    def update_visualization(self):
        """Update visualization with new IMU data"""
        if not self.serial_reader:
            return

        # Process all available data
        updated = False
        while True:
            data = self.serial_reader.get_data(timeout=0.001)
            if data is None:
                break

            # Update AHRS filter
            accel_raw = (data.accel_x, data.accel_y, data.accel_z)
            gyro_raw = (data.gyro_x, data.gyro_y, data.gyro_z)

            quaternion = self.ahrs_filter.update(accel_raw, gyro_raw)

            # Update 3D orientation
            self.orientation_widget.update_orientation(quaternion)

            # Update Euler angles display
            roll, pitch, yaw = self.ahrs_filter.get_euler_angles()
            self.orientation_widget.update_euler_display(roll, pitch, yaw)

            # Update graphs with smoothed data
            accel_g = self.ahrs_filter.get_accel_g()
            gyro_deg_s = self.ahrs_filter.get_gyro_deg_s()

            self.graphs_widget.add_accel_data(
                accel_g[0], accel_g[1], accel_g[2],
                timestamp=data.timestamp
            )
            self.graphs_widget.add_gyro_data(
                gyro_deg_s[0], gyro_deg_s[1], gyro_deg_s[2],
                timestamp=data.timestamp
            )

            # Track sample rate
            self.sample_count += 1
            updated = True

        # Update graphs if we received data
        if updated:
            self.graphs_widget.update_plots()

        # Update sample rate display (every second)
        current_time = QtCore.QTime.currentTime()
        elapsed = self.last_rate_update.msecsTo(current_time)
        if elapsed >= 1000:
            rate = self.sample_count * 1000.0 / elapsed
            self.rate_label.setText(f"Rate: {rate:.1f} Hz")
            self.sample_count = 0
            self.last_rate_update = current_time

    def on_filter_changed(self, filter_name: str):
        """Handle filter type change"""
        if filter_name == "Madgwick":
            filter_type = FilterType.MADGWICK
        else:
            filter_type = FilterType.MAHONY

        if self.ahrs_filter:
            self.ahrs_filter.switch_filter(filter_type, self.gain_spinbox.value())

    def on_gain_changed(self, value: float):
        """Handle filter gain change"""
        if self.ahrs_filter:
            filter_type = (FilterType.MADGWICK if self.filter_combo.currentText() == "Madgwick"
                          else FilterType.MAHONY)
            self.ahrs_filter.switch_filter(filter_type, value)

    def reset_filter(self):
        """Reset the AHRS filter"""
        if self.ahrs_filter:
            self.ahrs_filter.reset()

    def clear_graphs(self):
        """Clear all graph data"""
        self.graphs_widget.clear_all()

    def on_serial_error(self, error_msg: str):
        """Handle serial errors"""
        QtWidgets.QMessageBox.warning(self, "Serial Error", error_msg)
        self.disconnect_serial()

    def closeEvent(self, event):
        """Handle window close event"""
        if self.serial_reader:
            self.disconnect_serial()
        event.accept()


def main():
    """Main entry point"""
    app = QtWidgets.QApplication(sys.argv)

    # Set application style
    app.setStyle('Fusion')

    # Create and show main window
    visualizer = ImuVisualizer()
    visualizer.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

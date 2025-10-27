/*
 * Mpu6050Reader.cpp
 *
 * C++ program to read MPU6050 raw data from Arduino over USB serial connection.
 * This program connects to the Arduino running MPU6050Arduino.c and parses
 * the incoming sensor data.
 *
 * Dependencies:
 * - Standard C++ library (no external dependencies)
 * - POSIX serial port APIs (termios)
 *
 * Build with Bazel:
 * bazel build //imu/mpu6050:Mpu6050Reader
 *
 * Usage:
 * ./Mpu6050Reader /dev/ttyACM0
 */

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cstdint>
#include <chrono>
#include <thread>
#include <stdexcept>

// POSIX serial port headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

struct MPU6050Data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
    uint64_t timestamp_ms;
};

class MPU6050Reader {
public:
    MPU6050Reader(const std::string& port_name, unsigned int baud_rate = 115200)
        : port_name_(port_name),
          baud_rate_(baud_rate),
          serial_fd_(-1),
          is_connected_(false) {
    }

    ~MPU6050Reader() {
        disconnect();
    }

    bool connect() {
        // Open serial port
        serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            std::cerr << "Error opening serial port " << port_name_ << ": "
                      << strerror(errno) << std::endl;
            return false;
        }

        // Configure serial port
        struct termios tty;
        memset(&tty, 0, sizeof(tty));

        if (tcgetattr(serial_fd_, &tty) != 0) {
            std::cerr << "Error getting serial port attributes: "
                      << strerror(errno) << std::endl;
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // Set baud rate
        speed_t speed = B115200;
        switch (baud_rate_) {
            case 9600:   speed = B9600;   break;
            case 19200:  speed = B19200;  break;
            case 38400:  speed = B38400;  break;
            case 57600:  speed = B57600;  break;
            case 115200: speed = B115200; break;
            default:
                std::cerr << "Unsupported baud rate: " << baud_rate_ << std::endl;
                close(serial_fd_);
                serial_fd_ = -1;
                return false;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // 8N1 mode (8 data bits, no parity, 1 stop bit)
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        tty.c_cflag |= (CLOCAL | CREAD);                // Enable receiver, ignore modem controls
        tty.c_cflag &= ~(PARENB | PARODD);              // No parity
        tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;                        // No hardware flow control

        // Input flags
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

        // Output flags - disable output processing
        tty.c_oflag = 0;

        // Local flags - disable canonical mode, echo, etc.
        tty.c_lflag = 0;

        // Control characters
        tty.c_cc[VMIN]  = 1;  // Read blocks until at least 1 character is received
        tty.c_cc[VTIME] = 5;  // 0.5 seconds read timeout

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting serial port attributes: "
                      << strerror(errno) << std::endl;
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        is_connected_ = true;
        std::cout << "Connected to Arduino on " << port_name_
                  << " at " << baud_rate_ << " baud" << std::endl;

        // Wait for Arduino to reset after serial connection
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Flush any initial data
        tcflush(serial_fd_, TCIOFLUSH);

        return true;
    }

    void disconnect() {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
            is_connected_ = false;
            std::cout << "Disconnected from Arduino" << std::endl;
        }
    }

    bool isConnected() const {
        return is_connected_ && serial_fd_ >= 0;
    }

    bool readData(MPU6050Data& data) {
        if (!isConnected()) {
            return false;
        }

        try {
            std::string line = readLine();

            // Skip INFO and ERROR messages
            if (line.find("INFO:") == 0 || line.find("ERROR:") == 0) {
                std::cout << line << std::endl;
                return false;
            }

            // Parse the data line
            if (parseLine(line, data)) {
                // Add timestamp
                auto now = std::chrono::system_clock::now();
                auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
                data.timestamp_ms = now_ms.time_since_epoch().count();
                return true;
            }

            return false;
        } catch (const std::exception& e) {
            std::cerr << "Error reading data: " << e.what() << std::endl;
            is_connected_ = false;
            return false;
        }
    }

    void printData(const MPU6050Data& data) const {
        std::cout << "Timestamp: " << data.timestamp_ms << " ms | ";
        std::cout << "Accel(x,y,z): ("
                  << data.accel_x << ", "
                  << data.accel_y << ", "
                  << data.accel_z << ") | ";
        std::cout << "Gyro(x,y,z): ("
                  << data.gyro_x << ", "
                  << data.gyro_y << ", "
                  << data.gyro_z << ") | ";
        std::cout << "Temp: " << data.temperature << std::endl;
    }

    // Convert raw values to physical units
    double accelToG(int16_t raw_value) const {
        // For +/- 2g range: sensitivity = 16384 LSB/g
        return raw_value / 16384.0;
    }

    double gyroToDegPerSec(int16_t raw_value) const {
        // For +/- 250 deg/s range: sensitivity = 131 LSB/(deg/s)
        return raw_value / 131.0;
    }

    double tempToCelsius(int16_t raw_value) const {
        // Temperature formula: Temp = (TEMP_OUT / 340) + 36.53
        return (raw_value / 340.0) + 36.53;
    }

private:
    std::string readLine() {
        std::string result;
        char c;

        while (true) {
            ssize_t n = read(serial_fd_, &c, 1);

            if (n < 0) {
                throw std::runtime_error("Error reading from serial port: " +
                                       std::string(strerror(errno)));
            } else if (n == 0) {
                // Timeout occurred
                continue;
            }

            if (c == '\n') {
                break;
            } else if (c != '\r') {
                result += c;
            }
        }

        return result;
    }

    bool parseLine(const std::string& line, MPU6050Data& data) {
        // Expected format: AX:value,AY:value,AZ:value,GX:value,GY:value,GZ:value,TEMP:value
        std::istringstream iss(line);
        std::string token;

        try {
            std::vector<std::string> tokens;
            while (std::getline(iss, token, ',')) {
                tokens.push_back(token);
            }

            if (tokens.size() != 7) {
                return false;
            }

            data.accel_x = extractValue(tokens[0], "AX:");
            data.accel_y = extractValue(tokens[1], "AY:");
            data.accel_z = extractValue(tokens[2], "AZ:");
            data.gyro_x = extractValue(tokens[3], "GX:");
            data.gyro_y = extractValue(tokens[4], "GY:");
            data.gyro_z = extractValue(tokens[5], "GZ:");
            data.temperature = extractValue(tokens[6], "TEMP:");

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing line: " << e.what() << std::endl;
            return false;
        }
    }

    int16_t extractValue(const std::string& token, const std::string& prefix) {
        size_t pos = token.find(prefix);
        if (pos != std::string::npos) {
            std::string value_str = token.substr(pos + prefix.length());
            return static_cast<int16_t>(std::stoi(value_str));
        }
        throw std::runtime_error("Invalid token format: " + token);
    }

    std::string port_name_;
    unsigned int baud_rate_;
    int serial_fd_;
    bool is_connected_;
};

int main(int argc, char* argv[]) {
    std::string port_name = "/dev/ttyACM0"; // Default port

    if (argc > 1) {
        port_name = argv[1];
    }

    std::cout << "MPU6050 Reader - Connecting to Arduino on " << port_name << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl << std::endl;

    MPU6050Reader reader(port_name);

    if (!reader.connect()) {
        std::cerr << "Failed to connect to Arduino. Please check:" << std::endl;
        std::cerr << "  1. Arduino is connected to " << port_name << std::endl;
        std::cerr << "  2. You have permission to access the port (try: sudo usermod -a -G dialout $USER)" << std::endl;
        std::cerr << "  3. Arduino is running MPU6050Arduino.c sketch" << std::endl;
        return 1;
    }

    MPU6050Data data;
    uint32_t packet_count = 0;

    while (reader.isConnected()) {
        if (reader.readData(data)) {
            packet_count++;

            // Print raw values
            std::cout << "[Packet #" << packet_count << "] ";
            reader.printData(data);

            // Optional: Print converted values
            // std::cout << "  Accel(g): ("
            //           << reader.accelToG(data.accel_x) << ", "
            //           << reader.accelToG(data.accel_y) << ", "
            //           << reader.accelToG(data.accel_z) << ")" << std::endl;
            // std::cout << "  Gyro(deg/s): ("
            //           << reader.gyroToDegPerSec(data.gyro_x) << ", "
            //           << reader.gyroToDegPerSec(data.gyro_y) << ", "
            //           << reader.gyroToDegPerSec(data.gyro_z) << ")" << std::endl;
            // std::cout << "  Temperature: " << reader.tempToCelsius(data.temperature) << " °C" << std::endl;
        }
    }

    std::cout << "Connection lost. Total packets received: " << packet_count << std::endl;

    return 0;
}

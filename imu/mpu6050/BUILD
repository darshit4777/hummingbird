# Build file for MPU6050 IMU reader

cc_binary(
    name = "Mpu6050Reader",
    srcs = ["Mpu6050Reader.cpp"],
    copts = [
        "-std=c++11",
        "-Wall",
        "-Wextra",
    ],
    linkopts = [
        "-lpthread",
    ],
    visibility = ["//visibility:public"],
)

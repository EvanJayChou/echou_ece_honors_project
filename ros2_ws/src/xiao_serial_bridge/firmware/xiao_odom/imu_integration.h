#pragma once
#include "LSM6DS3.h"

struct ImuState {
    float gyro_bias_z = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    float vx = 0.0f;
    float vtheta = 0.0f;
    unsigned long last_us = 0;
};

// Blocks ~2 s collecting n samples to estimate gyro Z bias. Board must be still.
inline void calibrateGyroBias(LSM6DS3 &imu, ImuState &state, int n) {
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += imu.readFloatGyroZ();
        delay(10);
    }
    state.gyro_bias_z = (float)(sum / n);
}

// Call once per IMU sample. dt_s is elapsed time in seconds.
inline void integrateImu(LSM6DS3 &imu, ImuState &state, float dt_s) {
    float gz = (imu.readFloatGyroZ() - state.gyro_bias_z) * DEG_TO_RAD;
    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();

    state.vtheta = gz;
    state.theta += gz * dt_s;

    // Rotate accel from body frame to world frame
    float cos_t = cosf(state.theta);
    float sin_t = sinf(state.theta);
    float ax_w = ax * cos_t - ay * sin_t;
    float ay_w = ax * sin_t + ay * cos_t;

    state.vx = ax_w * dt_s;  // simple Euler — good enough for slow rover
    state.x += ax_w * dt_s * dt_s * 0.5f;
    state.y += ay_w * dt_s * dt_s * 0.5f;
}

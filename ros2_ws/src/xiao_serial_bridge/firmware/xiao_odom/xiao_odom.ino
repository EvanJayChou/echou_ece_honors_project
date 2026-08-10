#include "LSM6DS3.h"
#include "Wire.h"
#include "imu_integration.h"

LSM6DS3 imu(I2C_MODE, 0x6A);
ImuState state;

// Output rate: 50 Hz (every other 104 Hz sample)
static const unsigned long OUTPUT_INTERVAL_MS = 20;  // 50 Hz
static unsigned long last_output_ms = 0;
static unsigned long seq = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}

    if (imu.begin() != 0) {
        Serial.println("ERR:IMU_INIT");
        while (1) {}
    }

    // Hold still ~2 s for gyro bias calibration
    calibrateGyroBias(imu, state, 200);
    state.last_us = micros();
}

void loop() {
    unsigned long now_us = micros();
    float dt_s = (now_us - state.last_us) * 1e-6f;
    state.last_us = now_us;

    integrateImu(imu, state, dt_s);

    unsigned long now_ms = millis();
    if (now_ms - last_output_ms >= OUTPUT_INTERVAL_MS) {
        last_output_ms = now_ms;

        // SEQ,TIMESTAMP_MS,AX,AY,AZ,GX,GY,GZ,X,Y,THETA,VX,VTHETA
        Serial.print(seq++);           Serial.print(',');
        Serial.print(now_ms);          Serial.print(',');
        Serial.print(imu.readFloatAccelX(), 6); Serial.print(',');
        Serial.print(imu.readFloatAccelY(), 6); Serial.print(',');
        Serial.print(imu.readFloatAccelZ(), 6); Serial.print(',');
        Serial.print(imu.readFloatGyroX() * DEG_TO_RAD, 6);  Serial.print(',');
        Serial.print(imu.readFloatGyroY() * DEG_TO_RAD, 6);  Serial.print(',');
        Serial.print(state.vtheta, 6);  Serial.print(',');  // bias-corrected, matches dead-reckoning
        Serial.print(state.x, 6);     Serial.print(',');
        Serial.print(state.y, 6);     Serial.print(',');
        Serial.print(state.theta, 6); Serial.print(',');
        Serial.print(state.vx, 6);    Serial.print(',');
        Serial.println(state.vtheta, 6);
    }
}

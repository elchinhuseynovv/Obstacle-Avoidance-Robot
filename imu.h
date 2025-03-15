#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include "config.h"

class IMU {
private:
    const int MPU_ADDR = 0x68;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
    unsigned long lastUpdate;

public:
    IMU() : accelX(0), accelY(0), accelZ(0),
            gyroX(0), gyroY(0), gyroZ(0),
            roll(0), pitch(0), yaw(0),
            lastUpdate(0) {}

    void init() {
        Wire.begin();
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x6B);  // PWR_MGMT_1 register
        Wire.write(0);     // Wake up MPU-6050
        Wire.endTransmission(true);

        // Configure Accelerometer
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1C);  // ACCEL_CONFIG register
        Wire.write(0x10);  // ±8g range
        Wire.endTransmission(true);

        // Configure Gyroscope
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1B);  // GYRO_CONFIG register
        Wire.write(0x10);  // ±1000°/s range
        Wire.endTransmission(true);
    }

    void update() {
        if (millis() - lastUpdate < IMU_UPDATE_INTERVAL) return;

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);  // Starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);  // Request 14 registers

        // Read accelerometer data
        accelX = (Wire.read() << 8 | Wire.read()) / 4096.0;
        accelY = (Wire.read() << 8 | Wire.read()) / 4096.0;
        accelZ = (Wire.read() << 8 | Wire.read()) / 4096.0;

        // Skip temperature
        Wire.read(); Wire.read();

        // Read gyroscope data
        gyroX = (Wire.read() << 8 | Wire.read()) / 32.8;
        gyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
        gyroZ = (Wire.read() << 8 | Wire.read()) / 32.8;

        // Calculate orientation
        calculateOrientation();
        
        lastUpdate = millis();
    }

    float getRoll() { return roll; }
    float getPitch() { return pitch; }
    float getYaw() { return yaw; }
    
    void getAcceleration(float& x, float& y, float& z) {
        x = accelX;
        y = accelY;
        z = accelZ;
    }
    
    void getGyroscope(float& x, float& y, float& z) {
        x = gyroX;
        y = gyroY;
        z = gyroZ;
    }

private:
    void calculateOrientation() {
        // Simple complementary filter
        float dt = (millis() - lastUpdate) / 1000.0;
        
        // Calculate pitch and roll from accelerometer
        float accelRoll = atan2(accelY, accelZ) * RAD_TO_DEG;
        float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;
        
        // Integrate gyroscope data
        roll = 0.96 * (roll + gyroX * dt) + 0.04 * accelRoll;
        pitch = 0.96 * (pitch + gyroY * dt) + 0.04 * accelPitch;
        yaw += gyroZ * dt;
    }
};

#endif
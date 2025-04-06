#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include "config.h"

// Define conversion constants if not already defined
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif

class IMU {
private:
    const int MPU_ADDR = 0x68;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
    float temperature;
    unsigned long lastUpdate;
    bool initialized;
    
    // Calibration offsets
    float accelOffsetX, accelOffsetY, accelOffsetZ;
    float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    
    // Filter coefficients
    const float ALPHA = 0.96; // Complementary filter coefficient
    const float BETA = 0.04;  // 1 - ALPHA

public:
    IMU() : accelX(0), accelY(0), accelZ(0),
            gyroX(0), gyroY(0), gyroZ(0),
            roll(0), pitch(0), yaw(0),
            temperature(0), lastUpdate(0),
            initialized(false),
            accelOffsetX(0), accelOffsetY(0), accelOffsetZ(0),
            gyroOffsetX(0), gyroOffsetY(0), gyroOffsetZ(0) {}

    bool init() {
        Wire.begin();
        
        // Check if MPU is responding
        Wire.beginTransmission(MPU_ADDR);
        if (Wire.endTransmission() != 0) {
            if (DEBUG_MODE) {
                Serial.println(F("IMU initialization failed"));
            }
            return false;
        }

        // Wake up MPU-6050
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x6B);  // PWR_MGMT_1 register
        Wire.write(0);     // Wake up MPU-6050
        if (Wire.endTransmission(true) != 0) return false;

        // Configure Accelerometer (±8g)
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1C);  // ACCEL_CONFIG register
        Wire.write(0x10);  // ±8g range
        if (Wire.endTransmission(true) != 0) return false;

        // Configure Gyroscope (±1000°/s)
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1B);  // GYRO_CONFIG register
        Wire.write(0x10);  // ±1000°/s range
        if (Wire.endTransmission(true) != 0) return false;

        // Configure low pass filter
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1A);  // CONFIG register
        Wire.write(0x03);  // Set digital low pass filter to ~43Hz
        if (Wire.endTransmission(true) != 0) return false;

        // Perform calibration
        if (!calibrate()) {
            if (DEBUG_MODE) {
                Serial.println(F("IMU calibration failed"));
            }
            return false;
        }

        initialized = true;
        if (DEBUG_MODE) {
            Serial.println(F("IMU initialized successfully"));
        }
        return true;
    }

    bool update() {
        if (!initialized || (millis() - lastUpdate < IMU_UPDATE_INTERVAL)) {
            return false;
        }

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);  // Starting with register 0x3B (ACCEL_XOUT_H)
        if (Wire.endTransmission(false) != 0) return false;
        
        if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) {
            return false;
        }

        // Read and apply calibration to accelerometer data
        accelX = ((Wire.read() << 8 | Wire.read()) / 4096.0) - accelOffsetX;
        accelY = ((Wire.read() << 8 | Wire.read()) / 4096.0) - accelOffsetY;
        accelZ = ((Wire.read() << 8 | Wire.read()) / 4096.0) - accelOffsetZ;

        // Read temperature
        int16_t tempRaw = Wire.read() << 8 | Wire.read();
        temperature = tempRaw / 340.0 + 36.53;

        // Read and apply calibration to gyroscope data
        gyroX = ((Wire.read() << 8 | Wire.read()) / 32.8) - gyroOffsetX;
        gyroY = ((Wire.read() << 8 | Wire.read()) / 32.8) - gyroOffsetY;
        gyroZ = ((Wire.read() << 8 | Wire.read()) / 32.8) - gyroOffsetZ;

        // Calculate orientation with improved filtering
        calculateOrientation();
        
        lastUpdate = millis();
        return true;
    }

    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    float getTemperature() const { return temperature; }
    
    void getAcceleration(float& x, float& y, float& z) const {
        x = accelX;
        y = accelY;
        z = accelZ;
    }
    
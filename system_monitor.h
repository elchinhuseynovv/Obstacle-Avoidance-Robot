#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include "config.h"
#include "battery.h"
#include "motors.h"
#include "imu.h"

class SystemMonitor {
private:
    BatteryMonitor& battery;
    Motors& motors;
    IMU& imu;
    
    struct SystemHealth {
        bool batteryOk;
        bool motorsOk;
        bool sensorsOk;
        bool imuOk;
        float systemTemperature;
        unsigned long lastCheck;
    } health;

public:
    SystemMonitor(BatteryMonitor& b, Motors& m, IMU& i)
        : battery(b), motors(m), imu(i) {
        resetHealth();
    }

    bool checkSystem() {
        if (millis() - health.lastCheck < 1000) {
            return isSystemHealthy();
        }

        health.batteryOk = checkBattery();
        health.motorsOk = checkMotors();
        health.imuOk = checkIMU();
        health.systemTemperature = getSystemTemperature();
        health.lastCheck = millis();

        return isSystemHealthy();
    }

    String getHealthReport() {
        String report = "System Health Report:\n";
        report += "Battery: " + String(health.batteryOk ? "OK" : "FAIL") + "\n";
        report += "Motors: " + String(health.motorsOk ? "OK" : "FAIL") + "\n";
        report += "IMU: " + String(health.imuOk ? "OK" : "FAIL") + "\n";
        report += "Temperature: " + String(health.systemTemperature) + "°C\n";
        return report;
    }

private:
    void resetHealth() {
        health.batteryOk = true;
        health.motorsOk = true;
        health.sensorsOk = true;
        health.imuOk = true;
        health.systemTemperature = 25.0;
        health.lastCheck = 0;
    }

    bool checkBattery() {
        float voltage = battery.getVoltage();
        return voltage >= MINIMUM_BATTERY_VOLTAGE;
    }

    bool checkMotors() {
        return motors.selfTest();
    }

    bool checkIMU() {
        float roll = imu.getRoll();
        float pitch = imu.getPitch();
        return abs(roll) <= MAX_SAFE_ROLL && abs(pitch) <= MAX_SAFE_PITCH;
    }

    float getSystemTemperature() {
        // Combine readings from various sensors
        float imuTemp = imu.getTemperature();
        float motorTemp = motors.getStatus(0).temperature;
        return (imuTemp + motorTemp) / 2.0;
    }

    bool isSystemHealthy() {
        return health.batteryOk && health.motorsOk && health.imuOk &&
               health.systemTemperature < MAX_SAFE_TEMPERATURE;
    }
};

#endif
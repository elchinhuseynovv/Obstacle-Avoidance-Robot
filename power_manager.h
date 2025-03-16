#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "config.h"
#include "battery.h"

class PowerManager {
private:
    BatteryMonitor& battery;
    unsigned long lastPowerCheck;
    float powerConsumption;
    bool sleepMode;
    
    struct PowerMetrics {
        float averagePowerDraw;
        float peakPowerDraw;
        unsigned long operatingTime;
        int sleepCycles;
        float totalEnergyUsed; // in mWh
    } metrics;

public:
    PowerManager(BatteryMonitor& b) 
        : battery(b), lastPowerCheck(0), powerConsumption(0), sleepMode(false) {
        resetMetrics();
    }

    void init() {
        // Initialize power management features
        pinMode(POWER_LED_PIN, OUTPUT);
        digitalWrite(POWER_LED_PIN, HIGH);
    }

    void update() {
        unsigned long currentTime = millis();
        if (currentTime - lastPowerCheck > POWER_CHECK_INTERVAL) {
            float voltage = battery.getVoltage();
            updatePowerMetrics(voltage);
            checkPowerState(voltage);
            lastPowerCheck = currentTime;
        }
    }

    bool shouldEnterSleepMode() {
        return battery.getVoltage() < SLEEP_VOLTAGE_THRESHOLD;
    }

    void enterSleepMode() {
        if (!sleepMode) {
            sleepMode = true;
            metrics.sleepCycles++;
            digitalWrite(POWER_LED_PIN, LOW);
            // Implement additional power saving measures
        }
    }

    void exitSleepMode() {
        if (sleepMode) {
            sleepMode = false;
            digitalWrite(POWER_LED_PIN, HIGH);
        }
    }

    PowerMetrics getMetrics() const {
        return metrics;
    }

    float getCurrentPowerConsumption() const {
        return powerConsumption;
    }

private:
    void updatePowerMetrics(float voltage) {
        unsigned long currentTime = millis();
        float current = estimateCurrent(voltage);
        powerConsumption = voltage * current;

        // Update metrics
        metrics.averagePowerDraw = (metrics.averagePowerDraw * 0.9) + (powerConsumption * 0.1);
        metrics.peakPowerDraw = max(metrics.peakPowerDraw, powerConsumption);
        metrics.operatingTime = currentTime;
        metrics.totalEnergyUsed += (powerConsumption * POWER_CHECK_INTERVAL) / 3600000.0; // Convert to mWh
    }

    float estimateCurrent(float voltage) {
        // Simple current estimation based on voltage and known load
        // This should be calibrated for actual hardware
        const float baseCurrentDraw = 100.0; // mA
        const float voltageScale = 1.2;
        return baseCurrentDraw * (voltage / voltageScale);
    }

    void checkPowerState(float voltage) {
        if (voltage < CRITICAL_BATTERY_THRESHOLD) {
            enterEmergencyShutdown();
        } else if (voltage < SLEEP_VOLTAGE_THRESHOLD) {
            enterSleepMode();
        } else if (sleepMode && voltage > (SLEEP_VOLTAGE_THRESHOLD + 0.5)) {
            exitSleepMode();
        }
    }

    void enterEmergencyShutdown() {
        // Implement emergency shutdown procedure
        digitalWrite(POWER_LED_PIN, LOW);
        // Save critical data before shutdown
        while(1); // Hold in shutdown state
    }

    void resetMetrics() {
        metrics.averagePowerDraw = 0;
        metrics.peakPowerDraw = 0;
        metrics.operatingTime = 0;
        metrics.sleepCycles = 0;
        metrics.totalEnergyUsed = 0;
    }
};

#endif
#ifndef BATTERY_H
#define BATTERY_H

#include "config.h"

class BatteryMonitor {
private:
    float voltage;
    float previousVoltage;
    unsigned long lastCheck;
    unsigned long lastWarning;
    bool lowBatteryWarning;
    bool criticalBatteryWarning;
    const unsigned long WARNING_INTERVAL = 30000; // 30 seconds between warnings
    
    // Moving average filter for voltage readings
    static const int VOLTAGE_SAMPLES = 10;
    float voltageReadings[VOLTAGE_SAMPLES];
    int readingIndex;
    bool bufferFilled;

    // Battery health monitoring
    float voltageDropRate;
    unsigned long monitoringStartTime;
    float initialVoltage;
    float lowestVoltage;
    float highestVoltage;
    int warningCount;

public:
    BatteryMonitor() 
        : voltage(0.0), previousVoltage(0.0), lastCheck(0), lastWarning(0),
          lowBatteryWarning(false), criticalBatteryWarning(false),
          readingIndex(0), bufferFilled(false),
          voltageDropRate(0.0), monitoringStartTime(0),
          initialVoltage(0.0), lowestVoltage(99.0),
          highestVoltage(0.0), warningCount(0) {
        for (int i = 0; i < VOLTAGE_SAMPLES; i++) {
            voltageReadings[i] = 0.0;
        }
    }

    void init() {
        pinMode(BATTERY_PIN, INPUT);
        monitoringStartTime = millis();
        
        // Take initial readings
        voltage = readVoltage();
        initialVoltage = voltage;
        previousVoltage = voltage;
        
        if (DEBUG_MODE) {
            Serial.print(F("Battery Monitor initialized. Initial voltage: "));
            Serial.println(voltage);
        }
    }

    float getVoltage() {
        if (millis() - lastCheck >= BATTERY_CHECK_MS) {
            previousVoltage = voltage;
            voltage = readVoltage();
            updateBatteryMetrics();
            checkBatteryStatus();
            lastCheck = millis();
        }
        return voltage;
    }

    bool isLowBattery() const {
        return voltage < LOW_BATTERY_THRESHOLD;
    }

    bool isCriticalBattery() const {
        return voltage < CRITICAL_BATTERY_THRESHOLD;
    }

    float getVoltageDropRate() const {
        return voltageDropRate;
    }

    struct BatteryStatus {
        float currentVoltage;
        float dropRate;
        float lowestVoltage;
        float highestVoltage;
        unsigned long uptime;
        int warningCount;
        bool isLow;
        bool isCritical;
    };

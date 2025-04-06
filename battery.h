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

    BatteryStatus getStatus() const {
        return {
            voltage,
            voltageDropRate,
            lowestVoltage,
            highestVoltage,
            (millis() - monitoringStartTime) / 1000, // Convert to seconds
            warningCount,
            isLowBattery(),
            isCriticalBattery()
        };
    }

    bool performSelfTest() {
        float testVoltage = readVoltage();
        
        // Check if voltage reading is within possible range
        if (testVoltage < 0.0 || testVoltage > 15.0) {
            return false;
        }
        
        // Check if ADC is working by taking multiple readings
        float readings[3];
        for (int i = 0; i < 3; i++) {
            readings[i] = readVoltage();
            delay(10);
        }
        
        // Check for stuck ADC (all readings identical)
        if (readings[0] == readings[1] && readings[1] == readings[2]) {
            return false;
        }
        
        return true;
    }

private:
    float readVoltage() {
        // Take multiple readings for accuracy
        float sum = 0.0;
        const int samples = 5;
        
        for (int i = 0; i < samples; i++) {
            int rawValue = analogRead(BATTERY_PIN);
            float reading = (rawValue * VOLTAGE_REFERENCE) / 1023.0 * VOLTAGE_DIVIDER;
            sum += reading;
            delay(1); // Short delay between readings
        }
        
        float instantVoltage = sum / samples;
        
        // Update moving average
        voltageReadings[readingIndex] = instantVoltage;
        readingIndex = (readingIndex + 1) % VOLTAGE_SAMPLES;
        if (readingIndex == 0) {
            bufferFilled = true;
        }
        
        // Calculate moving average
        sum = 0.0;
        int count = bufferFilled ? VOLTAGE_SAMPLES : readingIndex;
        for (int i = 0; i < count; i++) {
            sum += voltageReadings[i];
        }
        
        return count > 0 ? sum / count : instantVoltage;
    }

    void updateBatteryMetrics() {
        // Update voltage extremes
        if (voltage < lowestVoltage) lowestVoltage = voltage;
        if (voltage > highestVoltage) highestVoltage = voltage;
        
        // Calculate voltage drop rate (V/hour)
        unsigned long runtime = (millis() - monitoringStartTime) / 1000; // seconds
        if (runtime > 0) {
            voltageDropRate = (initialVoltage - voltage) / (runtime / 3600.0);
        }
    }

    void checkBatteryStatus() {
        unsigned long now = millis();
        
        // Check for sudden voltage drops
        float voltageDrop = previousVoltage - voltage;
        if (voltageDrop > 0.5) { // Sudden 0.5V drop
            if (DEBUG_MODE) {
                Serial.println(F("WARNING: Sudden voltage drop detected!"));
            }
            warningCount++;
        }
        
        // Low battery warning
        if (isLowBattery() && !lowBatteryWarning) {
            lowBatteryWarning = true;
            warningCount++;
            if (now - lastWarning >= WARNING_INTERVAL) {
                if (DEBUG_MODE) {
                    Serial.println(F("WARNING: Low battery!"));
                    Serial.print(F("Voltage: "));
                    Serial.println(voltage);
                }
                lastWarning = now;
            }
        }
        
#ifndef BATTERY_H
#define BATTERY_H

#include "config.h"

class BatteryMonitor {
private:
    float voltage;
    unsigned long lastCheck;
    bool lowBatteryWarning;

public:
    BatteryMonitor() : voltage(0.0), lastCheck(0), lowBatteryWarning(false) {}

    void init() {
        pinMode(BATTERY_PIN, INPUT);
    }

    float getVoltage() {
        if (millis() - lastCheck > BATTERY_CHECK_MS) {
            int rawValue = analogRead(BATTERY_PIN);
            voltage = (rawValue * VOLTAGE_REFERENCE) / 1023.0 * VOLTAGE_DIVIDER;
            lastCheck = millis();
            
            checkBatteryStatus();
        }
        return voltage;
    }

    bool isLowBattery() {
        return voltage < LOW_BATTERY_THRESHOLD;
    }

private:
    void checkBatteryStatus() {
        if (isLowBattery() && !lowBatteryWarning) {
            lowBatteryWarning = true;
            if (DEBUG_MODE) {
                Serial.println("WARNING: Low battery!");
                Serial.print("Voltage: ");
                Serial.println(voltage);
            }
        }
    }
};

#endif
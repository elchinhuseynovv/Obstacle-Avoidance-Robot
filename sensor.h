#ifndef SENSOR_H
#define SENSOR_H

#include "config.h"

class UltrasonicSensor {
private:
    int trigPin;
    int echoPin;
    unsigned long lastRead;
    int lastDistance;
    bool initialized;
    
    struct FilteredData {
        int values[5];
        int index;
        bool filled;
    } filter;

public:
    UltrasonicSensor() : lastRead(0), lastDistance(0), initialized(false) {
        resetFilter();
    }

    void init(int trig, int echo) {
        trigPin = trig;
        echoPin = echo;
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
        initialized = true;
    }

    int getDistance() {
        if (!initialized) return -1;
        
        if (millis() - lastRead < 50) { // Limit reading frequency
            return lastDistance;
        }
        
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        
        long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT);
        int distance = duration * 0.034 / 2;
        
        // Validate and filter reading
        if (isValidReading(distance)) {
            lastDistance = applyFilter(distance);
            lastRead = millis();
        }
        
        return lastDistance;
    }
    
    bool selfTest() {
        if (!initialized) return false;
        
        // Perform multiple readings
        int validReadings = 0;
        for (int i = 0; i < 5; i++) {
            int distance = getDistance();
            if (distance >= 0 && distance <= 400) {
                validReadings++;
            }
            delay(50);
        }
        
        return validReadings >= 3; // At least 3 valid readings
    }

private:
    void resetFilter() {
        filter.index = 0;
        filter.filled = false;
        for (int i = 0; i < 5; i++) {
            filter.values[i] = 0;
        }
    }
    
    bool isValidReading(int distance) {
        // Check if reading is within valid range
        if (distance < 0 || distance > 400) {
            return false;
        }
        
        // Check for sudden large changes
        if (lastDistance > 0) {
            int maxChange = lastDistance * 0.3; // Max 30% change
            if (abs(distance - lastDistance) > maxChange) {
                return false;
            }
        }
        
        return true;
    }
    
    int applyFilter(int newValue) {
        // Update circular buffer
        filter.values[filter.index] = newValue;
        filter.index = (filter.index + 1) % 5;
        
        if (filter.index == 0) {
            filter.filled = true;
        }
        
        // Calculate median if buffer is filled
        if (filter.filled) {
            int temp[5];
            memcpy(temp, filter.values, sizeof(temp));
            
            // Simple bubble sort for median
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4 - i; j++) {
                    if (temp[j] > temp[j + 1]) {
                        int t = temp[j];
                        temp[j] = temp[j + 1];
                        temp[j + 1] = t;
                    }
                }
            }
            
            return temp[2]; // Return median value
        }
        
        return newValue;
    }
};

#endif
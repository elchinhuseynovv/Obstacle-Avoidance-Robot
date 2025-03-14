#ifndef SENSOR_H
#define SENSOR_H

#include "config.h"

class UltrasonicSensor {
private:
    unsigned long lastDistanceCheck;

public:
    UltrasonicSensor() : lastDistanceCheck(0) {}

    void init() {
        pinMode(TRIG, OUTPUT);
        pinMode(ECHO, INPUT);
    }

    int getDistance() {
        long duration;
        int distance;
        
        digitalWrite(TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG, LOW);
        
        duration = pulseIn(ECHO, HIGH, SENSOR_TIMEOUT);
        distance = duration * 0.034 / 2;
        
        // Error checking
        if (distance <= 0 || distance > 400) {
            if (DEBUG_MODE) {
                Serial.println("Warning: Invalid distance reading");
            }
            return 400; // Return maximum range if invalid
        }
        
        // Debug output
        if (DEBUG_MODE && millis() - lastDistanceCheck > DEBUG_UPDATE_MS) {
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" cm");
            lastDistanceCheck = millis();
        }
        
        return distance;
    }
};

#endif
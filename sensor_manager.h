#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "config.h"
#include "sensor.h"

class SensorManager {
private:
    UltrasonicSensor frontSensor;
    UltrasonicSensor leftSensor;
    UltrasonicSensor rightSensor;
    
    struct SensorData {
        int frontDistance;
        int leftDistance;
        int rightDistance;
        unsigned long timestamp;
        bool isValid;
    } lastReading;
    
    unsigned long lastUpdate;
    const unsigned long UPDATE_INTERVAL = 50; // 50ms between updates
    const int MAX_VALID_DISTANCE = 400; // Maximum valid distance reading
    
public:
    SensorManager() : lastUpdate(0) {
        resetReadings();
    }
    
    void init() {
        frontSensor.init(TRIG_FRONT, ECHO_FRONT);
        leftSensor.init(TRIG_LEFT, ECHO_LEFT);
        rightSensor.init(TRIG_RIGHT, ECHO_RIGHT);
    }
    
    SensorData update() {
        if (millis() - lastUpdate < UPDATE_INTERVAL) {
            return lastReading;
        }
        
        SensorData newReading;
        newReading.frontDistance = frontSensor.getDistance();
        newReading.leftDistance = leftSensor.getDistance();
        newReading.rightDistance = rightSensor.getDistance();
        newReading.timestamp = millis();
        newReading.isValid = validateReadings(newReading);
        
        if (newReading.isValid) {
            lastReading = newReading;
        }
        
        lastUpdate = millis();
        return lastReading;
    }
    
    bool isFrontClear() {
        return lastReading.frontDistance > SAFE_DISTANCE;
    }
    
    bool isLeftClear() {
        return lastReading.leftDistance > SAFE_DISTANCE;
    }
    
    bool isRightClear() {
        return lastReading.rightDistance > SAFE_DISTANCE;
    }
    
    int getClosestObstacle() {
        return min(min(lastReading.frontDistance, 
                      lastReading.leftDistance), 
                  lastReading.rightDistance);
    }
    
    String getObstacleDirection() {
        int front = lastReading.frontDistance;
        int left = lastReading.leftDistance;
        int right = lastReading.rightDistance;
        
        if (front <= DANGER_DISTANCE) return "FRONT";
        if (left <= DANGER_DISTANCE) return "LEFT";
        if (right <= DANGER_DISTANCE) return "RIGHT";
        return "NONE";
    }
    
    bool performSelfTest() {
        bool success = true;
        
        // Test front sensor
        int frontTest = frontSensor.getDistance();
        success &= (frontTest >= 0 && frontTest <= MAX_VALID_DISTANCE);
        
        // Test left sensor
        int leftTest = leftSensor.getDistance();
        success &= (leftTest >= 0 && leftTest <= MAX_VALID_DISTANCE);
        
        // Test right sensor
        int rightTest = rightSensor.getDistance();
        success &= (rightTest >= 0 && rightTest <= MAX_VALID_DISTANCE);
        
        return success;
    }

private:
    void resetReadings() {
        lastReading.frontDistance = MAX_VALID_DISTANCE;
        lastReading.leftDistance = MAX_VALID_DISTANCE;
        lastReading.rightDistance = MAX_VALID_DISTANCE;
        lastReading.timestamp = 0;
        lastReading.isValid = false;
    }
    
    bool validateReadings(const SensorData& reading) {
        // Check if readings are within valid range
        if (reading.frontDistance < 0 || reading.frontDistance > MAX_VALID_DISTANCE ||
            reading.leftDistance < 0 || reading.leftDistance > MAX_VALID_DISTANCE ||
            reading.rightDistance < 0 || reading.rightDistance > MAX_VALID_DISTANCE) {
            return false;
        }
        
        // Check for sudden large changes
        const int MAX_CHANGE = 50; // Maximum allowed change in cm between readings
        if (abs(reading.frontDistance - lastReading.frontDistance) > MAX_CHANGE ||
            abs(reading.leftDistance - lastReading.leftDistance) > MAX_CHANGE ||
            abs(reading.rightDistance - lastReading.rightDistance) > MAX_CHANGE) {
            return false;
        }
        
        return true;
    }
};

#endif
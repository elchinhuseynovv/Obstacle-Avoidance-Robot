#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "config.h"
#include "motors.h"
#include "sensor.h"

class Navigation {
private:
    Motors& motors;
    UltrasonicSensor& sensor;
    int consecutiveObstacles;
    unsigned long lastObstacleTime;
    NavigationMode currentMode;

public:
    Navigation(Motors& m, UltrasonicSensor& s) 
        : motors(m), sensor(s), consecutiveObstacles(0), 
          lastObstacleTime(0), currentMode(NORMAL) {}

    void update() {
        int distance = sensor.getDistance();
        updateNavigationMode(distance);
        
        switch (currentMode) {
            case NORMAL:
                normalNavigation(distance);
                break;
            case OBSTACLE_AVOIDANCE:
                avoidObstacle(distance);
                break;
            case RECOVERY:
                recoveryMode();
                break;
        }
    }

    NavigationMode getMode() {
        return currentMode;
    }

private:
    void updateNavigationMode(int distance) {
        if (distance <= DANGER_DISTANCE) {
            currentMode = OBSTACLE_AVOIDANCE;
        } else if (consecutiveObstacles >= MAX_CONSECUTIVE_OBSTACLES) {
            currentMode = RECOVERY;
        } else if (distance > SAFE_DISTANCE) {
            currentMode = NORMAL;
        }
    }

    void normalNavigation(int distance) {
        int targetSpeed = calculateSpeed(distance);
        motors.updateSpeed(targetSpeed);
        motors.forward(motors.getCurrentSpeed());
        consecutiveObstacles = 0;
    }

    void avoidObstacle(int distance) {
        consecutiveObstacles++;
        motors.stop();
        
        // Advanced obstacle avoidance strategy
        if (millis() - lastObstacleTime > OBSTACLE_TIMEOUT) {
            motors.backward(MIN_SPEED);
            delay(REVERSE_TIME_MS);
            
            // Scan surroundings for best path
            int leftDistance = scanDirection(-90);
            int rightDistance = scanDirection(90);
            
            if (leftDistance > rightDistance) {
                motors.turnLeft(120);
            } else {
                motors.turnRight(120);
            }
            
            lastObstacleTime = millis();
        }
    }

    void recoveryMode() {
        // Implement recovery behavior when stuck
        motors.stop();
        motors.backward(MIN_SPEED);
        delay(RECOVERY_REVERSE_TIME);
        motors.turnRight(180);
        consecutiveObstacles = 0;
        currentMode = NORMAL;
    }

    int scanDirection(int angle) {
        motors.turnRight(angle);
        delay(100);  // Stabilize readings
        int distance = sensor.getDistance();
        motors.turnLeft(angle);  // Return to original position
        return distance;
    }

    int calculateSpeed(int distance) {
        if (distance > SAFE_DISTANCE) {
            return MAX_SPEED;
        } else if (distance > CAUTION_DISTANCE) {
            return map(distance, CAUTION_DISTANCE, SAFE_DISTANCE, 
                      MIN_SPEED, MAX_SPEED);
        }
        return MIN_SPEED;
    }
};

#endif
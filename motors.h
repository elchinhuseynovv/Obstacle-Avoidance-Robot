#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

class Motors {
private:
    int currentLeftSpeed;
    int currentRightSpeed;
    unsigned long lastSpeedUpdate;
    unsigned long lastStallCheck;
    bool stallDetected;
    
    // Motor characteristics
    float maxAcceleration;
    float maxDeceleration;
    float motorTemperature;
    unsigned long motorRuntime;
    
    struct MotorStats {
        unsigned long totalRuntime;
        float averageSpeed;
        float maxSpeedReached;
        int stallCount;
        float efficiency;
    } stats;

public:
    Motors() 
        : currentLeftSpeed(0), currentRightSpeed(0), lastSpeedUpdate(0),
          lastStallCheck(0), stallDetected(false), maxAcceleration(ACCELERATION),
          maxDeceleration(DECELERATION), motorTemperature(25.0), motorRuntime(0) {
        resetStats();
    }

    void init() {
        pinMode(ENA, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(IN4, OUTPUT);
        pinMode(ENB, OUTPUT);
        stop();
    }

    void setSpeed(int leftSpeed, int rightSpeed) {
        // Implement smooth acceleration
        smoothSpeedTransition(currentLeftSpeed, leftSpeed, currentRightSpeed, rightSpeed);
        
        // Update motor speeds
        analogWrite(ENA, currentLeftSpeed);
        analogWrite(ENB, currentRightSpeed);
        
        // Update statistics
        updateStats();
    }

    void forward(int speed) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        setSpeed(speed, speed);
    }

    void backward(int speed) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        setSpeed(speed, speed);
    }

    void stop() {
        // Implement smooth deceleration
        while (currentLeftSpeed > 0 || currentRightSpeed > 0) {
            if (currentLeftSpeed > 0) {
                currentLeftSpeed = max(0, currentLeftSpeed - maxDeceleration);
            }
            if (currentRightSpeed > 0) {
                currentRightSpeed = max(0, currentRightSpeed - maxDeceleration);
            }
            setSpeed(currentLeftSpeed, currentRightSpeed);
            delay(SPEED_UPDATE_MS);
        }
        
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
    }

    void turn(float angularSpeed) {
        int turnPower = abs(angularSpeed) * (MAX_SPEED / MAX_ANGULAR_SPEED);
        if (angularSpeed > 0) {
            // Turn right
            setSpeed(turnPower, -turnPower);
        } else {
            // Turn left
            setSpeed(-turnPower, turnPower);
        }
    }

    void turnRight(int angle) {
        turn(MAX_ANGULAR_SPEED);
        delay(angle * TURN_DELAY_MS);
        stop();
    }

    void turnLeft(int angle) {
        turn(-MAX_ANGULAR_SPEED);
        delay(angle * TURN_DELAY_MS);
        stop();
    }

    float getAverageSpeed() {
        return (currentLeftSpeed + currentRightSpeed) / 2.0;
    }

    bool isStalled() {
        if (millis() - lastStallCheck > 1000) {
            // Check for stall conditions
            if (currentLeftSpeed > 0 || currentRightSpeed > 0) {
                // Implementation depends on hardware feedback
                // This is a placeholder for actual stall detection
                stallDetected = false; // Replace with actual detection
            }
            lastStallCheck = millis();
        }
        return stallDetected;
    }

    MotorStats getStats() {
        return stats;
    }

    bool selfTest() {
        // Perform basic motor test
        forward(MIN_SPEED);
        delay(100);
        stop();
        delay(100);
        backward(MIN_SPEED);
        delay(100);
        stop();
        
        // Check if motors responded
        // This is a basic test, actual implementation would depend on feedback
        return true; // Replace with actual test result
    }

private:
    void smoothSpeedTransition(int& currentLeft, int targetLeft, 
                             int& currentRight, int targetRight) {
        // Calculate speed differences
        int diffLeft = targetLeft - currentLeft;
        int diffRight = targetRight - currentRight;
        
        // Apply acceleration limits
        if (abs(diffLeft) > maxAcceleration) {
            currentLeft += (diffLeft > 0) ? maxAcceleration : -maxAcceleration;
        } else {
            currentLeft = targetLeft;
        }
        
        if (abs(diffRight) > maxAcceleration) {
            currentRight += (diffRight > 0) ? maxAcceleration : -maxAcceleration;
        } else {
            currentRight = targetRight;
        }
        
        // Constrain speeds
        currentLeft = constrain(currentLeft, -MAX_SPEED, MAX_SPEED);
        currentRight = constrain(currentRight, -MAX_SPEED, MAX_SPEED);
        
        // Update timing
        lastSpeedUpdate = millis();
    }

    void updateStats() {
        unsigned long currentTime = millis();
        float timeElapsed = (currentTime - lastSpeedUpdate) / 1000.0;
        
        // Update runtime
        if (currentLeftSpeed > 0 || currentRightSpeed > 0) {
            motorRuntime += currentTime - lastSpeedUpdate;
        }
        
        // Update maximum speed
        float currentAvgSpeed = getAverageSpeed();
        stats.maxSpeedReached = max(stats.maxSpeedReached, currentAvgSpeed);
        
        // Update average speed (weighted moving average)
        stats.averageSpeed = (stats.averageSpeed * 0.95) + (currentAvgSpeed * 0.05);
        
        // Update efficiency (simplified calculation)
        // In a real implementation, this would consider power consumption
        stats.efficiency = (currentAvgSpeed / MAX_SPEED) * 100.0;
        
        // Update total runtime
        stats.totalRuntime = motorRuntime;
    }

    void resetStats() {
        stats.totalRuntime = 0;
        stats.averageSpeed = 0;
        stats.maxSpeedReached = 0;
        stats.stallCount = 0;
        stats.efficiency = 100.0;
    }
};

#endif
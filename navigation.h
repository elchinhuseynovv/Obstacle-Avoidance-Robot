#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "config.h"
#include "motors.h"
#include "sensor.h"
#include "imu.h"

struct Position {
    float x;
    float y;
    float heading;
};

class Navigation {
private:
    Motors& motors;
    UltrasonicSensor& frontSensor;
    UltrasonicSensor& leftSensor;
    UltrasonicSensor& rightSensor;
    IMU& imu;
    
    Position currentPosition;
    Position targetPosition;
    Position pathMemory[PATH_MEMORY_SIZE];
    
    int pathMemoryIndex;
    int consecutiveObstacles;
    int recoveryAttempts;
    unsigned long lastObstacleTime;
    unsigned long lastPositionUpdate;
    
    NavigationMode currentMode;
    RobotState currentState;
    ErrorCode lastError;

    // Advanced navigation parameters
    float targetHeading;
    float angularSpeed;
    float linearSpeed;
    bool isWallFollowing;
    int wallFollowingSide; // 1 for right, -1 for left

public:
    Navigation(Motors& m, UltrasonicSensor& front, UltrasonicSensor& left, 
              UltrasonicSensor& right, IMU& imuSensor) 
        : motors(m), frontSensor(front), leftSensor(left), rightSensor(right), imu(imuSensor),
          pathMemoryIndex(0), consecutiveObstacles(0), recoveryAttempts(0),
          lastObstacleTime(0), lastPositionUpdate(0), currentMode(NORMAL),
          currentState(IDLE), lastError(NO_ERROR), targetHeading(0),
          angularSpeed(0), linearSpeed(0), isWallFollowing(false), wallFollowingSide(1) {
        
        resetPosition();
    }

    void update() {
        updatePosition();
        updateSensorData();
        
        switch (currentMode) {
            case NORMAL:
                normalNavigation();
                break;
            case OBSTACLE_AVOIDANCE:
                avoidObstacle();
                break;
            case WALL_FOLLOWING:
                followWall();
                break;
            case RECOVERY:
                performRecovery();
                break;
            case POSITION_TRACKING:
                navigateToPosition();
                break;
            default:
                handleError();
                break;
        }
        
        updatePathMemory();
    }

    void setTarget(float x, float y) {
        targetPosition.x = x;
        targetPosition.y = y;
        calculateTargetHeading();
    }

    NavigationMode getMode() { return currentMode; }
    RobotState getState() { return currentState; }
    ErrorCode getLastError() { return lastError; }
    Position getCurrentPosition() { return currentPosition; }

private:
    void updatePosition() {
        if (millis() - lastPositionUpdate < IMU_UPDATE_INTERVAL) return;
        
        float deltaTime = (millis() - lastPositionUpdate) / 1000.0;
        float distance = motors.getAverageDistance();
        float heading = imu.getYaw();
        
        currentPosition.x += distance * cos(heading * DEG_TO_RAD);
        currentPosition.y += distance * sin(heading * DEG_TO_RAD);
        currentPosition.heading = heading;
        
        lastPositionUpdate = millis();
    }

    void updateSensorData() {
        int frontDist = frontSensor.getDistance();
        int leftDist = leftSensor.getDistance();
        int rightDist = rightSensor.getDistance();
        
        // Update navigation mode based on sensor data
        if (frontDist < DANGER_DISTANCE) {
            currentMode = OBSTACLE_AVOIDANCE;
            consecutiveObstacles++;
        } else if (isWallFollowing && (leftDist < WALL_FOLLOW_DISTANCE || 
                  rightDist < WALL_FOLLOW_DISTANCE)) {
            currentMode = WALL_FOLLOWING;
        } else if (consecutiveObstacles >= OBSTACLE_AVOID_THRESHOLD) {
            currentMode = RECOVERY;
        } else {
            currentMode = NORMAL;
            consecutiveObstacles = 0;
        }
    }

    void normalNavigation() {
        float headingError = calculateHeadingError();
        
        if (abs(headingError) > 5.0) {
            // Adjust heading
            float turnSpeed = constrain(headingError * 2, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
            motors.turn(turnSpeed);
        } else {
            // Move forward with speed based on distance to target
            float distanceToTarget = calculateDistanceToTarget();
            float targetSpeed = calculateTargetSpeed(distanceToTarget);
            motors.forward(targetSpeed);
        }
    }

    void avoidObstacle() {
        motors.stop();
        
        // Scan surroundings
        int leftClearance = scanDirection(-90);
        int rightClearance = scanDirection(90);
        
        // Choose best direction
        if (leftClearance > rightClearance && leftClearance > SAFE_DISTANCE) {
            motors.turnLeft(90);
            motors.forward(MAX_SPEED / 2);
        } else if (rightClearance > SAFE_DISTANCE) {
            motors.turnRight(90);
            motors.forward(MAX_SPEED / 2);
        } else {
            // No clear path, enter recovery mode
            currentMode = RECOVERY;
        }
    }

    void followWall() {
        int sideDistance = (wallFollowingSide == 1) ? 
                          rightSensor.getDistance() : 
                          leftSensor.getDistance();
        
        float error = WALL_FOLLOW_DISTANCE - sideDistance;
        float correction = error * 2; // Simple P controller
        
        int baseSpeed = MAX_SPEED / 2;
        if (wallFollowingSide == 1) {
            motors.setSpeed(baseSpeed - correction, baseSpeed + correction);
        } else {
            motors.setSpeed(baseSpeed + correction, baseSpeed - correction);
        }
    }

    void performRecovery() {
        if (recoveryAttempts >= RECOVERY_ATTEMPTS) {
            currentState = ERROR_STATE;
            lastError = STUCK;
            return;
        }
        
        // Back up and turn around
        motors.backward(MIN_SPEED);
        delay(RECOVERY_REVERSE_TIME);
        motors.turnRight(180);
        
        recoveryAttempts++;
        currentMode = NORMAL;
    }

    void navigateToPosition() {
        float distanceToTarget = calculateDistanceToTarget();
        
        if (distanceToTarget < POSITION_TOLERANCE) {
            motors.stop();
            currentState = IDLE;
            return;
        }
        
        float headingError = calculateHeadingError();
        float turnSpeed = constrain(headingError * 2, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
        float forwardSpeed = calculateTargetSpeed(distanceToTarget);
        
        motors.setSpeed(
            forwardSpeed - turnSpeed,
            forwardSpeed + turnSpeed
        );
    }

    void updatePathMemory() {
        pathMemory[pathMemoryIndex] = currentPosition;
        pathMemoryIndex = (pathMemoryIndex + 1) % PATH_MEMORY_SIZE;
    }

    float calculateHeadingError() {
        float targetHeading = atan2(
            targetPosition.y - currentPosition.y,
            targetPosition.x - currentPosition.x
        ) * RAD_TO_DEG;
        
        float error = targetHeading - currentPosition.heading;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }

    float calculateDistanceToTarget() {
        float dx = targetPosition.x - currentPosition.x;
        float dy = targetPosition.y - currentPosition.y;
        return sqrt(dx * dx + dy * dy);
    }

    float calculateTargetSpeed(float distance) {
        if (distance < CAUTION_DISTANCE) {
            return map(distance, 0, CAUTION_DISTANCE, MIN_SPEED, MAX_SPEED);
        }
        return MAX_SPEED;
    }

    int scanDirection(int angle) {
        motors.turnRight(angle);
        delay(100); // Allow readings to stabilize
        int distance = frontSensor.getDistance();
        motors.turnLeft(angle); // Return to original position
        return distance;
    }

    void resetPosition() {
        currentPosition = {0, 0, 0};
        targetPosition = {0, 0, 0};
        pathMemoryIndex = 0;
    }

    void handleError() {
        motors.stop();
        currentState = ERROR_STATE;
    }
};

#endif
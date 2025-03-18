#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "config.h"
#include "motors.h"
#include "sensor.h"
#include "imu.h"
#include "pid_controller.h"
#include <math.h>

struct Position {
    float x;
    float y;
    float heading;
    float velocity;
    float acceleration;
    unsigned long timestamp;
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

    // Enhanced navigation parameters
    float targetHeading;
    float angularSpeed;
    float linearSpeed;
    bool isWallFollowing;
    int wallFollowingSide; // 1 for right, -1 for left
    
    // PID Controllers for navigation
    PIDController headingPID;
    PIDController distancePID;
    PIDController wallFollowPID;

public:
    Navigation(Motors& m, UltrasonicSensor& front, UltrasonicSensor& left, 
              UltrasonicSensor& right, IMU& imuSensor) 
        : motors(m), frontSensor(front), leftSensor(left), rightSensor(right), imu(imuSensor),
          pathMemoryIndex(0), consecutiveObstacles(0), recoveryAttempts(0),
          lastObstacleTime(0), lastPositionUpdate(0), currentMode(NORMAL),
          currentState(IDLE), lastError(NO_ERROR), targetHeading(0),
          angularSpeed(0), linearSpeed(0), isWallFollowing(false), wallFollowingSide(1),
          headingPID(2.0, 0.1, 0.5, 0, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED),
          distancePID(1.0, 0.05, 0.2, 0, -MAX_SPEED, MAX_SPEED),
          wallFollowPID(1.5, 0.1, 0.3, WALL_FOLLOW_DISTANCE, -MAX_SPEED/2, MAX_SPEED/2) {
        
        resetPosition();
        initializeNavigation();
    }

    void update() {
        updatePosition();
        updateSensorData();
        
        // Enhanced state machine
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
        updateMetrics();
    }

    void setTarget(float x, float y) {
        targetPosition.x = x;
        targetPosition.y = y;
        targetPosition.timestamp = millis();
        targetHeading = calculateTargetHeading();
        resetPIDControllers();
    }

    NavigationMode getMode() const { return currentMode; }
    RobotState getState() const { return currentState; }
    ErrorCode getLastError() const { return lastError; }
    Position getCurrentPosition() const { return currentPosition; }
    
    // Enhanced status reporting
    struct NavigationStatus {
        float distanceToTarget;
        float headingError;
        float currentSpeed;
        float targetSpeed;
        NavigationMode mode;
        bool isObstacleDetected;
        unsigned long navigationTime;
    };
    
    NavigationStatus getStatus() const {
        NavigationStatus status;
        status.distanceToTarget = calculateDistanceToTarget();
        status.headingError = calculateHeadingError();
        status.currentSpeed = linearSpeed;
        status.targetSpeed = calculateTargetSpeed(status.distanceToTarget);
        status.mode = currentMode;
        status.isObstacleDetected = isObstacleDetected();
        status.navigationTime = millis() - targetPosition.timestamp;
        return status;
    }

private:
    void initializeNavigation() {
        // Initialize PID controllers
        headingPID.reset();
        distancePID.reset();
        wallFollowPID.reset();
        
        // Initialize position tracking
        resetPosition();
        
        // Set initial navigation parameters
        linearSpeed = 0;
        angularSpeed = 0;
        isWallFollowing = false;
    }

    void updatePosition() {
        if (millis() - lastPositionUpdate < IMU_UPDATE_INTERVAL) return;
        
        float deltaTime = (millis() - lastPositionUpdate) / 1000.0;
        if (deltaTime <= 0) return;
        
        float distance = motors.getAverageDistance();
        float heading = imu.getYaw();
        
        // Enhanced position calculation with velocity and acceleration
        float currentVelocity = distance / deltaTime;
        float acceleration = (currentVelocity - currentPosition.velocity) / deltaTime;
        
        currentPosition.x += distance * cos(heading * DEG_TO_RAD);
        currentPosition.y += distance * sin(heading * DEG_TO_RAD);
        currentPosition.heading = heading;
        currentPosition.velocity = currentVelocity;
        currentPosition.acceleration = acceleration;
        currentPosition.timestamp = millis();
        
        lastPositionUpdate = millis();
    }

    void updateSensorData() {
        int frontDist = frontSensor.getDistance();
        int leftDist = leftSensor.getDistance();
        int rightDist = rightSensor.getDistance();
        
        // Validate sensor readings
        if (frontDist < 0 || leftDist < 0 || rightDist < 0) {
            handleError();
            return;
        }
        
        // Enhanced obstacle detection and mode selection
        if (frontDist < DANGER_DISTANCE) {
            handleObstacleDetected(frontDist);
        } else if (shouldEnterWallFollowing(leftDist, rightDist)) {
            enterWallFollowingMode(leftDist, rightDist);
        } else if (shouldEnterRecoveryMode()) {
            enterRecoveryMode();
        } else {
            currentMode = NORMAL;
            consecutiveObstacles = 0;
        }
    }

    void normalNavigation() {
        float headingError = calculateHeadingError();
        float distanceToTarget = calculateDistanceToTarget();
        
        // Enhanced navigation using PID controllers
        float headingCorrection = headingPID.compute(headingError);
        float speedCorrection = distancePID.compute(distanceToTarget);
        
        // Apply corrections with smooth transitions
        angularSpeed = smoothTransition(angularSpeed, headingCorrection, MAX_ANGULAR_SPEED/10);
        linearSpeed = smoothTransition(linearSpeed, speedCorrection, MAX_SPEED/10);
        
        // Set motor speeds with bounds checking
        float leftSpeed = constrain(linearSpeed - angularSpeed, -MAX_SPEED, MAX_SPEED);
        float rightSpeed = constrain(linearSpeed + angularSpeed, -MAX_SPEED, MAX_SPEED);
        
        motors.setSpeed(leftSpeed, rightSpeed);
        currentState = MOVING;
    }

    void avoidObstacle() {
        static unsigned long avoidanceStartTime = 0;
        static int avoidancePhase = 0;
        
        if (avoidanceStartTime == 0) {
            avoidanceStartTime = millis();
            avoidancePhase = 0;
        }
        
        switch (avoidancePhase) {
            case 0: // Stop and assess
                motors.stop();
                if (millis() - avoidanceStartTime > 500) {
                    avoidancePhase = 1;
                }
                break;
                
            case 1: // Scan surroundings
                {
                    int leftClearance = scanDirection(-90);
                    int rightClearance = scanDirection(90);
                    
                    if (leftClearance > rightClearance && leftClearance > SAFE_DISTANCE) {
                        avoidancePhase = 2; // Turn left
                    } else if (rightClearance > SAFE_DISTANCE) {
                        avoidancePhase = 3; // Turn right
                    } else {
                        avoidancePhase = 4; // No clear path
                    }
                }
                break;
                
            case 2: // Turn left
                motors.turn(-90);
                avoidancePhase = 5;
                break;
                
            case 3: // Turn right
                motors.turn(90);
                avoidancePhase = 5;
                break;
                
            case 4: // No clear path
                currentMode = RECOVERY;
                break;
                
            case 5: // Move forward
                motors.forward(MAX_SPEED / 2);
                if (frontSensor.getDistance() > SAFE_DISTANCE) {
                    avoidanceStartTime = 0;
                    currentMode = NORMAL;
                }
                break;
        }
    }

    void followWall() {
        int sideDistance = (wallFollowingSide == 1) ? 
                          rightSensor.getDistance() : 
                          leftSensor.getDistance();
        
        // Enhanced wall following using PID
        float correction = wallFollowPID.compute(sideDistance);
        
        int baseSpeed = MAX_SPEED / 2;
        if (wallFollowingSide == 1) {
            motors.setSpeed(baseSpeed - correction, baseSpeed + correction);
        } else {
            motors.setSpeed(baseSpeed + correction, baseSpeed - correction);
        }
        
        // Check if wall following should be terminated
        if (shouldExitWallFollowing()) {
            exitWallFollowingMode();
        }
    }

    void performRecovery() {
        static unsigned long recoveryStartTime = 0;
        static int recoveryPhase = 0;
        
        if (recoveryAttempts >= RECOVERY_ATTEMPTS) {
            currentState = ERROR_STATE;
            lastError = STUCK;
            return;
        }
        
        if (recoveryStartTime == 0) {
            recoveryStartTime = millis();
            recoveryPhase = 0;
        }
        
        switch (recoveryPhase) {
            case 0: // Back up
                motors.backward(MIN_SPEED);
                if (millis() - recoveryStartTime > RECOVERY_REVERSE_TIME) {
                    recoveryPhase = 1;
                }
                break;
                
            case 1: // Turn around
                motors.turn(180);
                recoveryPhase = 2;
                break;
                
            case 2: // Move forward
                motors.forward(MIN_SPEED);
                if (frontSensor.getDistance() > SAFE_DISTANCE) {
                    recoveryStartTime = 0;
                    recoveryAttempts++;
                    currentMode = NORMAL;
                }
                break;
        }
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
        
        // Apply smooth speed transitions
        motors.setSpeed(
            smoothTransition(motors.getLeftSpeed(), forwardSpeed - turnSpeed, MAX_SPEED/10),
            smoothTransition(motors.getRightSpeed(), forwardSpeed + turnSpeed, MAX_SPEED/10)
        );
    }

    float calculateTargetHeading() {
        return atan2(
            targetPosition.y - currentPosition.y,
            targetPosition.x - currentPosition.x
        ) * RAD_TO_DEG;
    }

    float calculateHeadingError() const {
        float error = targetHeading - currentPosition.heading;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }

    float smoothTransition(float current, float target, float maxChange) {
        float diff = target - current;
        if (abs(diff) > maxChange) {
            return current + (diff > 0 ? maxChange : -maxChange);
        }
        return target;
    }

    void handleObstacleDetected(int distance) {
        currentMode = OBSTACLE_AVOIDANCE;
        consecutiveObstacles++;
        lastObstacleTime = millis();
    }

    bool shouldEnterWallFollowing(int leftDist, int rightDist) {
        return (leftDist < WALL_FOLLOW_DISTANCE || rightDist < WALL_FOLLOW_DISTANCE) &&
               currentMode != OBSTACLE_AVOIDANCE;
    }

    void enterWallFollowingMode(int leftDist, int rightDist) {
        currentMode = WALL_FOLLOWING;
        isWallFollowing = true;
        wallFollowingSide = (leftDist < rightDist) ? -1 : 1;
        wallFollowPID.reset();
    }

    bool shouldEnterRecoveryMode() {
        return consecutiveObstacles >= OBSTACLE_AVOID_THRESHOLD ||
               (millis() - lastObstacleTime > STUCK_TIMEOUT && currentMode == OBSTACLE_AVOIDANCE);
    }

    void enterRecoveryMode() {
        currentMode = RECOVERY;
        recoveryAttempts = 0;
    }

    bool shouldExitWallFollowing() {
        return frontSensor.getDistance() < DANGER_DISTANCE ||
               (wallFollowingSide == 1 ? rightSensor.getDistance() : leftSensor.getDistance()) > WALL_FOLLOW_DISTANCE * 2;
    }

    void exitWallFollowingMode() {
        isWallFollowing = false;
        currentMode = NORMAL;
    }

    void resetPIDControllers() {
        headingPID.reset();
        distancePID.reset();
        wallFollowPID.reset();
    }

    void resetPosition() {
        currentPosition = {0, 0, 0, 0, 0, millis()};
        targetPosition = currentPosition;
        pathMemoryIndex = 0;
    }

    float calculateDistanceToTarget() const {
        float dx = targetPosition.x - currentPosition.x;
        float dy = targetPosition.y - currentPosition.y;
        return sqrt(dx * dx + dy * dy);
    }

    float calculateTargetSpeed(float distance) const {
        if (distance < CAUTION_DISTANCE) {
            return map(distance, 0, CAUTION_DISTANCE, MIN_SPEED, MAX_SPEED);
        }
        return MAX_SPEED;
    }

    bool isObstacleDetected() const {
        return frontSensor.getDistance() < DANGER_DISTANCE ||
               leftSensor.getDistance() < DANGER_DISTANCE ||
               rightSensor.getDistance() < DANGER_DISTANCE;
    }

    int scanDirection(int angle) {
        motors.turn(angle);
        delay(100); // Allow readings to stabilize
        int distance = frontSensor.getDistance();
        motors.turn(-angle); // Return to original position
        return distance;
    }

    void updatePathMemory() {
        pathMemory[pathMemoryIndex] = currentPosition;
        pathMemoryIndex = (pathMemoryIndex + 1) % PATH_MEMORY_SIZE;
    }

    void updateMetrics() {
        // Basic metrics tracking
        static unsigned long lastMetricsUpdate = 0;
        unsigned long now = millis();
        
        if (now - lastMetricsUpdate >= 1000) { // Update every second
            // Update distance traveled
            float distance = calculateDistanceToTarget();
            
            // Update average speed
            float avgSpeed = currentPosition.velocity;
            
            // Could log or store these metrics as needed
            lastMetricsUpdate = now;
        }
    }

    void handleError() {
        motors.stop();
        currentState = ERROR_STATE;
        if (DEBUG_MODE) {
            Serial.println(F("Navigation error detected"));
        }
    }
};

#endif
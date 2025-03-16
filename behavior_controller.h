#ifndef BEHAVIOR_CONTROLLER_H
#define BEHAVIOR_CONTROLLER_H

#include "config.h"
#include "motors.h"
#include "sensor_manager.h"
#include "navigation.h"
#include "imu.h"

class BehaviorController {
private:
    Motors& motors;
    SensorManager& sensors;
    Navigation& navigation;
    IMU& imu;
    
    RobotState currentState;
    NavigationMode currentMode;
    ErrorCode currentError;
    
    struct BehaviorMetrics {
        unsigned long modeStartTime;
        int obstaclesAvoided;
        int recoveryAttempts;
        float totalDistanceTraveled;
        float averageSpeed;
        int errorCount;
    } metrics;
    
    unsigned long lastUpdate;
    const unsigned long UPDATE_INTERVAL = 100; // 100ms between updates

public:
    BehaviorController(Motors& m, SensorManager& s, Navigation& n, IMU& i)
        : motors(m), sensors(s), navigation(n), imu(i),
          currentState(IDLE), currentMode(NORMAL), currentError(NO_ERROR),
          lastUpdate(0) {
        resetMetrics();
    }
    
    void update() {
        if (millis() - lastUpdate < UPDATE_INTERVAL) return;
        
        // Update sensor data
        SensorManager::SensorData sensorData = sensors.update();
        
        // Check for emergency conditions
        if (checkEmergencyConditions()) {
            handleEmergency();
            return;
        }
        
        // Update navigation mode based on sensor data
        updateNavigationMode(sensorData);
        
        // Execute current behavior
        executeBehavior();
        
        // Update metrics
        updateMetrics();
        
        lastUpdate = millis();
    }
    
    RobotState getState() const { return currentState; }
    NavigationMode getMode() const { return currentMode; }
    ErrorCode getError() const { return currentError; }
    BehaviorMetrics getMetrics() const { return metrics; }

private:
    bool checkEmergencyConditions() {
        // Check battery
        if (battery.getVoltage() < CRITICAL_BATTERY_THRESHOLD) {
            currentError = BATTERY_LOW;
            return true;
        }
        
        // Check orientation
        float roll = imu.getRoll();
        float pitch = imu.getPitch();
        if (abs(roll) > MAX_SAFE_ROLL || abs(pitch) > MAX_SAFE_PITCH) {
            currentError = IMU_ERROR;
            return true;
        }
        
        // Check motor stall
        if (motors.isStalled()) {
            currentError = MOTOR_STALL;
            return true;
        }
        
        return false;
    }
    
    void handleEmergency() {
        motors.stop();
        currentState = EMERGENCY_STOP;
        metrics.errorCount++;
        
        // Log emergency
        if (DEBUG_MODE) {
            Serial.print(F("Emergency: "));
            Serial.println(getErrorString(currentError));
        }
    }
    
    void updateNavigationMode(const SensorManager::SensorData& sensorData) {
        NavigationMode newMode = currentMode;
        
        // Check for obstacles
        if (!sensors.isFrontClear()) {
            newMode = OBSTACLE_AVOIDANCE;
            metrics.obstaclesAvoided++;
        }
        // Check for wall following conditions
        else if (ENABLE_WALL_FOLLOWING && 
                (sensorData.leftDistance < WALL_FOLLOW_DISTANCE || 
                 sensorData.rightDistance < WALL_FOLLOW_DISTANCE)) {
            newMode = WALL_FOLLOWING;
        }
        // Check for position tracking
        else if (ENABLE_POSITION_TRACKING && 
                navigation.hasTarget()) {
            newMode = POSITION_TRACKING;
        }
        // Default to normal navigation
        else {
            newMode = NORMAL;
        }
        
        // Update mode if changed
        if (newMode != currentMode) {
            currentMode = newMode;
            metrics.modeStartTime = millis();
        }
    }
    
    void executeBehavior() {
        switch (currentMode) {
            case NORMAL:
                navigation.normalNavigation();
                currentState = MOVING;
                break;
                
            case OBSTACLE_AVOIDANCE:
                navigation.avoidObstacle();
                currentState = AVOIDING;
                break;
                
            case WALL_FOLLOWING:
                navigation.followWall();
                currentState = MOVING;
                break;
                
            case POSITION_TRACKING:
                navigation.navigateToPosition();
                currentState = MOVING;
                break;
                
            case RECOVERY:
                handleRecovery();
                break;
                
            default:
                motors.stop();
                currentState = ERROR_STATE;
                break;
        }
    }
    
    void handleRecovery() {
        if (metrics.recoveryAttempts >= RECOVERY_ATTEMPTS) {
            currentError = STUCK;
            currentState = ERROR_STATE;
            return;
        }
        
        navigation.performRecovery();
        metrics.recoveryAttempts++;
        currentState = MOVING;
    }
    
    void updateMetrics() {
        metrics.totalDistanceTraveled += motors.getAverageSpeed() * 
                                       (UPDATE_INTERVAL / 1000.0);
        metrics.averageSpeed = (metrics.averageSpeed * 0.9) + 
                             (motors.getAverageSpeed() * 0.1);
    }
    
    void resetMetrics() {
        metrics.modeStartTime = millis();
        metrics.obstaclesAvoided = 0;
        metrics.recoveryAttempts = 0;
        metrics.totalDistanceTraveled = 0;
        metrics.averageSpeed = 0;
        metrics.errorCount = 0;
    }
    
    const char* getErrorString(ErrorCode error) {
        switch (error) {
            case BATTERY_LOW: return "Battery Low";
            case MOTOR_STALL: return "Motor Stall";
            case SENSOR_ERROR: return "Sensor Error";
            case IMU_ERROR: return "IMU Error";
            case STUCK: return "Robot Stuck";
            case OVERHEATING: return "Overheating";
            case CALIBRATION_FAILED: return "Calibration Failed";
            default: return "Unknown Error";
        }
    }
};

#endif
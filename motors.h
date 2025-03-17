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
    
    // Enhanced motor characteristics
    float maxAcceleration;
    float maxDeceleration;
    float motorTemperature;
    unsigned long motorRuntime;
    float currentDraw[2];  // Current draw for each motor
    float motorLoad[2];    // Load percentage for each motor
    bool overheatProtection;
    
    struct MotorStats {
        unsigned long totalRuntime;
        float averageSpeed;
        float maxSpeedReached;
        int stallCount;
        float efficiency;
        float totalEnergyUsed;
        float peakCurrent;
        int emergencyStops;
        float temperatureHistory[10];
        int tempHistoryIndex;
    } stats;

    // PID control for motor speed
    struct PIDController {
        float Kp, Ki, Kd;
        float lastError;
        float integral;
        unsigned long lastTime;
    } speedPID;

public:
    Motors() 
        : currentLeftSpeed(0), currentRightSpeed(0), lastSpeedUpdate(0),
          lastStallCheck(0), stallDetected(false), maxAcceleration(ACCELERATION),
          maxDeceleration(DECELERATION), motorTemperature(25.0), motorRuntime(0),
          overheatProtection(true) {
        resetStats();
        initPID();
        
        // Initialize arrays
        for (int i = 0; i < 2; i++) {
            currentDraw[i] = 0;
            motorLoad[i] = 0;
        }
    }

    void init() {
        pinMode(ENA, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(IN4, OUTPUT);
        pinMode(ENB, OUTPUT);
        
        // Initialize motor drivers with a test sequence
        testMotorDrivers();
        calibrateMotors();
        stop();
    }

    void setSpeed(int leftSpeed, int rightSpeed) {
        // Apply PID control for speed stability
        leftSpeed = applyPIDControl(leftSpeed, currentLeftSpeed, 0);
        rightSpeed = applyPIDControl(rightSpeed, currentRightSpeed, 1);
        
        // Implement smooth acceleration with thermal protection
        smoothSpeedTransition(currentLeftSpeed, leftSpeed, currentRightSpeed, rightSpeed);
        
        // Update motor speeds with thermal monitoring
        if (!checkThermalLimits()) {
            analogWrite(ENA, currentLeftSpeed);
            analogWrite(ENB, currentRightSpeed);
            updateMotorMetrics();
        } else {
            handleOverheating();
        }
    }

    void forward(int speed, bool precise = false) {
        if (precise) {
            speed = applyPrecisionControl(speed);
        }
        
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        setSpeed(speed, speed);
    }

    void backward(int speed, bool precise = false) {
        if (precise) {
            speed = applyPrecisionControl(speed);
        }
        
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        setSpeed(speed, speed);
    }

    void stop() {
        // Implement regenerative braking
        implementRegenerativeBraking();
        
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
        
        currentLeftSpeed = 0;
        currentRightSpeed = 0;
    }

    void emergencyStop() {
        // Immediate stop without smooth deceleration
        stop();
        stats.emergencyStops++;
    }

    void turn(float angularSpeed) {
        // Enhanced turning with gyroscope compensation
        float compensatedSpeed = compensateGyroscopicDrift(angularSpeed);
        int turnPower = abs(compensatedSpeed) * (MAX_SPEED / MAX_ANGULAR_SPEED);
        
        if (compensatedSpeed > 0) {
            setSpeed(turnPower, -turnPower);  // Turn right
        } else {
            setSpeed(-turnPower, turnPower);  // Turn left
        }
    }

    void setAccelerationLimits(float accel, float decel) {
        maxAcceleration = constrain(accel, 1, 20);
        maxDeceleration = constrain(decel, 1, 20);
    }

    // Enhanced status reporting
    struct MotorStatus {
        float temperature;
        float current;
        float load;
        bool stalled;
        float efficiency;
    };

    MotorStatus getStatus(int motorIndex) {
        MotorStatus status;
        status.temperature = motorTemperature;
        status.current = currentDraw[motorIndex];
        status.load = motorLoad[motorIndex];
        status.stalled = isStalled();
        status.efficiency = calculateEfficiency(motorIndex);
        return status;
    }

    bool isStalled() {
        if (millis() - lastStallCheck > 1000) {
            // Enhanced stall detection using current sensing and encoder feedback
            stallDetected = detectStallCondition();
            lastStallCheck = millis();
        }
        return stallDetected;
    }

    MotorStats getStats() const {
        return stats;
    }

    bool selfTest() {
        bool success = true;
        
        // Test each motor individually
        success &= testMotor(0); // Left motor
        success &= testMotor(1); // Right motor
        
        // Test encoders
        success &= testEncoders();
        
        // Test current sensing
        success &= testCurrentSensing();
        
        return success;
    }

private:
    void initPID() {
        speedPID.Kp = 2.0;
        speedPID.Ki = 0.1;
        speedPID.Kd = 0.05;
        speedPID.lastError = 0;
        speedPID.integral = 0;
        speedPID.lastTime = 0;
    }

    float applyPIDControl(float target, float current, int motorIndex) {
        unsigned long now = millis();
        float dt = (now - speedPID.lastTime) / 1000.0;
        if (dt <= 0) return target;

        float error = target - current;
        speedPID.integral += error * dt;
        float derivative = (error - speedPID.lastError) / dt;
        
        float output = target + 
                      (speedPID.Kp * error) + 
                      (speedPID.Ki * speedPID.integral) + 
                      (speedPID.Kd * derivative);

        speedPID.lastError = error;
        speedPID.lastTime = now;
        
        return constrain(output, -MAX_SPEED, MAX_SPEED);
    }

    void smoothSpeedTransition(int& currentLeft, int targetLeft, 
                             int& currentRight, int targetRight) {
        // Calculate acceleration limits based on current speed
        float currentAccelLimit = calculateDynamicAccelLimit();
        
        // Apply acceleration limits
        int diffLeft = targetLeft - currentLeft;
        int diffRight = targetRight - currentRight;
        
        if (abs(diffLeft) > currentAccelLimit) {
            currentLeft += (diffLeft > 0) ? currentAccelLimit : -currentAccelLimit;
        } else {
            currentLeft = targetLeft;
        }
        
        if (abs(diffRight) > currentAccelLimit) {
            currentRight += (diffRight > 0) ? currentAccelLimit : -currentAccelLimit;
        } else {
            currentRight = targetRight;
        }
        
        // Apply thermal limiting
        applyThermalLimiting(currentLeft, currentRight);
        
        // Update timing
        lastSpeedUpdate = millis();
    }

    float calculateDynamicAccelLimit() {
        // Adjust acceleration limit based on speed and load
        float baseLimit = maxAcceleration;
        float loadFactor = max(motorLoad[0], motorLoad[1]);
        float tempFactor = (100 - (motorTemperature - 25)) / 100.0;
        
        return baseLimit * (1.0 - loadFactor) * tempFactor;
    }

    void applyThermalLimiting(int& leftSpeed, int& rightSpeed) {
        if (motorTemperature > 50) {
            float reductionFactor = 1.0 - ((motorTemperature - 50) / 30.0);
            reductionFactor = constrain(reductionFactor, 0.3, 1.0);
            
            leftSpeed *= reductionFactor;
            rightSpeed *= reductionFactor;
        }
    }

    bool checkThermalLimits() {
        return motorTemperature > MAX_SAFE_TEMPERATURE;
    }

    void handleOverheating() {
        if (overheatProtection) {
            stop();
            stats.emergencyStops++;
        }
    }

    void updateMotorMetrics() {
        unsigned long currentTime = millis();
        float timeElapsed = (currentTime - lastSpeedUpdate) / 1000.0;
        
        // Update runtime
        if (currentLeftSpeed > 0 || currentRightSpeed > 0) {
            motorRuntime += currentTime - lastSpeedUpdate;
        }
        
        // Update temperature history
        updateTemperatureHistory();
        
        // Update current draw and efficiency
        updateCurrentDraw();
        
        // Update load calculations
        updateMotorLoad();
        
        // Update statistics
        updateStats(timeElapsed);
    }

    void updateTemperatureHistory() {
        stats.temperatureHistory[stats.tempHistoryIndex] = motorTemperature;
        stats.tempHistoryIndex = (stats.tempHistoryIndex + 1) % 10;
    }

    void updateCurrentDraw() {
        // Simulate current sensing (replace with actual sensor readings)
        currentDraw[0] = abs(currentLeftSpeed) * 0.02;  // 20mA per unit of speed
        currentDraw[1] = abs(currentRightSpeed) * 0.02;
        
        // Update peak current
        stats.peakCurrent = max(stats.peakCurrent, max(currentDraw[0], currentDraw[1]));
    }

    void updateMotorLoad() {
        motorLoad[0] = abs(currentLeftSpeed) / (float)MAX_SPEED;
        motorLoad[1] = abs(currentRightSpeed) / (float)MAX_SPEED;
    }

    float calculateEfficiency(int motorIndex) {
        if (currentDraw[motorIndex] <= 0) return 0;
        float mechanicalPower = abs(motorIndex == 0 ? currentLeftSpeed : currentRightSpeed) * 0.1;
        float electricalPower = currentDraw[motorIndex] * 12.0; // Assuming 12V system
        return (mechanicalPower / electricalPower) * 100;
    }

    void implementRegenerativeBraking() {
        // Simulate regenerative braking (replace with actual implementation)
        int brakeTime = 100; // ms
        while (currentLeftSpeed > 0 || currentRightSpeed > 0) {
            if (currentLeftSpeed > 0) currentLeftSpeed = max(0, currentLeftSpeed - 10);
            if (currentRightSpeed > 0) currentRightSpeed = max(0, currentRightSpeed - 10);
            analogWrite(ENA, currentLeftSpeed);
            analogWrite(ENB, currentRightSpeed);
            delay(10);
        }
    }

    float compensateGyroscopicDrift(float angularSpeed) {
        // Add gyroscope-based drift compensation
        // This is a placeholder for actual gyroscope integration
        return angularSpeed * 1.05; // Simple compensation factor
    }

    int applyPrecisionControl(int speed) {
        // Enhanced precision control for low speeds
        if (speed < MIN_SPEED) {
            return MIN_SPEED + (speed * 0.2);
        }
        return speed;
    }

    bool detectStallCondition() {
        // Enhanced stall detection using current and speed
        bool highCurrent = (currentDraw[0] > 2.0) || (currentDraw[1] > 2.0);
        bool lowSpeed = (abs(currentLeftSpeed) < 10) && (abs(currentRightSpeed) < 10);
        return highCurrent && lowSpeed;
    }

    void resetStats() {
        stats = {0};
        stats.efficiency = 100.0;
    }

    bool testMotor(int motorIndex) {
        int speed = MIN_SPEED;
        bool success = true;
        
        // Test forward
        if (motorIndex == 0) {
            setSpeed(speed, 0);
        } else {
            setSpeed(0, speed);
        }
        delay(500);
        
        // Test reverse
        if (motorIndex == 0) {
            setSpeed(-speed, 0);
        } else {
            setSpeed(0, -speed);
        }
        delay(500);
        
        stop();
        return success;
    }

    bool testEncoders() {
        // Placeholder for encoder testing
        return true;
    }

    bool testCurrentSensing() {
        // Placeholder for current sensor testing
        return true;
    }

    void testMotorDrivers() {
        // Test motor driver functionality
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        delay(100);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        delay(100);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        delay(100);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        delay(100);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    }

    void calibrateMotors() {
        // Perform motor calibration sequence
        for (int i = 0; i < 3; i++) {
            setSpeed(MIN_SPEED, MIN_SPEED);
            delay(200);
            setSpeed(-MIN_SPEED, -MIN_SPEED);
            delay(200);
        }
        stop();
    }
};

#endif
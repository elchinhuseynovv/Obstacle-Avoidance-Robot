// Advanced Autonomous Robot Control System
#include "libraries.h"
#include "config.h"
#include "motors.h"
#include "sensor.h"
#include "display.h"
#include "battery.h"
#include "navigation.h"
#include "line_follower.h"
#include "encoder.h"
#include "imu.h"
#include "pid_controller.h"

// Object instances
Motors motors;
UltrasonicSensor sensor;
Display display;
BatteryMonitor battery;
Navigation* navigation;
LineFollower lineFollower(motors);
IMU imu;
Encoder rightEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B, WHEEL_DIAMETER, PULSES_PER_REV);
Encoder leftEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B, WHEEL_DIAMETER, PULSES_PER_REV);

// System state variables
bool systemError = false;
bool emergencyStop = false;
bool calibrationMode = false;
unsigned long lastStateUpdate = 0;
unsigned long lastTelemetryUpdate = 0;
unsigned long systemUptime = 0;
const char* navigationModeStrings[] = {"Normal", "Avoiding", "Recovery", "Line Follow", "Calibrating"};

// System metrics
struct SystemMetrics {
    float averageSpeed;
    float totalDistance;
    int obstaclesAvoided;
    int errorCount;
    float batteryDrainRate;
    float lastBatteryVoltage;
    unsigned long lastBatteryCheck;
} metrics;

// Interrupt handlers for encoders
void rightEncoderISR() {
    rightEncoder.handleInterrupt();
}

void leftEncoderISR() {
    leftEncoder.handleInterrupt();
}

// Emergency stop interrupt handler
void emergencyStopISR() {
    emergencyStop = true;
    motors.stop();
    display.showError("EMERGENCY STOP");
}

void setup() {
    // Initialize serial communication
    if (DEBUG_MODE) {
        Serial.begin(SERIAL_BAUD);
        Serial.println(F("Initializing Advanced Robot Control System..."));
    }
    
    // Initialize system metrics
    initializeMetrics();
    
    // Initialize components with error checking
    if (!initializeComponents()) {
        systemError = true;
        display.showError("Init Failed!");
        return;
    }
    
    // Initialize interrupts
    setupInterrupts();
    
    // Perform initial system checks
    if (!performSystemChecks()) {
        systemError = true;
        display.showError("Check Failed!");
        return;
    }
    
    // Enter calibration mode if needed
    if (digitalRead(CALIBRATION_PIN) == HIGH) {
        enterCalibrationMode();
    }
    
    if (DEBUG_MODE) {
        Serial.println(F("Initialization complete"));
        printSystemInfo();
    }
}

void loop() {
    // Update system uptime
    systemUptime = millis();
    
    // Handle emergency stop
    if (emergencyStop) {
        handleEmergencyStop();
        return;
    }
    
    // Update IMU and process motion data
    updateMotionControl();
    
    // Check battery and system health
    if (!checkSystemHealth()) {
        return;
    }
    
    // Main control loop
    if (!systemError) {
        // Handle different operating modes
        if (calibrationMode) {
            handleCalibrationMode();
        } else {
            handleNormalOperation();
        }
        
        // Update metrics and telemetry
        updateMetrics();
        sendTelemetryData();
    }
    
    // Maintain consistent loop timing
    maintainLoopTiming();
}

bool initializeComponents() {
    // Initialize with error checking
    bool success = true;
    
    success &= motors.init();
    success &= sensor.init();
    success &= display.init();
    success &= battery.init();
    success &= lineFollower.init();
    success &= imu.init();
    
    // Initialize encoders with error checking
    rightEncoder.init();
    leftEncoder.init();
    
    // Create navigation controller
    navigation = new Navigation(motors, sensor);
    
    return success;
}

void setupInterrupts() {
    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
    
    // Emergency stop interrupt
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencyStopISR, FALLING);
}

bool performSystemChecks() {
    bool checksPass = true;
    
    // Check battery voltage
    float voltage = battery.getVoltage();
    checksPass &= (voltage > MINIMUM_BATTERY_VOLTAGE);
    
    // Check sensors
    checksPass &= (sensor.selfTest());
    
    // Check motor connections
    checksPass &= (motors.selfTest());
    
    // Check IMU
    checksPass &= (imu.selfTest());
    
    return checksPass;
}

void handleEmergencyStop() {
    static unsigned long lastEmergencyMessage = 0;
    
    motors.stop();
    if (millis() - lastEmergencyMessage > 1000) {
        if (DEBUG_MODE) {
            Serial.println(F("EMERGENCY STOP ACTIVE"));
        }
        display.showError("EMERGENCY STOP");
        lastEmergencyMessage = millis();
    }
}

void updateMotionControl() {
    // Update IMU data
    imu.update();
    
    // Process motion data for enhanced control
    float roll, pitch, yaw;
    roll = imu.getRoll();
    pitch = imu.getPitch();
    yaw = imu.getYaw();
    
    // Adjust motor control based on orientation
    if (abs(roll) > MAX_SAFE_ROLL || abs(pitch) > MAX_SAFE_PITCH) {
        motors.stop();
        systemError = true;
        display.showError("Unsafe Angle!");
    }
}

bool checkSystemHealth() {
    // Check battery status
    float batteryVoltage = battery.getVoltage();
    if (battery.isLowBattery()) {
        handleLowBattery();
        return false;
    }
    
    // Monitor temperature
    if (imu.getTemperature() > MAX_SAFE_TEMPERATURE) {
        handleOverheating();
        return false;
    }
    
    // Check for motor stalls
    if (motors.isStalled()) {
        handleMotorStall();
        return false;
    }
    
    return true;
}

void handleNormalOperation() {
    // Update navigation
    navigation->update();
    
    // Update line follower if in line following mode
    if (navigation->getMode() == LINE_FOLLOWING) {
        lineFollower.update();
    }
    
    // Update display with current status
    NavigationMode currentMode = navigation->getMode();
    display.updateStatus(
        sensor.getDistance(),
        motors.getCurrentSpeed(),
        navigationModeStrings[currentMode]
    );
    
    // Process sensor data for obstacle detection
    processSensorData();
}

void handleCalibrationMode() {
    static unsigned long calibrationStart = 0;
    if (calibrationStart == 0) {
        calibrationStart = millis();
        display.showStatus("Calibrating...");
    }
    
    // Perform calibration sequence
    lineFollower.calibrate();
    imu.calibrate();
    
    // Exit calibration after timeout
    if (millis() - calibrationStart > CALIBRATION_TIMEOUT) {
        calibrationMode = false;
        display.showStatus("Cal Complete");
        delay(1000);
    }
}

void updateMetrics() {
    // Update system metrics
    metrics.totalDistance += (rightEncoder.getDistance() + leftEncoder.getDistance()) / 2;
    metrics.averageSpeed = (rightEncoder.getSpeed() + leftEncoder.getSpeed()) / 2;
    
    // Calculate battery drain rate
    if (millis() - metrics.lastBatteryCheck > BATTERY_CHECK_MS) {
        float currentVoltage = battery.getVoltage();
        metrics.batteryDrainRate = (metrics.lastBatteryVoltage - currentVoltage) * 
                                 (3600000.0 / (millis() - metrics.lastBatteryCheck)); // mV/hour
        metrics.lastBatteryVoltage = currentVoltage;
        metrics.lastBatteryCheck = millis();
    }
}

void sendTelemetryData() {
    if (millis() - lastTelemetryUpdate > TELEMETRY_UPDATE_MS) {
        if (DEBUG_MODE) {
            printDebugInfo();
        }
        // Send telemetry data over wireless if available
        sendWirelessTelemetry();
        lastTelemetryUpdate = millis();
    }
}

void printDebugInfo() {
    Serial.println(F("\n=== Robot Status Update ==="));
    Serial.print(F("Uptime: "));
    Serial.print(systemUptime / 1000);
    Serial.println(F(" seconds"));
    
    Serial.print(F("Battery: "));
    Serial.print(battery.getVoltage());
    Serial.println(F("V"));
    
    Serial.print(F("Distance Traveled: "));
    Serial.print(metrics.totalDistance);
    Serial.println(F("mm"));
    
    Serial.print(F("Average Speed: "));
    Serial.print(metrics.averageSpeed);
    Serial.println(F("mm/s"));
    
    Serial.print(F("Obstacles Avoided: "));
    Serial.println(metrics.obstaclesAvoided);
    
    Serial.print(F("Error Count: "));
    Serial.println(metrics.errorCount);
    
    Serial.print(F("Battery Drain Rate: "));
    Serial.print(metrics.batteryDrainRate);
    Serial.println(F("mV/hour"));
    
    // Print IMU data
    float roll, pitch, yaw;
    roll = imu.getRoll();
    pitch = imu.getPitch();
    yaw = imu.getYaw();
    
    Serial.println(F("\nOrientation:"));
    Serial.print(F("Roll: "));
    Serial.print(roll);
    Serial.print(F(" Pitch: "));
    Serial.print(pitch);
    Serial.print(F(" Yaw: "));
    Serial.println(yaw);
    
    Serial.println(F("=====================\n"));
}

void maintainLoopTiming() {
    static unsigned long lastLoopTime = 0;
    unsigned long loopTime = micros() - lastLoopTime;
    
    // Maintain consistent loop timing
    if (loopTime < LOOP_TIME_US) {
        delayMicroseconds(LOOP_TIME_US - loopTime);
    }
    
    lastLoopTime = micros();
}

void initializeMetrics() {
    metrics.averageSpeed = 0;
    metrics.totalDistance = 0;
    metrics.obstaclesAvoided = 0;
    metrics.errorCount = 0;
    metrics.batteryDrainRate = 0;
    metrics.lastBatteryVoltage = battery.getVoltage();
    metrics.lastBatteryCheck = millis();
}

void handleLowBattery() {
    motors.stop();
    display.showError("Low Battery!");
    metrics.errorCount++;
    if (DEBUG_MODE) {
        Serial.println(F("ERROR: Low battery! Robot stopped."));
    }
    systemError = true;
}

void handleOverheating() {
    motors.stop();
    display.showError("Overheating!");
    metrics.errorCount++;
    if (DEBUG_MODE) {
        Serial.println(F("ERROR: System overheating! Robot stopped."));
    }
    systemError = true;
}

void handleMotorStall() {
    motors.stop();
    display.showError("Motor Stall!");
    metrics.errorCount++;
    if (DEBUG_MODE) {
        Serial.println(F("ERROR: Motor stall detected! Robot stopped."));
    }
    systemError = true;
}

void processSensorData() {
    static unsigned long lastObstacleTime = 0;
    int distance = sensor.getDistance();
    
    if (distance < DANGER_DISTANCE && millis() - lastObstacleTime > OBSTACLE_TIMEOUT) {
        metrics.obstaclesAvoided++;
        lastObstacleTime = millis();
    }
}

void sendWirelessTelemetry() {
    // Implementation for wireless telemetry if available
    #ifdef WIRELESS_ENABLED
        // Send telemetry data wirelessly
        // This is a placeholder for wireless communication implementation
    #endif
}
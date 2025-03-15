// Advanced Obstacle Avoidance Robot with Enhanced Features
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

// System state
bool systemError = false;
const char* navigationModeStrings[] = {"Normal", "Avoiding", "Recovery", "Line Follow"};

// Interrupt handlers for encoders
void rightEncoderISR() {
    rightEncoder.handleInterrupt();
}

void leftEncoderISR() {
    leftEncoder.handleInterrupt();
}

void setup() {
    // Initialize serial communication
    if (DEBUG_MODE) {
        Serial.begin(SERIAL_BAUD);
        Serial.println("Initializing robot systems...");
    }
    
    // Initialize components
    motors.init();
    sensor.init();
    display.init();
    battery.init();
    lineFollower.init();
    imu.init();
    
    // Initialize encoders with interrupts
    rightEncoder.init();
    leftEncoder.init();
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
    
    // Create navigation controller
    navigation = new Navigation(motors, sensor);
    
    if (DEBUG_MODE) {
        Serial.println("Robot initialization complete");
        printSystemInfo();
    }
}

void loop() {
    // Update IMU data
    imu.update();
    
    // Check battery status
    float batteryVoltage = battery.getVoltage();
    if (battery.isLowBattery()) {
        handleLowBattery();
        return;
    }
    
    // Main control loop
    if (!systemError) {
        // Update navigation
        navigation->update();
        
        // Update line follower if in line following mode
        if (navigation->getMode() == LINE_FOLLOWING) {
            lineFollower.update();
        }
        
        // Update display
        NavigationMode currentMode = navigation->getMode();
        display.updateStatus(
            sensor.getDistance(),
            motors.getCurrentSpeed(),
            navigationModeStrings[currentMode]
        );
        
        // Debug output
        if (DEBUG_MODE) {
            printDebugInfo(batteryVoltage, currentMode);
        }
    }
    
    // Small delay to prevent sensor readings from being too frequent
    delay(10);
}

void handleLowBattery() {
    motors.stop();
    display.showError("Low Battery!");
    if (DEBUG_MODE) {
        Serial.println("ERROR: Low battery! Robot stopped.");
    }
    systemError = true;
    delay(1000);
}

void printSystemInfo() {
    Serial.println("\n=== System Information ===");
    Serial.println("Hardware Configuration:");
    Serial.println("- Motors: L298N Dual H-Bridge");
    Serial.println("- Distance Sensor: HC-SR04");
    Serial.println("- IMU: MPU6050");
    Serial.println("- Display: 16x2 LCD");
    Serial.println("- Line Sensors: 5 Channel Array");
    Serial.println("- Encoders: Quadrature");
    Serial.println("======================\n");
}

void printDebugInfo(float voltage, NavigationMode mode) {
    static unsigned long lastDebugOutput = 0;
    if (millis() - lastDebugOutput > DEBUG_UPDATE_MS) {
        Serial.println("\n--- Robot Status ---");
        Serial.print("Battery: ");
        Serial.print(voltage);
        Serial.println("V");
        Serial.print("Mode: ");
        Serial.println(navigationModeStrings[mode]);
        Serial.print("Speed: ");
        Serial.println(motors.getCurrentSpeed());
        
        // Print encoder data
        Serial.print("Right Distance: ");
        Serial.print(rightEncoder.getDistance());
        Serial.println(" mm");
        Serial.print("Left Distance: ");
        Serial.print(leftEncoder.getDistance());
        Serial.println(" mm");
        
        // Print IMU data
        Serial.print("Orientation (Roll,Pitch,Yaw): ");
        Serial.print(imu.getRoll());
        Serial.print(", ");
        Serial.print(imu.getPitch());
        Serial.print(", ");
        Serial.println(imu.getYaw());
        
        Serial.println("-----------------\n");
        lastDebugOutput = millis();
    }
}
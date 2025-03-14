// Advanced Obstacle Avoidance Robot with Enhanced Features
#include "libraries.h"
#include "config.h"
#include "motors.h"
#include "sensor.h"
#include "display.h"
#include "battery.h"
#include "navigation.h"

// Object instances
Motors motors;
UltrasonicSensor sensor;
Display display;
BatteryMonitor battery;
Navigation* navigation;

// System state
bool systemError = false;
const char* navigationModeStrings[] = {"Normal", "Avoiding", "Recovery"};

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
    
    // Create navigation controller
    navigation = new Navigation(motors, sensor);
    
    if (DEBUG_MODE) {
        Serial.println("Robot initialization complete");
    }
}

void loop() {
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
        Serial.println("-----------------\n");
        lastDebugOutput = millis();
    }
}
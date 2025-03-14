// Enhanced Configuration file for Advanced Obstacle Avoidance Robot
#ifndef CONFIG_H
#define CONFIG_H

// Motor Driver Pin Configuration
#define ENA 9  // Enable motor A (Right)
#define IN1 8  // Motor A input 1
#define IN2 7  // Motor A input 2
#define IN3 6  // Motor B input 1
#define IN4 5  // Motor B input 2
#define ENB 10 // Enable motor B (Left)

// Ultrasonic Sensor Pin Configuration
#define TRIG 3
#define ECHO 2

// Battery Monitoring
#define BATTERY_PIN A0
#define VOLTAGE_REFERENCE 5.0
#define VOLTAGE_DIVIDER 2.0
#define LOW_BATTERY_THRESHOLD 7.2
#define BATTERY_CHECK_MS 5000

// LCD Display Configuration
#define LCD_ADDRESS 0x27
#define LCD_UPDATE_MS 500

// Navigation Parameters
#define SAFE_DISTANCE 30     // Distance in cm for safe navigation
#define CAUTION_DISTANCE 20  // Distance in cm to start slowing down
#define DANGER_DISTANCE 10   // Distance in cm to take immediate action
#define MAX_CONSECUTIVE_OBSTACLES 5
#define OBSTACLE_TIMEOUT 2000
#define REVERSE_TIME_MS 1000
#define RECOVERY_REVERSE_TIME 2000

// Motor Speed Parameters
#define MAX_SPEED 255        // Maximum motor speed (0-255)
#define MIN_SPEED 100        // Minimum motor speed for reliable movement
#define TURN_SPEED 180       // Speed during turns
#define ACCELERATION 5       // Speed increment for smooth acceleration

// Debug Configuration
#define DEBUG_MODE true      // Enable/disable debug messages
#define SERIAL_BAUD 9600     // Serial communication baud rate

// Timing Parameters
#define TURN_DELAY_MS 6      // Milliseconds per degree of turning
#define SPEED_UPDATE_MS 20   // Delay between speed updates
#define DEBUG_UPDATE_MS 500  // Delay between debug messages
#define SENSOR_TIMEOUT 30000 // Ultrasonic sensor timeout in microseconds

// Navigation Modes
enum NavigationMode {
    NORMAL,
    OBSTACLE_AVOIDANCE,
    RECOVERY
};

#endif
// Enhanced Configuration file for Advanced Robot Control System
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
#define TRIG_FRONT 3
#define ECHO_FRONT 2
#define TRIG_LEFT 14
#define ECHO_LEFT 15
#define TRIG_RIGHT 16
#define ECHO_RIGHT 17

// Line Follower Sensor Pins
#define LINE_SENSOR_1 A1
#define LINE_SENSOR_2 A2
#define LINE_SENSOR_3 A3
#define LINE_SENSOR_4 A4
#define LINE_SENSOR_5 A5

// Encoder Pins
#define RIGHT_ENCODER_A 11
#define RIGHT_ENCODER_B 12
#define LEFT_ENCODER_A 13
#define LEFT_ENCODER_B 4

// Battery Monitoring
#define BATTERY_PIN A0
#define VOLTAGE_REFERENCE 5.0
#define VOLTAGE_DIVIDER 2.0
#define LOW_BATTERY_THRESHOLD 7.2
#define CRITICAL_BATTERY_THRESHOLD 6.8
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
#define DECELERATION 8       // Speed decrement for smooth stopping

// Line Following PID Parameters
#define LINE_KP 0.8         // Proportional gain
#define LINE_KI 0.2         // Integral gain
#define LINE_KD 0.1         // Derivative gain
#define LINE_SAMPLE_TIME 20  // PID sample time in ms

// Encoder Parameters
#define WHEEL_DIAMETER 65.0  // mm
#define PULSES_PER_REV 20   // Encoder pulses per revolution
#define SPEED_CALC_INTERVAL 100 // ms
#define WHEEL_BASE 150.0    // Distance between wheels in mm

// IMU Parameters
#define IMU_UPDATE_INTERVAL 20 // ms
#define IMU_CALIBRATION_SAMPLES 100
#define GYRO_SENSITIVITY 131.0 // LSB/(deg/s)
#define ACCEL_SENSITIVITY 16384.0 // LSB/g

// Debug Configuration
#define DEBUG_MODE true      // Enable/disable debug messages
#define SERIAL_BAUD 115200   // Serial communication baud rate

// Timing Parameters
#define TURN_DELAY_MS 6      // Milliseconds per degree of turning
#define SPEED_UPDATE_MS 20   // Delay between speed updates
#define DEBUG_UPDATE_MS 500  // Delay between debug messages
#define SENSOR_TIMEOUT 30000 // Ultrasonic sensor timeout in microseconds

// Emergency Stop Configuration
#define EMERGENCY_STOP_PIN 18
#define CALIBRATION_PIN 19

// System Safety Thresholds
#define MAX_SAFE_TEMPERATURE 60.0  // Celsius
#define MAX_SAFE_ROLL 45.0        // Degrees
#define MAX_SAFE_PITCH 45.0       // Degrees
#define MINIMUM_BATTERY_VOLTAGE 6.8

// Advanced Navigation Parameters
#define PATH_MEMORY_SIZE 10  // Remember last N positions
#define MIN_TURN_RADIUS 100  // Minimum turning radius in mm
#define MAX_ANGULAR_SPEED 180 // Maximum angular speed in degrees/second
#define POSITION_TOLERANCE 5.0 // Position tolerance in mm

// Behavior Parameters
#define OBSTACLE_AVOID_THRESHOLD 3  // Number of consecutive obstacles before changing strategy
#define STUCK_TIMEOUT 5000         // Time in ms before considering robot stuck
#define RECOVERY_ATTEMPTS 3        // Maximum recovery attempts before seeking help
#define WALL_FOLLOW_DISTANCE 15    // Distance to maintain from wall in cm

// Feature Flags
#define ENABLE_WALL_FOLLOWING
#define ENABLE_POSITION_TRACKING
#define ENABLE_ADVANCED_NAVIGATION
//#define WIRELESS_ENABLED

// Navigation Modes
enum NavigationMode {
    NORMAL,
    OBSTACLE_AVOIDANCE,
    WALL_FOLLOWING,
    RECOVERY,
    LINE_FOLLOWING,
    POSITION_TRACKING,
    CALIBRATING,
    ERROR
};

// Robot States
enum RobotState {
    IDLE,
    MOVING,
    TURNING,
    AVOIDING,
    CALIBRATING,
    ERROR_STATE,
    EMERGENCY_STOP
};

// Error Codes
enum ErrorCode {
    NO_ERROR = 0,
    BATTERY_LOW = 1,
    MOTOR_STALL = 2,
    SENSOR_ERROR = 3,
    IMU_ERROR = 4,
    STUCK = 5,
    OVERHEATING = 6,
    CALIBRATION_FAILED = 7
};

#endif
# Author

Elchin Huseynov

# Advanced Obstacle Avoidance Robot

An intelligent robot capable of autonomous navigation and obstacle avoidance using ultrasonic sensors and dynamic speed control.

## Features

- Variable speed control based on obstacle distance
- Smart turning decisions based on obstacle detection
- Smooth acceleration and deceleration
- Safety features and emergency stops
- Debug mode with Serial monitoring
- Object-oriented design with modular components

## Hardware Requirements

- Arduino board (Uno/Nano recommended)
- L298N Motor Driver Module
- HC-SR04 Ultrasonic Sensor
- 2x DC Motors with wheels
- 9V or 12V battery pack
- Chassis and mounting hardware

## Pin Configuration

### Motor Driver
- ENA (Right Motor Enable) -> Pin 9
- IN1 (Right Motor Input 1) -> Pin 8
- IN2 (Right Motor Input 2) -> Pin 7
- IN3 (Left Motor Input 1) -> Pin 6
- IN4 (Left Motor Input 2) -> Pin 5
- ENB (Left Motor Enable) -> Pin 10

### Ultrasonic Sensor
- TRIG -> Pin 3
- ECHO -> Pin 2

## Circuit Diagram

```
                           Arduino
                     +---------------+
                     |               |
     HC-SR04        |               |      L298N
+-------------+     |               |    +---------+
|  TRIG  ECHO |     |               |    | ENA IN1 |
|   |     |   |     |               |    |  |   |  |
|   3     2   |     |               |    |  9   8  |
+-------------+     |               |    |         |
                    |               |    | IN2  IN3 |
                    |               |    |  7   6   |
                    |               |    |         |
                    |               |    | IN4 ENB |
                    |               |    |  5   10 |
                    |               |    |         |
                    +---------------+    +---------+
```

## Project Structure

- `main.ino`: Main program file
- `config.h`: Configuration and pin definitions
- `motors.h`: Motor control class
- `sensor.h`: Ultrasonic sensor class
- `README.md`: Project documentation

## Configuration

All configurable parameters are in `config.h`:
- Pin assignments
- Distance thresholds
- Speed settings
- Debug options
- Timing parameters

## Features Explained

### Speed Control
- MAX_SPEED (255): Full speed for clear paths
- MIN_SPEED (100): Minimum speed for reliable movement
- TURN_SPEED (180): Optimal speed for turning
- Dynamic speed adjustment based on obstacle distance

### Safety Features
- Emergency stop when obstacles are too close
- Smooth deceleration to prevent sudden stops
- Invalid sensor reading detection
- Timeout for sensor readings

### Navigation
- Smart turning decisions based on obstacle patterns
- Alternate turning directions when stuck
- Smooth acceleration/deceleration
- Variable speed based on obstacle distance

### Debug Mode
- Real-time distance measurements
- Status messages
- Error reporting
- Performance monitoring

## Usage

1. Connect the hardware according to the pin configuration
2. Upload the code to your Arduino
3. Power on the robot
4. The robot will automatically:
   - Navigate autonomously
   - Avoid obstacles
   - Adjust speed based on surroundings
   - Make smart turning decisions

## Customization

Modify `config.h` to adjust:
- Speed settings
- Distance thresholds
- Debug options
- Pin assignments
- Timing parameters
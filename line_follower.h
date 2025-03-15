#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "config.h"
#include "motors.h"
#include "pid_controller.h"

class LineFollower {
private:
    const int numSensors = 5;
    int sensorPins[5];
    Motors& motors;
    PIDController pid;
    bool isEnabled;

public:
    LineFollower(Motors& m) 
        : motors(m), 
          pid(LINE_KP, LINE_KI, LINE_KD, 0, -MAX_SPEED, MAX_SPEED),
          isEnabled(false) {
        sensorPins[0] = LINE_SENSOR_1;
        sensorPins[1] = LINE_SENSOR_2;
        sensorPins[2] = LINE_SENSOR_3;
        sensorPins[3] = LINE_SENSOR_4;
        sensorPins[4] = LINE_SENSOR_5;
    }

    void init() {
        for (int i = 0; i < numSensors; i++) {
            pinMode(sensorPins[i], INPUT);
        }
    }

    void enable() {
        isEnabled = true;
        pid.reset();
    }

    void disable() {
        isEnabled = false;
        motors.stop();
    }

    void update() {
        if (!isEnabled) return;

        float position = calculatePosition();
        float correction = pid.compute(position);

        int leftSpeed = MAX_SPEED - correction;
        int rightSpeed = MAX_SPEED + correction;

        leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
        rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

        motors.setSpeed(leftSpeed, rightSpeed);
    }

private:
    float calculatePosition() {
        int sensorValues[5];
        int sum = 0;
        int weightedSum = 0;

        for (int i = 0; i < numSensors; i++) {
            sensorValues[i] = digitalRead(sensorPins[i]);
            sum += sensorValues[i];
            weightedSum += sensorValues[i] * (i * 1000);
        }

        if (sum == 0) return 0;
        return (weightedSum / sum) - 2000; // Center position = 0
    }
};

#endif
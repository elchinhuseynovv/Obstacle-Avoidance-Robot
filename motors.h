#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

class Motors {
private:
    int currentSpeed;
    unsigned long lastSpeedChange;

public:
    Motors() : currentSpeed(0), lastSpeedChange(0) {}

    void init() {
        pinMode(ENA, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(IN4, OUTPUT);
        pinMode(ENB, OUTPUT);
        stop();
    }

    void setSpeed(int rightSpeed, int leftSpeed) {
        analogWrite(ENA, rightSpeed);
        analogWrite(ENB, leftSpeed);
    }

    void forward(int speed) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        setSpeed(speed, speed);
        currentSpeed = speed;
    }

    void backward(int speed) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        setSpeed(speed, speed);
        currentSpeed = speed;
    }

    void stop() {
        // Gradual deceleration
        while (currentSpeed > 0) {
            currentSpeed -= ACCELERATION;
            if (currentSpeed < 0) currentSpeed = 0;
            setSpeed(currentSpeed, currentSpeed);
            delay(SPEED_UPDATE_MS);
        }
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    }

    void turnRight(int angle = 90) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        setSpeed(TURN_SPEED, TURN_SPEED);
        delay(angle * TURN_DELAY_MS);
    }

    void turnLeft(int angle = 90) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        setSpeed(TURN_SPEED, TURN_SPEED);
        delay(angle * TURN_DELAY_MS);
    }

    int getCurrentSpeed() {
        return currentSpeed;
    }

    void updateSpeed(int targetSpeed) {
        unsigned long currentTime = millis();
        if (currentTime - lastSpeedChange > SPEED_UPDATE_MS) {
            if (currentSpeed < targetSpeed) {
                currentSpeed += ACCELERATION;
                if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
            } else if (currentSpeed > targetSpeed) {
                currentSpeed -= ACCELERATION;
                if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
            }
            lastSpeedChange = currentTime;
        }
    }
};

#endif
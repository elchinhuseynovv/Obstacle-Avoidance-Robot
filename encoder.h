#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"

class Encoder {
private:
    int pinA;
    int pinB;
    volatile long count;
    float wheelDiameter;
    int pulsesPerRevolution;

public:
    Encoder(int pA, int pB, float diameter, int ppr) 
        : pinA(pA), pinB(pB), count(0), 
          wheelDiameter(diameter), pulsesPerRevolution(ppr) {}

    void init() {
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
    }

    void handleInterrupt() {
        if (digitalRead(pinB) == HIGH) {
            count++;
        } else {
            count--;
        }
    }

    long getCount() {
        return count;
    }

    void reset() {
        count = 0;
    }

    float getDistance() {
        float revolutions = (float)count / pulsesPerRevolution;
        return PI * wheelDiameter * revolutions;
    }

    float getSpeed() {
        static unsigned long lastTime = 0;
        static long lastCount = 0;
        unsigned long currentTime = millis();
        
        if (currentTime - lastTime >= SPEED_CALC_INTERVAL) {
            float deltaTime = (currentTime - lastTime) / 1000.0;
            float countDiff = count - lastCount;
            float revolutions = countDiff / pulsesPerRevolution;
            float distance = PI * wheelDiameter * revolutions;
            float speed = distance / deltaTime;
            
            lastTime = currentTime;
            lastCount = count;
            
            return speed;
        }
        return 0;
    }
};

#endif
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    float kp, ki, kd;
    float previousError;
    float integral;
    unsigned long lastTime;
    float targetValue;
    float maxOutput;
    float minOutput;

public:
    PIDController(float p, float i, float d, float target, float min = -255, float max = 255)
        : kp(p), ki(i), kd(d), previousError(0), integral(0), lastTime(0),
          targetValue(target), maxOutput(max), minOutput(min) {}

    void setTarget(float target) {
        targetValue = target;
    }

    float compute(float currentValue) {
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0;
        
        if (lastTime == 0) {
            lastTime = currentTime;
            return 0;
        }

        float error = targetValue - currentValue;
        integral += error * deltaTime;
        float derivative = (error - previousError) / deltaTime;

        float output = kp * error + ki * integral + kd * derivative;
        output = constrain(output, minOutput, maxOutput);

        previousError = error;
        lastTime = currentTime;

        return output;
    }

    void reset() {
        previousError = 0;
        integral = 0;
        lastTime = 0;
    }
};

#endif
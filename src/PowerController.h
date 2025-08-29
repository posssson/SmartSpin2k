#pragma once
#include <algorithm>
#include <cmath>

// Simple PID controller to follow wanted watts
class PowerPID {
public:
    PowerPID(float kp = 0.5, float ki = 0.05, float kd = 0.1, float tolerancePct = 0.02)
        : Kp(kp), Ki(ki), Kd(kd), tolerancePercent(tolerancePct), prevError(0), integral(0) {}

    // Update function takes target (wanted) and actual measured power
    float update(float target, float actual) {
        float error = target - actual;
        integral += error;
        float derivative = error - prevError;
        prevError = error;

        return Kp * error + Ki * integral + Kd * derivative;
    }

    // Check if within tolerance band
    bool withinTolerance(float target, float actual) const {
        float tolerance = std::max(10.0f, target * tolerancePercent); // adaptive: at least 10W or 5%
        return std::fabs(actual - target) <= tolerance;
    }

private:
    float Kp, Ki, Kd;
    float tolerancePercent;
    float prevError;
    float integral;
};
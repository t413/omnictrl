#include "pid.h"
#include <Arduino.h>

PIDCtrl::PIDCtrl(float kp, float ki, float kd, float lim, float rampLim) : P(kp), I(ki), D(kd), limit(lim), rampLimit(rampLim) {
    reset();
}

void PIDCtrl::reset() {
    prevErr = 0;
    prevOut = 0;
    prevInt = 0;
    prevTime = 0;
}

float PIDCtrl::update(unsigned long now, float error, float integralError) {
    float dt = (now - prevTime) / 1000.0;
    prevTime = now;

    float Pout = P * error * tuneScale; // proportional term

    if (integralError == NOT_SET)
        integralError = error; // just use regular error
    float Iout = prevInt + I * tuneScale * dt * (integralError + prevErr) / 2;
    Iout = constrain(Iout, -limit, limit); // prevent integral windup
    float Dout = D * tuneScale * (error - prevErr) / dt;
    float out = constrain(Pout + Iout + Dout, -limit, limit);

    if (rampLimit > 0) {
        float maxChange = rampLimit * dt;
        out = constrain(out, prevOut - maxChange, prevOut + maxChange);
    }
    prevErr = integralError;
    prevOut = out;
    prevInt = Iout;

    return out;
}

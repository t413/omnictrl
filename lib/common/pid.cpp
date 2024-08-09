#include "pid.h"
#include <Arduino.h>

PIDCtrl::PIDCtrl(float kp, float ki, float kd, float lim) : P(kp), I(ki), D(kd), limit(lim) {
    reset();
}

void PIDCtrl::reset() {
    prevErr = 0;
    prevOut = 0;
    prevInt = 0;
    prevTime = 0;
}

float PIDCtrl::update(float error) {
    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0;
    prevTime = now;

    float Pout = P * error;
    float Iout = prevInt + I * dt * (error + prevErr) / 2;
    Iout = constrain(Iout, -limit, limit); // prevent integral windup
    float Dout = D * (error - prevErr) / dt;
    float out = constrain(Pout + Iout + Dout, -limit, limit);

    prevErr = error;
    prevOut = out;
    prevInt = Iout;

    return out;
}

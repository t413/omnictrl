#pragma once

class PIDCtrl {
public:
    PIDCtrl(float P, float I, float D, float limit);
    ~PIDCtrl() = default;

    float update(float error);
    void reset();

    float P = 0;
    float I = 0;
    float D = 0;
    float limit = 0;

protected:
    float prevErr = 0;
    float prevOut = 0;
    float prevInt = 0;
    unsigned long prevTime = 0;

};


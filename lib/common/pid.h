#pragma once

constexpr float NOT_SET = -123456.0;

class PIDCtrl {
public:
    PIDCtrl(float P, float I, float D, float limit);
    ~PIDCtrl() = default;

    float update(unsigned long now, float error, float integralError = NOT_SET);
    void reset();
    float getPrevOut() const { return prevOut; }

    float P = 0;
    float I = 0;
    float D = 0;
    float limit = 0;
    float tuneScale = 0.1; //scale factor for tuning

protected:
    float prevErr = 0;
    float prevOut = 0;
    float prevInt = 0;
    unsigned long prevTime = 0;

};


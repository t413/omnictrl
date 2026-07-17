// lib/behaviors/behavior.h
#pragma once
#include <stdint.h>
#include <vector>
#include <math.h>
#include "rfprotocol.h"

class Controller;
class DynamicsBase;
namespace behavior {
struct Manager;

struct Control {
    MotionControl m;
    float d_fwd = 0, d_side = 0, d_yaw = 0; //offsets in motor speed terms
    float pitchOffsetDeg = 0;
    Control() { }
    Control(MotionControl in) : m(in) { }
};

// Base for any behavior
struct Behavior {
    virtual ~Behavior() = default;
    static float getIntensity(const MotionControl& m);
    virtual const char* getName() const = 0;
    virtual Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr) = 0;
};

struct Oscillator {
    float lastTime_ = 0.0f;
    float phase_ = 0.0f;
    float freqMin_, freqMax_;
    float ampMin_, ampMax_;

    Oscillator(float freqMin = 0.1f, float freqMax = 5.0, float ampMin = 0.3, float ampMax = 1.2) :
        freqMin_(freqMin), freqMax_(freqMax), ampMin_(ampMin), ampMax_(ampMax) { }

    // Returns scaled amplitude * sin(phase)
    // intensity: [0..1], higher = faster & bigger
    // dt: delta time in seconds
    float update(float intensity, float timeSeconds);
    // Returns cos(phase) at current phase, scaled by amplitude
    float getCos(float intensity) const;
};




// ------------- //
//   Behaviors   //
// ------------- //

struct Happy : Behavior {
public:
    Oscillator osc_ = Oscillator(1.2f, 7.0f, 2.6, 0.5f);  // freqMin, freqMax, ampMin, ampMax
    float frequency_ = 4.3f;   // Hz
    float radius_ = 1.0f;   // circle radius on fwd axis

    virtual const char* getName() const { return "Happy"; }
    Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr) override;
};

struct Excited : public Behavior {
    Oscillator osc_ = Oscillator(2.0f, 8.0f, 2.0f, 1.0f);  // freqMin, freqMax, ampMin, ampMax
    virtual const char* getName() const { return "Excited"; }
    Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr) override;
};

struct Scared : public Behavior {
    Oscillator osc_ = Oscillator(2.0f, 7.0f, 2.5f, 1.2f);  // freqMin, freqMax, ampMin, ampMax
    virtual const char* getName() const { return "Scared"; }
    Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr) override;
};

struct Drunk : public Behavior {
    MotionControl filtered_;
    float alpha_ = 0.04;
    virtual const char* getName() const { return "Drunk"; }
    Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr) override;
};

// ------------- //
//    Manager    //
// ------------- //

struct Manager {
    std::vector<Behavior*> behaviors_;
    int8_t activeIdx_ = -1;

    Manager();
    void increment();
    void clear();
    bool isActive() const;

    virtual const char* getName() const;
    Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr);
};

} // namespace behavior

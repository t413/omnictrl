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
    Control(MotionControl in) : m(in) { }
};

// Base for any behavior
struct Behavior {
    uint32_t startTime_ = 0;
    virtual ~Behavior() = default;
    virtual Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr) = 0;
};



// ------------- //
//   Behaviors   //
// ------------- //

struct Happy : Behavior {
public:
    float frequency_ = 4.3f;   // Hz
    float radius_ = 1.0f;   // circle radius on fwd axis

    Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr) override;
};

struct Excited : public Behavior {
    float frequency_ = 5.0f;   // Hz
    float amplitude_ = 0.8f;   // yaw shake amplitude [-1..1]

    Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr) override;
};

struct Scared : public Behavior {
    float frequency_ = 5.0f;
    float amplitude_ = 0.9f;
    Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr) override;
};

struct Drunk : public Behavior {
    MotionControl filtered_;
    float alpha_ = 0.04;
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

    Control iterate(uint32_t now, const MotionControl&, bool isBalancing, Manager& mgr);
};

} // namespace behavior

#include "behavior.h"
#include "utils.h"
#include "log.h"

namespace behavior {

float Oscillator::update(float intensity, float ts) {
    float dt = ts - lastTime_;
    lastTime_ = ts;
    intensity = constrain(intensity, 0.0f, 1.0f);
    float freq = freqMin_ + (freqMax_ - freqMin_) * intensity;
    float amp = ampMin_ + (ampMax_ - ampMin_) * intensity;

    float omega = 2.0f * M_PI * freq;
    phase_ += omega * dt;
    while (phase_ > 2.0f * M_PI) phase_ -= 2.0f * M_PI;

    return amp * sinf(phase_);
}

// Returns cos(phase) at current phase, scaled by amplitude
float Oscillator::getCos(float intensity) const {
    intensity = constrain(intensity, 0.0f, 1.0f);
    float amp = ampMin_ + (ampMax_ - ampMin_) * intensity;
    return amp * cosf(phase_);
}

float Behavior::getIntensity(const MotionControl& m) {
    return constrain(m.adjust, 0.0f, 1.0f);
}



// ------------- //
//   Behaviors   //
// ------------- //

Control Happy::iterate(uint32_t now, const MotionControl& in, bool isBalancing) {
    Control out(in);
    float intensity = getIntensity(in);
    float tf = now / 1000.0f;
    osc_.freqMax_ = isBalancing? FREQMAX / 1.8f : FREQMAX;

    float sinVal = osc_.update(intensity, tf);
    float cosVal = osc_.getCos(intensity);

    if (isBalancing) {
        out.pitchOffsetDeg = 6.0f * sinVal;
    } else {
        out.d_fwd += cosVal;
        out.d_side += sinVal;
    }
    return out;
}

Control Excited::iterate(uint32_t now, const MotionControl& in, bool isBalancing) {
    Control out(in);
    float intensity = getIntensity(in);
    float tf = now / 1000.0f;
    osc_.freqMax_ = isBalancing? FREQMAX / 2.0f : FREQMAX;

    float sinVal = osc_.update(intensity, tf);
    out.d_yaw += sinVal;
    return out;
}

Control Scared::iterate(uint32_t now, const MotionControl& in, bool isBalancing) {
    Control out(in);
    if (isBalancing)
        return out; //skip
    float intensity = getIntensity(in);
    float tf = now / 1000.0f;

    float sinVal = osc_.update(intensity, tf);
    out.d_side += sinVal;
    return out;
}

Control Drunk::iterate(uint32_t now, const MotionControl& in, bool isBalancing) {
    Control out(in);
    //filters all output by alpha_, persisting via filtered_.
    float a = isBalancing? ALPHA * 2 : ALPHA;
    out.m.fwd  = filtered_.fwd  = a * in.fwd  + (1.0f - a) * filtered_.fwd;
    out.m.side = filtered_.side = a * in.side + (1.0f - a) * filtered_.side;
    out.m.yaw  = filtered_.yaw  = a * in.yaw  + (1.0f - a) * filtered_.yaw;
    return out;
}


// ------------- //
//    Manager    //
// ------------- //

Manager::Manager() {
    behaviors_ = {new Excited, new Happy, new Scared, new Drunk};
    clear();
}
void Manager::increment(bool up) {
    activeIdx_ = (activeIdx_ + (up? 1 : -1)) % behaviors_.size();
    D_LOG("Behavior activating #%d", activeIdx_);
}
void Manager::clear() {
    D_LOG("Behavior clear (from #%d)", activeIdx_);
    prevActIdx_ = activeIdx_;
    activeIdx_ = behaviors_.size();
}
void Manager::clearOrRestore() {
    if (isActive()) clear();
    else activeIdx_ = prevActIdx_;
}

bool Manager::isActive() const {
    return activeIdx_ >= 0 && activeIdx_ < behaviors_.size();
}

const char* Manager::getName() const {
    if (activeIdx_ < 0 || activeIdx_ >= behaviors_.size())
        return (char*)"None";
    return behaviors_[activeIdx_]->getName();
}

Control Manager::iterate(uint32_t now, const MotionControl& in, bool isBalancing) {
    if (activeIdx_ < 0 || activeIdx_ >= behaviors_.size())
        return Control(in); //no behavior active
    return behaviors_[activeIdx_]->iterate(now, in, isBalancing);
}


} // namespace behavior

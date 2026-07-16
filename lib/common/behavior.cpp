#include "behavior.h"
#include "log.h"

namespace behavior {


Control Happy::iterate(uint32_t now, const MotionControl& in, bool isBalancing, Manager& mgr) {
    Control out(in);

    float elapsed = (now - startTime_) / 1000.0f;  // seconds
    float omega = 2.0f * M_PI * frequency_;
    float phase = omega * elapsed;

    // Circle: fwd = radius * cos(phase), side = radius * sin(phase)
    if (isBalancing) {
        out.pitchOffsetDeg = 6.0 * sinf(phase / 2);
    } else {
        out.d_fwd += radius_ * cosf(phase);
        out.d_side += radius_ * sinf(phase);
    }
    return out;
}

Control Excited::iterate(uint32_t now, const MotionControl& in, bool isBalancing, Manager& mgr) {
    Control out(in);
    float elapsed = (now - startTime_) / 1000.0f;
    float omega = 2.0f * M_PI * frequency_;
    float phase = omega * elapsed;
    out.d_yaw += amplitude_ * sinf(phase);
    return out;
}

Control Scared::iterate(uint32_t now, const MotionControl& in, bool isBalancing, Manager& mgr) {
    Control out(in);
    if (isBalancing)
        mgr.clear(); //balancing is brave! no being scared.
    float elapsed = (now - startTime_) / 1000.0f;
    float omega = 2.0f * M_PI * frequency_;
    float phase = omega * elapsed;
    out.d_side += amplitude_ * sinf(phase);
    return out;
}

Control Drunk::iterate(uint32_t now, const MotionControl& in, bool isBalancing, Manager& mgr) {
    Control out(in);
    if (isBalancing)
        mgr.clear(); //balancing takes sobriety!
    //filters all output by alpha_, persisting via filtered_.
    out.m.fwd  = filtered_.fwd  = alpha_ * in.fwd  + (1.0f - alpha_) * filtered_.fwd;
    out.m.side = filtered_.side = alpha_ * in.side + (1.0f - alpha_) * filtered_.side;
    out.m.yaw  = filtered_.yaw  = alpha_ * in.yaw  + (1.0f - alpha_) * filtered_.yaw;
    return out;
}


// ------------- //
//    Manager    //
// ------------- //

Manager::Manager() {
    behaviors_ = {new Happy, new Excited, new Scared, new Drunk};
    clear();
}
void Manager::increment() {
    activeIdx_ = (activeIdx_ + 1) % (behaviors_.size() + 1); //+1 allows for disabled
    D_LOG("Behavior activating #%d", activeIdx_);
}
void Manager::clear() {
    D_LOG("Behavior clear (from #%d)", activeIdx_);
    activeIdx_ = behaviors_.size();
}

Control Manager::iterate(uint32_t now, const MotionControl& in, bool isBalancing, Manager& mgr) {
    if (activeIdx_ < 0 || activeIdx_ >= behaviors_.size())
        return Control(in); //no behavior active
    return behaviors_[activeIdx_]->iterate(now, in, isBalancing, mgr);
}


} // namespace behavior

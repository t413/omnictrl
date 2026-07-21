#pragma once
#include <stdint.h>
#include <WString.h>

class MotionTask;
class DriveManager;
struct SharedState;

class DynamicsBase {
protected:
  MotionTask* ctx_;

public:
  DynamicsBase(MotionTask* ctx) : ctx_(ctx) { }
  virtual ~DynamicsBase() = default;

  virtual void init() = 0;
  virtual void enable(bool) = 0;
  virtual void iterate(uint32_t now, const SharedState&, DriveManager&) = 0;
  virtual void updateState(uint32_t now, SharedState&) { }
  virtual void resetPids() = 0;
  virtual String getStatus() const = 0;
  virtual bool isBalancing() const { return false; }
};

#pragma once
#include <stdint.h>
#include <WString.h>

class Controller;
class MotorDrive;

class DynamicsBase {
protected:
  Controller* ctrl_;

public:
  DynamicsBase(Controller* ctrl) : ctrl_(ctrl) { }
  virtual ~DynamicsBase() = default;

  virtual void init() = 0;
  virtual void enable(bool) = 0;
  virtual void iterate(uint32_t now) = 0;
  virtual void resetPids() = 0;
  virtual String getStatus() const = 0;
};

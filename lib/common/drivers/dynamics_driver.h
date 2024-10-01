#pragma once
#include <stdint.h>
#include <map>

class Controller;
typedef std::map<std::string, float*> Adjustables;

class DynamicsDriver {
public:
  virtual DynamicsDriver* canInit(Controller*) = 0;
  virtual void loop() = 0;

  virtual Adjustables getAdjustables() = 0;

};

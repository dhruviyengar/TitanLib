#pragma once

#include "titanlib/chassis/bezier/bezier.hpp"
#include <map>
#include "titanlib/chassis/bezier/bezier.hpp"

namespace titanlib {

class MotionPlan {

public:
  MotionPlan(CubicBezier bezier, float maxAccel, float maxDeAccel,
             float maxVel);
  void generate(float ceiling = 1);
  float getLinearVelocity(float t);
  float getAngularVelocity(float t);
  CubicBezier getCurve();

private:
  CubicBezier bezier;
  float maxAccel;
  float maxDeAccel;
  float maxVel;
  std::map<float, float> linearVelocities;
  std::map<float, float> angularVelocities;
};

} // namespace titanlib
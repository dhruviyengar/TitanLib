#pragma once

#include "titanlib/chassis/profile/profile.hpp"
#include <cmath>

namespace titanlib {
class SinusoidalProfile : public MotionProfile {

public:
  SinusoidalProfile(float totalTime, float distance);
  float getPosition(float time) override;
  float getVelocity(float time) override;

private:
  float totalTime;
  float distance;
};
} // namespace titanlib
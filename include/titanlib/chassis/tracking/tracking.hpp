#pragma once

#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <cmath>

namespace titanlib {

class TrackingWheel {
public:
  TrackingWheel(pros::Rotation *rotation, float wheelSize, float vertOffset,
                float horzOffset);
  void tare();
  float getDistanceTravelled();
  float getWheelSize();
  float getVerticalOffset();
  float getHorizontalOffset();

private:
  pros::Rotation *rotation;
  float wheelSize;
  float vertOffset;
  float horzOffset;
  float prevPosition = 0;
};

} // namespace titanlib
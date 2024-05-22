#include "titanlib/chassis/tracking/tracking.hpp"

namespace titanlib {

TrackingWheel::TrackingWheel(pros::Rotation *rotation, float wheelSize,
                             float vertOffset, float horzOffset) {
  this->rotation = rotation;
  this->wheelSize = wheelSize;
  this->vertOffset = vertOffset;
  this->horzOffset = horzOffset;
  tare();
}

void TrackingWheel::tare() {
  rotation->reset_position();
  prevPosition = 0;
}

float TrackingWheel::getDistanceTravelled() {
  float position = rotation->get_position() / 100.0;
  float rotation = (position - prevPosition) / 360;
  float distanceTravelled = (rotation * wheelSize * M_PI);
  prevPosition = position;
  return distanceTravelled;
}

float TrackingWheel::getWheelSize() { return wheelSize; }

float TrackingWheel::getVerticalOffset() { return vertOffset; }

float TrackingWheel::getHorizontalOffset() { return horzOffset; }

} // namespace titanlib
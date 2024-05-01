#include "titanlib/chassis/profile/trapezoidal.hpp"
#include <cmath>

//ALL UNITS ARE IN IN/S AND IN/S/S
namespace titanlib {

TrapezoidalProfile::TrapezoidalProfile(float maxAccel, float maxVel,
                                       float distance) {
  this->maxAccel = maxAccel;
  this->maxVel = maxVel;
  this->distance = distance;
  this->accelTime = maxVel / maxAccel;
  this->accelDist = 0.5 * maxAccel * powf(accelTime, 2);
  this->cruiseTime = (distance - (2 * accelDist)) / maxVel;
  this->cruiseDist = maxVel * cruiseTime;
}

float TrapezoidalProfile::getPosition(float time) {
  if (time < accelTime) {
    return 0.5 * maxAccel * powf(time, 2);
  } else if (time < accelTime + cruiseTime) {
    return accelDist + maxVel * (time - accelTime);
  } else if (time < 2 * accelTime + cruiseTime) {
    return (accelDist + cruiseDist) +
           ((maxVel * (time - accelTime - cruiseTime))) -
           (0.5 * maxAccel * powf(time - accelTime - cruiseTime, 2));
  } else {
    return distance;
  }
}

float TrapezoidalProfile::getVelocity(float time) {
  if (time < accelTime) {
    return time * maxAccel;
  } else if (time < accelTime + cruiseTime) {
    return maxVel;
  } else if (time < 2 * accelTime + cruiseTime) {
    return maxVel - maxAccel * (time - cruiseTime - accelTime);
  } else {
    return 0;
  }
}

} // namespace titanlib
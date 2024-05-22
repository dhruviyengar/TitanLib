#include "titanlib/chassis/chassis.hpp"
#include "titanlib/chassis/ramsete/ramsete.hpp"
#include "titanlib/chassis/util/util.hpp"

namespace titanlib {

void Chassis::followPlan(MotionPlan plan) {
  isMoving.set(true);
  float prevT = 0;
  Ramsete ramsete(0.05, 0.7);
  while (true) {
    float t = plan.getCurve().closestPoint(getPos(), prevT, 0.01);
    Point pathPoint = plan.getCurve().getPoint(t);
    float v = plan.getLinearVelocity(t);
    float w = plan.getAngularVelocity(t);
    std::pair<float, float> velocities = ramsete.ramseteOutput(
        getPos(), pathPoint, getHeading(), plan.getCurve().getHeading(t), v, w);
    printf("%f %f %f %f %f %f\n", v, w, velocities.first, velocities.second, getHeading(), plan.getCurve().getHeading(t));
    v = velocities.first;
    w = velocities.second;
    float left = v + ((w * 12) / 2);
    float right = v - ((w * 12) / 2);
    float rpmScale = (1.0 / 60.0) * ((wheelSize * M_PI) / ratio);
    leftMotors->move_velocity(left / rpmScale);
    rightMotors->move_velocity(right / rpmScale);
    pros::delay(10);
    prevT = t;
  }
  leftMotors->move(0);
  rightMotors->move(0);
  isMoving.set(false);
}

} // namespace titanlib
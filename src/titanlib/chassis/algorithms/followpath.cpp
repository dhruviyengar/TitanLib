#include "api.h"
#include "titanlib/chassis/chassis.hpp"
#include "titanlib/chassis/profile/trapezoidal.hpp"
#include "titanlib/chassis/util/util.hpp"

namespace titanlib {

void Chassis::followPath(CubicBezier bezier, float maxAccel, float maxVel,
                         FollowPathParams params, float lookAhead, bool async) {
  TrapezoidalProfile profile(maxAccel, maxVel, bezier.arcLength(0, 1));
  // need to add in task
  isMoving.set(true);
  float start = pros::millis();
  float prevT = 0;
  PID linearPID = PID(linear);
  PID angularPID = PID(angular);
  float time = 0;
  while (time < profile.getTotalTime()) {
    float position = profile.getPosition(time);
    float velocity = profile.getVelocity(time);
    float theta = getHeading();
    if (params.forwards == false) {
      theta = constrainAngle(theta + 180);
    }
    float t = bezier.closestPoint(getPos(), prevT, 0.01);
    float curvature = -bezier.getCurvature(t);
    if (curvature == NAN || curvature == -NAN || curvature == INFINITY ||
        curvature == -INFINITY)
      curvature = 0;
    float linearVel =
        linearPID.update(position - bezier.arcLength(0, t),
                         10); // need to add in FF and velocity calculations
    float targetHeading = constrainAngle(
        slopeToHeading(bezier.getYDerivative(t) / bezier.getXDerivative(t)) *
        sgn(bezier.getXDerivative(t)));
    float angularVel =
        angularPID.update(angleError(getHeading(), targetHeading), 10);
    Point bezierPoint = bezier.getPoint(t);
    int slopeSgn = sgn(bezierPoint - getPos());
    angularVel += bezierPoint.distance(getPos()) * slopeSgn;
    if (params.forwards == false)
      linearVel = -linearVel;
    float leftVel = linearVel + angularVel;
    float rightVel = linearVel - angularVel;
    leftVel = ratioSpeedsToMax(leftVel, rightVel, 127).first;
    rightVel = ratioSpeedsToMax(leftVel, rightVel, 127).second;
    leftMotors->move(leftVel);
    rightMotors->move(rightVel);
    prevT = t;
    printf("%f", time);
    pros::delay(10);
    time += (pros::millis() - start) / 1000.0;
  }
  leftMotors->move(0);
  rightMotors->move(0);
  isMoving.set(false);
}

} // namespace titanlib
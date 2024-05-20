#include "titanlib/chassis/chassis.hpp"
#include "titanlib/chassis/profile/trapezoidal.hpp"
#include "api.h"
#include "titanlib/chassis/util/util.hpp"

namespace titanlib {

void Chassis::followPath(CubicBezier bezier, float maxAccel, float maxVel, FollowPathParams params, float lookAhead, bool async) {
    TrapezoidalProfile profile(maxAccel, maxVel, bezier.arcLength(0, 1));
    //need to add in task
    isMoving.set(true);
    float start = pros::millis();
    float prevT = 0;
    PID linearPID = PID(linear);
    PID angularPID = PID(angular);
    while ((pros::millis() - start) / 1000.0 < profile.getTotalTime()) {
        float position = profile.getPosition((pros::millis() - start) / 1000.0);
        float velocity = profile.getVelocity((pros::millis() - start) / 1000.0);
        float theta = getHeading();
        if (params.forwards == false) {
            theta = constrainAngle(theta + 180);
        }
        float t = bezier.closestPoint(getPos(), prevT, 0.01);
        float linearVel = linearPID.update(position - bezier.arcLength(0, t), 10); //need to add in FF and velocity calculations
        float curvature = -bezier.getCurvature(t);
        if (curvature == NAN || curvature == -NAN || curvature == INFINITY || curvature == -INFINITY) curvature = 0;
        float targetHeading = constrainAngle(slopeToHeading(bezier.getYDerivative(t) / bezier.getXDerivative(t)) * sgn(bezier.getXDerivative(t)));
        float angularVel = angularPID.update(angleError(getHeading(), targetHeading), 10);
        /*if (bezier.getPoint(t).distance(bezier.getPoint(prevT)) != 0) {
            Point crossTrack = lerp(bezier.getPoint(prevT), bezier.getPoint(t), lookAhead / bezier.getPoint(t).distance(bezier.getPoint(prevT)));
            float crossAngle = angleError(theta, getPos().angle(crossTrack));
            angularVel += crossAngle * 0.5;
        }*/
        if (params.forwards == false) linearVel = -linearVel;
        float leftVel = linearVel + angularVel;
        float rightVel = linearVel - angularVel;
        leftVel = ratioSpeedsToMax(leftVel, rightVel, 127).first;
        rightVel = ratioSpeedsToMax(leftVel, rightVel, 127).second;
        leftMotors->move(leftVel);
        rightMotors->move(rightVel);
        prevT = t;
        printf("%f", (pros::millis() - start) / 1000.0);
        pros::delay(10);
    }
    leftMotors->move(0);
    rightMotors->move(0);
    isMoving.set(false);
}

}
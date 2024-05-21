#include "titanlib/chassis/chassis.hpp"
#include "titanlib/chassis/util/util.hpp"

namespace titanlib {

void Chassis::followPlan(MotionPlan plan) {
    isMoving.set(true);
    float prevT = 0;
    while (true) {
        float t = plan.getCurve().closestPoint(getPos(), prevT, 0.01);
        Point pathPoint = plan.getCurve().getPoint(t);
        float thetaError = angleError(getHeading(), getPos().angle(pathPoint)) * (M_PI / 180);
        Point error(sin(thetaError) * pathPoint.distance(getPos()), cos(thetaError) * pathPoint.distance(getPos()));
        float v = plan.getLinearVelocity(t);
        float w = plan.getAngularVelocity(t);
        if (t < 0.95) w -= error.getX() * 0.19;
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

}
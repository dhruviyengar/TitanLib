#pragma once

#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "titanlib/chassis/pid/pid.hpp"
#include "titanlib/chassis/point/point.hpp"
#include "titanlib/chassis/profile/profile.hpp"
#include "titanlib/chassis/tracking/tracking.hpp"
#include "titanlib/variable/variable.hpp"
#include "titanlib/chassis/bezier/bezier.hpp"
#include "titanlib/chassis/util/util.hpp"
#include "titanlib/chassis/profile/motion.hpp"


namespace titanlib {

struct FollowPathParams {
  bool forwards = true;
};

class Chassis {

public:
  Chassis(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors,
          pros::Imu *imu, TrackingWheel *vertWheel, TrackingWheel *horzWheel,
          float ratio, float wheelSize, float chasePower, PIDSettings linear,
          PIDSettings angular);
  void followPlan(MotionPlan plan);
  void calibrate();
  void setPose(Point point, float heading);
  Point getPos();
  float getHeading();

private:
  Variable<Point> pos = Variable<Point>(Point(0, 0));
  Variable<float> heading = Variable<float>(0);
  Variable<bool> isMoving = Variable<bool>(false);
  float ratio;
  float wheelSize;
  float chasePower;
  PIDSettings linear = PIDSettings(0, 0, 0, 0, 0);
  PIDSettings angular = PIDSettings(0, 0, 0, 0, 0);
  pros::MotorGroup *leftMotors;
  pros::MotorGroup *rightMotors;
  float prevPosLeft = 0;
  float prevPosRight = 0;
  pros::Imu *imu;
  TrackingWheel *vertWheel = nullptr;
  TrackingWheel *horzWheel = nullptr;
  void updateOdom();
};

} // namespace titanlib
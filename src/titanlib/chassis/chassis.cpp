#include "titanlib/chassis/chassis.hpp"
#include "pros/misc.h"

namespace titanlib {

Chassis::Chassis(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors,
                 pros::Imu *imu, TrackingWheel *vertWheel,
                 TrackingWheel *horzWheel, float ratio, float wheelSize,
                 float chasePower, PIDSettings linear, PIDSettings angular) {
  this->leftMotors = leftMotors;
  this->rightMotors = rightMotors;
  this->imu = imu;
  this->vertWheel = vertWheel;
  this->horzWheel = horzWheel;
  this->ratio = ratio;
  this->wheelSize = wheelSize;
  this->chasePower = chasePower;
  this->linear = linear;
  this->angular = angular;
}

void Chassis::calibrate() {
  leftMotors->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  rightMotors->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  leftMotors->tare_position();
  rightMotors->tare_position();
  imu->reset(true);
  pros::Task odomTask{[=] {
    while (true) {
      updateOdom();
      pros::delay(10);
    }
  }};
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "-");
}

void Chassis::setPose(Point pos, float heading) {
  this->pos.set(pos);
  this->heading.set(heading);
}

Point Chassis::getPos() { return pos.get(); }

float Chassis::getHeading() { return heading.get(); }

} // namespace titanlib
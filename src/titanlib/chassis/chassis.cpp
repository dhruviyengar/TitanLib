#include "titanlib/chassis/chassis.hpp"

namespace titanlib {

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

Point Chassis::getPos() {
    return pos.get();
}

float Chassis::getHeading() {
    return heading.get();
}

} // namespace titanlib
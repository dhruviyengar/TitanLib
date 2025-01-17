#include "main.h"

#include "titanlib/chassis/chassis.hpp"
#include "titanlib/chassis/pid/pid.hpp"
#include "titanlib/chassis/profile/motion.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

using namespace titanlib;

pros::Motor left1(1, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor left2(2, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor left3(3, pros::E_MOTOR_GEAR_BLUE, true);

pros::Motor right1(8, pros::E_MOTOR_GEAR_BLUE, false);
pros::Motor right2(9, pros::E_MOTOR_GEAR_BLUE, false);
pros::Motor right3(7, pros::E_MOTOR_GEAR_BLUE, false);

pros::MotorGroup leftMotors{left1, left2, left3};
pros::MotorGroup rightMotors{right1, right2, right3};

pros::IMU imu(11);

titanlib::Chassis chassis(&leftMotors, &rightMotors, &imu, nullptr, nullptr,
                          1.333, 3.25, 2,
                          titanlib::PIDSettings(8, 12, 300, 1, 2000),
                          titanlib::PIDSettings(5, 40, 100, 3, 2000));

int screen_refresh() {
  while (true) {
    pros::lcd::set_text(0, "Position: " + chassis.getPos().asString());
    pros::lcd::set_text(1, "Heading: " + std::to_string(chassis.getHeading()));
    pros::delay(10);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();
  pros::Task screen(screen_refresh);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
  // CubicBezier bezier(Point(0, 0), Point(40, 40), Point(0, 20), Point(10, 20))
  chassis.setPose(titanlib::Point(0, 0), 90);

  CubicBezier bezier(Point(0, 0), Point(40, 40), Point(20, 0), Point(40, 40));
  CubicBezier halfPoint = bezier.deCasteljau(0.5);
  // chassis.followPath(bezier, 24, 36);
  // printf("%f", bezier.getCurvature(0.1));
  MotionPlan plan(bezier, 30.0, 20.0, 35.0);

  float start = pros::millis();
  plan.generate();
  // printf("%f\n%f", plan.getCurve().arcLength(0, 0.01, true),
  // plan.getCurve().arcLength(0, 0.01, false));
  printf("%f", pros::millis() - start);
  pros::delay(5000);
  chassis.followPlan(plan);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {}

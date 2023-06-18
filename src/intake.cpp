
#include "puncher.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"


pros::Motor intake_Right(23, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake_Left(23, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);



//intake into the robot
void intake_in() {
  intake_Right.move_velocity(200);
  intake_Left.move_velocity(200);
}

//intake out of the robot
void intake_out() {
  intake_Right.move_velocity(-200);
  intake_Left.move_velocity(-200);
}

//stop the intake
void intake_stop() {
  intake_Right.move_velocity(0);
  intake_Left.move_velocity(0);
}
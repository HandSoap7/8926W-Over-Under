// Intake File
#include "puncher.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"


// Recoginition of both intake motors, brain ports, gearset (5.5w motors are fixed to 18),
// which one is reversed (opposite sides), and what encoder units are being used (degrees)
pros::Motor intake_Right(23, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake_Left(23, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);



// intake into the robot
// Sets both motors to positive max power (200)
void intake_in() {
  intake_Right.move_velocity(200);
  intake_Left.move_velocity(200);
}

// intake out of the robot
// Sets both motors to negative max power (-200)
void intake_out() {
  intake_Right.move_velocity(-200);
  intake_Left.move_velocity(-200);
}

// stop the intake
// Sets both motors to zero (0)
void intake_stop() {
  intake_Right.move_velocity(0);
  intake_Left.move_velocity(0);
}
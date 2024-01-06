// Intake File
#include "intake.hpp"
#include "EZ-Template/util.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"


// Recoginition of both intake motors, brain ports, gearset (5.5w motors are fixed to 18),
// which one is reversed (opposite sides), and what encoder units are being used (degrees)
pros::Motor intake(11, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);





// intake into the robot
// Sets both motors to positive max power (200)
void intake_in(int rpm) {
  intake.move_velocity(rpm);
}


// intake out of the robot
// Sets both motors to negative max power (-200)
void intake_out(int rpm) {
  intake.move_velocity(-rpm);
}

// stop the intake
// Sets both motors to zero (0)
void intake_stop() {
  intake.move_velocity(0);
}

void intake_hold() {
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void intake_coast() {
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void intakeHold() {
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
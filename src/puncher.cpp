
#include "puncher.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

pros::Motor puncher(9, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::ADIDigitalIn puncher_limit('A');
pros::ADIDigitalIn puncher_limit2('B');

pros::Rotation puncher_rotation(9);


void puncher_task(void* param) {
  while (true) {
    if (puncher_rotation.get_angle() < 200) {
      puncher.move_velocity(200);
    } else {
      puncher.move_velocity(0);
    }
    pros::delay(10);
  }
}

void puncher_init() {
  pros::Task puncher_reload(puncher_task, NULL, "Puncher Task");
}

void puncher_fire() {
  puncher.move_velocity(200);
  pros::delay(1000);
  puncher.move_velocity(0);
}

void puncher_stop() {
  puncher.move_velocity(0);
}

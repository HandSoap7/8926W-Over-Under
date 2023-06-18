
#include "puncher.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"


//Puncher motor, limit switches, and rotation sensor ports
pros::Motor puncher(9, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::ADIDigitalIn puncher_limit('A');
pros::ADIDigitalIn puncher_limit2('B');

pros::Rotation puncher_rotation(9);




//puncher reload task using rotation sensor
//puncher stops at a certain degree (200) and if it's below the degree it will run the motor
void puncher_reload_rotation_task(void* param) {
  while (true) {
    if (puncher_rotation.get_angle() < 20000) { // 20000 is the desired centidegree 
      puncher.move_velocity(200);
    } else {
      puncher.move_velocity(0);
    }
    pros::delay(10);
  }
}


//puncher reload task using limit switches
//puncher stops when either limit switch is pressed and if neither are pressed it will run the motor
void puncher_reload_limit_task(void* param) {
  while (true) {
    // of either puncher limits are pressed, stop the puncher, otherwise run the motor
    if (puncher_limit.get_value() == 0 || puncher_limit2.get_value() == 0) {
      puncher.move_velocity(0);
    } else {
      puncher.move_velocity(200);
    }
    pros::delay(10);
  }
}


//puncher init function to start the puncher reload tasks at the beginning of the program
void puncher_init() {
    pros::Task puncher_reload_rotation(puncher_reload_rotation_task, NULL, "Puncher reload task (rotation)");
    pros::Task puncher_reload_limit(puncher_reload_limit_task, NULL, "Puncher reload task (limit)");
}


//Function to overide the puncher reload task and run the puncher motor
void puncher_fire() {
  puncher.move_velocity(200);
  pros::delay(50);
}


//Function to stop the puncher motor (shouldn't be used in most cases)
void puncher_stop() {
  puncher.move_velocity(0);
}

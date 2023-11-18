
#include "catapult.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "definitions.hpp"

//pros::Controller master(pros::E_CONTROLLER_MASTER); // master controller


//catapult motor, limit switches, and rotation sensor ports
pros::Motor catapult(9, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);


// Recognizes the type of value (digital(0/1) or analog(0-127)), the name in the code, and the port on the brain
pros::ADIDigitalIn catapult_limit('G');
//pros::ADIDigitalIn catapult_limit2('H');

// Recognizes the type of sensor, the name in the code, and the port on the brain
pros::Rotation catapult_rotation(2);


void cata_move(int voltage){
  catapult.move_voltage(voltage);
}



bool ManualOverride = false;
bool FastFire = false;
int CataStopDegree = 5600;
int IntakeBlockDegree = 4500;
int HangStopDegree = 2200;
int UsuableStopDegree = IntakeBlockDegree;

void FastFireState(bool state){
FastFire = state;
}

void ManualOverrideState(bool state){
ManualOverride = state;
}

void SetStopDegree(int State){

if (State == 1){
UsuableStopDegree = IntakeBlockDegree;
}
else if(State == 2){
UsuableStopDegree = CataStopDegree;
}
else if(State == 3){
UsuableStopDegree = HangStopDegree;
}
else{
}

}



//catapult reload task using rotation sensor
//catapult stops at a certain degree (200) and if it's below the degree it will run the motor
void catapult_reload_rotation_task(void* param) {
  while (ManualOverride==false) {
    //printf("Stop Degree is %i /n", UsuableStopDegree);

    if (FastFire==true){
      cata_move(12000);
      }
    else{
      //Hard stop Degree is around 10100 millidegrees, The stop degree is atleast 10000 millidegrees
      if (catapult_rotation.get_angle() <= UsuableStopDegree) { // 20000 is the desired centidegree 
        cata_move(12000);
        printf("Rot degree = %i\n", catapult_rotation.get_angle());
      } 
      else {
        cata_move(0);
      }

      }
    pros::delay(150);
  }
}

bool runCata = true;

void TaskState(bool State){
  runCata = State;
}


/*
bool AutonOverride = false;





//catapult reload task using limit switches
//catapult stops when either limit switch is pressed and if neither are pressed it will run the motor
void catapult_reload_limit_task(void* param) {
 // printf("catapult_reload_limit_task\n");
 catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (runCata==true) {

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      cata_move(12000);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
      cata_move(12000);
    }
    else if(AutonOverride == true){
      cata_move(12000);
    }
    else{
    cata_move(0);
    // of either catapult limits are pressed, stop the catapult, otherwise run the motor
    //printf("limit 1: %i\n", catapult_limit.get_value());
    if (catapult_limit.get_value() == 1) {  //(catapult_limit.get_value() == 0 || catapult_limit2.get_value() == 0)
      cata_move(0);
    } else {
      cata_move(12000);
    }
    }
    pros::delay(20);

    //printf(runCata ? "true\n" : "false\n")
    
  }
}

*/

/*
//catapult init function to start the catapult reload tasks at the beginning of the program
void catapult_init() {
  pros::Task Reload_Rotation(catapult_reload_rotation_task);
  pros::Task Reload_Limit(catapult_reload_limit_task);
}
*/


//Fire the catapult
void catapult_fire() {
  runCata = false;
  cata_move(12000);
  pros::delay(500);
  runCata = true;
  cata_move(0);
}

void RapidFire(){
  cata_move(12000);
}


//Function to stop the catapult motor (shouldn't be used in most cases)
void catapult_stop() {
  runCata=false;
  cata_move(0);
}

//Function to return the catapult rotation sensor degree value
double catapult_get_rotation() {
  return catapult_rotation.get_angle();
}

//Function to return the catapult limist switch value
bool catapult_get_limit() {
  return catapult_limit.get_value(); //&& catapult_limit2.get_value();
  
}


//Function to reset the rotation sensor degrees to 0
void rotation_reset() {
  catapult_rotation.reset();
}


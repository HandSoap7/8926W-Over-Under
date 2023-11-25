
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
//pros::ADIDigitalIn catapult_limit('G');
//pros::ADIDigitalIn catapult_limit2('H');

// Recognizes the type of sensor, the name in the code, and the port on the brain
pros::Rotation catapult_rotation(2);
  

void cata_move(int voltage){
  catapult.move_voltage(voltage);
}



bool ManualOverride = false;
bool FastFire = false;
int CataStopDegree = 5600;
int IntakeBlockDegree = 4250;
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


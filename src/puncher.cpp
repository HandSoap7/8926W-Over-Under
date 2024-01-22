
#include "puncher.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "definitions.hpp"



//////////////////////////////////////////////////////////////////////

// Initialization

/////////////////////////////////////////////////////////////////////


//puncher motor, limit switches, and rotation sensor ports
pros::Motor puncher11Watt(9, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor puncherHalfWatt(10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);


// Recognizes the type of sensor, the name in the code, and the port on the brain
pros::Rotation puncher_rotation(2);
pros::Distance puncher_distance(2);


//////////////////////////////////////////////////////////////////////

// Universal

/////////////////////////////////////////////////////////////////////

void puncher_move(int voltage){
  puncher11Watt.move_voltage(voltage);
  puncherHalfWatt.move_voltage(voltage);
}



bool ManualOverride = false;
bool FastFire = false;
int CataStopDegree = 5300;
int IntakeBlockDegree = 4300;
int HangStopDegree = 2250;
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


//////////////////////////////////////////////////////////////////////

// Rotation Sensor

/////////////////////////////////////////////////////////////////////


//puncher reload task using ROTATION sensor
//puncher stops at a certain degree (200) and if it's below the degree it will run the motor

void puncher_reload_rotation_task(void* param) {
  while (ManualOverride==false) {
    //printf("Stop Degree is %i /n", UsuableStopDegree);

    if (FastFire==true){
      puncher_move(12000);
      }
    else{
      //Hard stop Degree is around 10100 millidegrees, The stop degree is atleast 10000 millidegrees
      if (puncher_rotation.get_angle() <= UsuableStopDegree) { // 20000 is the desired centidegree 
        puncher_move(12000);
        //printf("Rot degree = %i\n", puncher_rotation.get_angle());
      } 
      else {
        puncher_move(0);
      }

      }
    pros::delay(20);
  }
  puncher_move(0);
}





//////////////////////////////////////////////////////////////////////

// Distance Sensor

/////////////////////////////////////////////////////////////////////


int DistanceFromSensor = 50;    //Goal distance from sensor in millimeters

//puncher reload task using DISTANCE sensor
//puncher stops at a certain degree (200) and if it's below the degree it will run the motor

void puncher_reload_distance_task(void* param) {
  while (ManualOverride==false) {
    printf("Goal Distance from sensor is : %imm/n", DistanceFromSensor);

    if (FastFire==true){
      puncher_move(12000);
      }
    else{
      //Hard stop Degree is around 10100 millidegrees, The stop degree is atleast 10000 millidegrees
      if (puncher_rotation.get_angle() <= DistanceFromSensor) { // 20000 is the desired centidegree 
        pros::delay(200);
        puncher_move(12000);
        printf("Current distance from sensor is : %imm/n", puncher_distance.get());
      } 
      else {
        puncher_move(0);
      }

      }
    pros::delay(20);
  }
  puncher_move(0);
}
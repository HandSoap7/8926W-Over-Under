#include "autons.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "catapult.hpp"
#include "intake.hpp"
#include "main.h"
#include "pistons.hpp"
#include "pros/rtos.hpp"
#include <set>


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int DRIVE_SPEED = 120; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 110;
const int SWING_SPEED = 100;



///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 4, 0.003, 35, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 1.35, 0, 8.5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 1.25, 0, 6, 0);
  chassis.set_pid_constants(&chassis.turnPID, 6.5, 0.003, 50, 17.5);
  chassis.set_pid_constants(&chassis.swingPID, 15, 0, 125, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 400, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 60, 40, 250, 150, 300, 350);
}

void reliable_exit_conditions() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}


int counter = 0;


///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches


  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  while(chassis.get_mode() == ez::DRIVE){
  printf("%i,%i,%i,%i,%i\n", counter, chassis.right_sensor(), chassis.right_velocity(), chassis.left_sensor(), chassis.left_velocity() );
  pros::delay(util::DELAY_TIME);
  counter = counter + 20;
  }
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();
}



///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at


  chassis.set_turn_pid(90, TURN_SPEED);
  /*while(counter <= 1500){
  printf("%i,%i,%i,%i,%i,%f\n", counter, chassis.right_sensor(), -chassis.right_velocity(), -chassis.left_sensor(), chassis.left_velocity(), chassis.get_gyro() );
  pros::delay(util::DELAY_TIME);
  counter = counter + 20;
  }*/
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
}

// Left Chassis Ports (negative port will reverse it!)
pros::Motor LF(7, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor LM(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor LB(19, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
// Right Chassis Ports (negative port will reverse it!)
pros::Motor RF(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RM(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor RB(15, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);


void fullForceMove(int time, bool forwards){
  if (forwards == true){
  LF.move_voltage(12000);
  LM.move_voltage(12000);
  LB.move_voltage(12000);
  RF.move_voltage(12000);
  RM.move_voltage(12000);
  RB.move_voltage(12000);
  }
  else{
  LF.move_voltage(-12000);
  LM.move_voltage(-12000);
  LB.move_voltage(-12000);
  RF.move_voltage(-12000);
  RM.move_voltage(-12000);
  RB.move_voltage(-12000);
  }
  pros::delay(time);
  LF.move_voltage(0);
  LM.move_voltage(0);
  LB.move_voltage(0);
  RF.move_voltage(0);
  RM.move_voltage(0);
  RB.move_voltage(0);

}


double CurrentResistanceAmp = 0;
double CurrentResistanceTorque = 0;
void MoveTilResistance(double Resistance){
  CurrentResistanceTorque = LF.get_torque() + LM.get_torque() + LB.get_torque() + RF.get_torque() + RM.get_torque() + RB.get_torque() /6;
    CurrentResistanceAmp = LF.get_current_draw() + LM.get_current_draw() + LB.get_current_draw() + RF.get_current_draw() + RM.get_current_draw() + RB.get_current_draw() /6;
    printf("Current Resistance is %d \n", CurrentResistanceTorque);
    printf("Current Resistance is %d \n", CurrentResistanceAmp);
    printf("Resistance is %d \n", Resistance);
  
  while(Resistance > CurrentResistanceTorque){
    LF.move_velocity(12000);
    LM.move_velocity(12000);
    LB.move_velocity(12000);
    RF.move_velocity(12000);
    RM.move_velocity(12000);
    RB.move_velocity(12000);
    CurrentResistanceTorque = LF.get_torque() + LM.get_torque() + LB.get_torque() + RF.get_torque() + RM.get_torque() + RB.get_torque() /6;
    CurrentResistanceAmp = LF.get_current_draw() + LM.get_current_draw() + LB.get_current_draw() + RF.get_current_draw() + RM.get_current_draw() + RB.get_current_draw() /6;
    printf("Current Resistance is %d \n", CurrentResistanceTorque);
    printf("Current Resistance is %d \n", CurrentResistanceAmp);
    printf("Resistance is %i \n", Resistance);
    pros::delay(20);
  }
}






void SixBallSafe() {
  //basic initialization
  //pros::Task Reload_Limit(catapult_reload_limit_task);
  HorizWingR.set(false);
  HorizWingL.set(false);
  VertWingL.set(false);
  PistonHang.set(false);
  SideHang.set(false);

  //start of auton
  intake_in(600);

  //drive to the first ball
  chassis.set_drive_pid(5, DRIVE_SPEED, true);
  chassis.wait_drive();

  pros::delay(150);

    intake_in(0);


  //backup toawrds matchloader
  chassis.set_drive_pid(-39, DRIVE_SPEED, true);
  chassis.wait_drive();

  //drive towards matchloader and sweep the matchload triball out
  chassis.set_swing_pid(ez::LEFT_SWING, -45, -TURN_SPEED);
  chassis.wait_drive();

  HorizWingL.set(true);

  //drive towards goals
  chassis.set_drive_pid(-19, 85, true);
  chassis.wait_drive();

  //backup and repeat
  chassis.set_turn_pid(-100, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-65, TURN_SPEED);
  chassis.wait_drive();

  HorizWingL.set(false);

  intake_in(100);

  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_until(2);

  //backup and turn towards triball near the post
  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(105, TURN_SPEED);
  chassis.wait_drive();

  intake_out(600);

  pros::delay(300);

  chassis.set_drive_pid(35, 127, true);
  chassis.wait_until(12);

  chassis.set_drive_pid(-14, DRIVE_SPEED, true);

  chassis.set_turn_pid(2, TURN_SPEED);
  chassis.wait_drive();

  intake_in(600);

  //drive towards triball
  chassis.set_drive_pid(30, 127, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, 30, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(23, DRIVE_SPEED, true);
  chassis.wait_drive();

  pros::delay(200);

  chassis.set_turn_pid(160, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(22, 125, true);
  intake_out(600);
  chassis.wait_drive();

  pros::delay(100);

  chassis.set_drive_pid(-18, DRIVE_SPEED, true);
  chassis.wait_drive();

  //turn towards middle triball
  chassis.set_turn_pid(60, TURN_SPEED);
  chassis.wait_drive();

  intakeHold();


  //drive towards middle triball while intaking
  intake_in(600);

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  //turn towards goal and sweep other triballs in
  chassis.set_turn_pid(180, TURN_SPEED);

  chassis.wait_drive();

  HorizWingL.set(true);
  HorizWingR.set(true);

  intake_out(600);

  chassis.set_drive_pid(100, 127, true);
  chassis.wait_until(34);

  //go back and forth to knock triballs in

  chassis.set_drive_pid(-10, 127, true);
  chassis.wait_drive();

  chassis.set_drive_pid(18, 127, true);
  chassis.wait_until(4);

  chassis.set_drive_pid(-6, 127, true);
  chassis.wait_drive();

  intake_stop();

  //toggle off wings

  HorizWingL.set(false);
  HorizWingR.set(false);
}

void HighScoringShooting(){

}

void CloseMidRush(){
 
}


void SuperSimpleAWP(){


  HorizWingR.set(false);
  HorizWingL.set(false);
  VertWingL.set(false);
  PistonHang.set(false);
  SideHang.set(false);

  intake_out(600);

  chassis.set_turn_pid(0, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-7, DRIVE_SPEED);
  chassis.wait_drive();

  HorizWingR.set(true);

  chassis.set_turn_pid(2, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(2, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, -45, DRIVE_SPEED);
  chassis.wait_drive();

  
  HorizWingR.set(false);


  chassis.set_turn_pid(-28, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(28, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-42, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(16, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-25, DRIVE_SPEED);
  chassis.wait_drive();

 HorizWingL.set(true);
 
}



void SixBallMiddleMiddle(){

  chassis.set_drive_pid(60, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(-10, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(12, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(-20, 127);
  chassis.wait_drive();
}



void SixBallMiddleTop(){


}



void CloseMiddleOver(){




}



void CloseMiddleOverWait(){


}



void CloseTopMiddle(){



}



void CloseMiddleOverTouchHang(){



}

void Auton_Skills(){
  intake_coast();


  
  FastFireState(true);
  pros::delay(27000); //27000
  intake_out(200);
  pros::delay(4000); //4000 msec
  intake_in(400);
  pros::delay(1000);

  FastFireState(false);
  intake_out(600);

  pros::delay(200);
  chassis.reset_gyro(5);
  pros::delay(200);

  chassis.set_drive_pid(-2, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_out(600);

  chassis.set_turn_pid(32, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-98, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-16, DRIVE_SPEED, true);
  chassis.wait_drive();


  chassis.set_swing_pid(ez::LEFT_SWING, -37, -DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-17, DRIVE_SPEED, true);
  chassis.wait_until(2);

  chassis.set_turn_pid(-60, TURN_SPEED); 
  chassis.wait_drive();

  chassis.set_turn_pid(-144, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-50, DRIVE_SPEED, true);
  chassis.wait_until(30);

  pros::delay(400);

  //reset imu to 0 heading
  chassis.reset_gyro(0);

  pros::delay(200);

  HorizWingL.set(true);
  HorizWingR.set(true);

  chassis.set_turn_pid(-55, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(50, DRIVE_SPEED, true);
  chassis.wait_until(18);

  chassis.set_swing_pid(LEFT_SWING, 0, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(100, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_drive_pid(-10, 127, true);
  chassis.wait_drive();

  chassis.set_drive_pid(100, 127);
  chassis.wait_until(10);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();



  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 90, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-42, DRIVE_SPEED);
  chassis.wait_drive();


  HorizWingR.set(true);
  HorizWingL.set(true);

  chassis.set_turn_pid(60, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(22, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 0, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(50, DRIVE_SPEED, true);
  chassis.wait_until(16);


  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-14, 127, true);
  chassis.wait_drive();

  HorizWingR.set(true);
  HorizWingL.set(true);

  chassis.set_drive_pid(40, 127);
  chassis.wait_until(10);


  pros::delay(400);

  chassis.set_drive_pid(-40, DRIVE_SPEED, true);
  chassis.wait_drive();

  

  chassis.set_turn_pid(-55, 115);
  chassis.wait_drive();

  chassis.set_drive_pid(58, 90, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, 45, DRIVE_SPEED);
  chassis.wait_drive();



  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, 90, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(50, 127, true);
  chassis.wait_until(4);

  chassis.set_drive_pid(-8, 127, true);
  chassis.wait_drive();

  chassis.set_drive_pid(50, 127, true);
  chassis.wait_until(10);

  pros::delay(300);

  chassis.set_drive_pid(-16, DRIVE_SPEED, true);
  chassis.wait_until(4);

  
  
}

void tuning_constraints() {
 chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 4, .005, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 1.55, 0, 8, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 1.25, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 6.5, 0.003, 50, 17.5);
  chassis.set_pid_constants(&chassis.swingPID, 15, 0, 125, 0);
}

void tuning_exit_conditiions() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 6, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 6, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 25, 300, 50, 300, 500);
}

void EZ_Tune(){
  tuning_constraints();
  exit_condition_defaults();


 // chassis.set_drive_pid(10, DRIVE_SPEED, false, true);
  //chassis.set_turn_pid(90, TURN_SPEED);
  chassis.set_swing_pid(RIGHT_SWING, -90, DRIVE_SPEED);
  chassis.wait_drive();

}
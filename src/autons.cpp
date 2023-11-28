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
const int TURN_SPEED  = 105;
const int SWING_SPEED = 90;



///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 3, .005, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 4.5, 0.003, 27.5, 17.5);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void tuning_constraints() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void modified_exit_condition() {
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







void SixBallSafe() {
  //basic initialization
  //pros::Task Reload_Limit(catapult_reload_limit_task);
  intakeHold();
  WingL.set(false);
  WingR.set(false);

  //start of auton
  intake_in(600);

  pros::delay(450);

  //drive to the first ball
  chassis.set_drive_pid(5, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_stop();

  //backup toawrds matchloader
  chassis.set_drive_pid(-35, DRIVE_SPEED, true);
  chassis.wait_drive();

  //drive towards matchloader and sweep the matchload triball out
  chassis.set_swing_pid(ez::LEFT_SWING, -45, -TURN_SPEED);
  chassis.wait_drive();

  WingL.set(true);

  //drive towards goals
  chassis.set_drive_pid(-15, DRIVE_SPEED, true);
  chassis.wait_drive();

  //backup and repeat
  chassis.set_turn_pid(-100, TURN_SPEED);
  chassis.wait_drive();

  WingL.set(false);

  chassis.set_turn_pid(-75, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-18, DRIVE_SPEED, true);
  chassis.wait_drive();

  //backup and turn towards triball near the post
  chassis.set_drive_pid(8, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(2, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();
  
  intake_coast();

  chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();

  intake_out(250);

  pros::delay(250);

  intake_stop();

  chassis.set_turn_pid(28, TURN_SPEED);
  chassis.wait_drive();

  intake_hold();

  intake_in(450);

  //drive towards triball
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_stop();

  //backup, turn towards goal, and out take
  chassis.set_drive_pid(-1, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_coast();

  chassis.set_turn_pid(145, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_out(350);

  pros::delay(100);

  chassis.set_drive_pid(-18, DRIVE_SPEED, true);
  chassis.wait_drive();

  pros::delay(450);

  //turn towards middle triball
  chassis.set_turn_pid(60, TURN_SPEED);
  chassis.wait_drive();

  intakeHold();


  //drive towards middle triball while intaking
  intake_in(450);

  chassis.set_drive_pid(21, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_stop();

  //turn towards goal and sweep other triballs in
  chassis.set_turn_pid(185, TURN_SPEED);

  intake_coast();

  chassis.wait_drive();

  WingL.set(true);
  WingR.set(true);

  intake_out(600);

  chassis.set_drive_pid(32, DRIVE_SPEED, true);
  pros::delay(200);
  chassis.wait_drive();

  //go back and forth to knock triballs in

  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(7, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_stop();

  //toggle off wings
  WingL.set(false);
  WingR.set(false);

}

void HighScoringShooting(){

  //basic initialization
  //pros::Task Reload_Limit(catapult_reload_limit_task);
  intake_coast();
  WingL.set(false);
  WingR.set(false);

  //start of auton
  intake_in(450);    //outtake immediately to get rid of preloaded alliance ball

  chassis.set_drive_pid(46, DRIVE_SPEED, true);  //drive about 2/5 the way there
  chassis.wait_drive();

  pros::delay(400);

  chassis.set_drive_pid(-10, 50, true); //reverse to avoid getting entangled
  chassis.wait_drive();

  intake_stop(); //stop intaking

  chassis.set_turn_pid(-115, 60); //turn towards the goal
  chassis.wait_drive();

  pros::delay(250); //let triball settle


  pros::delay(400);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, TURN_SPEED);
  chassis.wait_drive();

  intake_in(450);

  chassis.set_drive_pid(21, DRIVE_SPEED, true); //drive towards the triball
  chassis.wait_drive();

  pros::delay(100);  //make sure to pick it up

  chassis.set_drive_pid(-12, DRIVE_SPEED, true); //reverse to avoid getting entangled
  chassis.wait_drive();

  pros::delay(200);

  intake_stop(); //stop intaking 

  chassis.set_turn_pid(-112, 70); //turn towards the goal
  chassis.wait_drive();

  pros::delay(250);

  pros::delay(400);

  intake_in(400);

  chassis.set_turn_pid(-165, DRIVE_SPEED); //turn towards pole
  chassis.wait_drive();

  chassis.set_drive_pid(35, DRIVE_SPEED, true); //drive towards pole
  chassis.wait_drive();
}

void AWPattempt(){
  intake_coast();
  WingL.set(false);
  WingR.set(false);

  WingL.set(true);

  intake_in(500);
  pros::delay(200);

  WingL.set(false);

  chassis.set_drive_pid(47, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_stop();

  chassis.set_turn_pid(82, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_until(15);

  //reset imu to 0 heading
  // chassis.reset_gyro(0);

  pros::delay(200);

  WingL.set(true);

  pros::delay(200);

  WingL.set(false);

  chassis.set_swing_pid(LEFT_SWING, 37, -DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-48, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(130, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-8, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 165, -DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-7, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(18, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 130, DRIVE_SPEED);
  chassis.wait_drive();

  WingR.set(true);

  chassis.set_drive_pid(4, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 85, DRIVE_SPEED);
  chassis.wait_drive();



  chassis.set_drive_pid(36.5, 120);
  pros::delay(300);
  WingR.set(false);
  intake_out(600);
  chassis.wait_drive();
}


void SuperSimpleAWP(){

  WingR.set(false);
  WingL.set(false);
  Blocker.set(false);
  SetStopDegree(2);
 
}



void SixBallMiddleMiddle(){
  WingR.set(false);
  WingL.set(false);
  Blocker.set(false);
  SetStopDegree(2);


}



void SixBallMiddleTop(){
  WingR.set(false);
  WingL.set(false);
  Blocker.set(false);
  SetStopDegree(2);

}



void CloseMiddleOver(){
  WingR.set(false);
  WingL.set(false);
  Blocker.set(false);
  SetStopDegree(2);


}



void CloseMiddleOverWait(){
  WingR.set(false);
  WingL.set(false);
  Blocker.set(false);
  SetStopDegree(2);


}



void CloseTopMiddle(){
  WingR.set(false);
  WingL.set(false);
  Blocker.set(false);
  SetStopDegree(2);


}



void CloseMiddleOverTouchHang(){
  WingR.set(false);
  WingL.set(false);
  Blocker.set(false);
  SetStopDegree(2);


}

void Auton_Skills(){
  intake_coast();
  WingL.set(false);
  WingR.set(false);


  cata_move(12000);
  pros::delay(1000); //30000
  intake_out(200);
  pros::delay(1000); //4000 msec
  intake_in(400);
  pros::delay(1000);

  cata_move(0);

  pros::delay(200);
  chassis.reset_gyro(0);
  pros::delay(200);

  chassis.set_drive_pid(-2, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_out(200);

  chassis.set_turn_pid(37, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-86, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED, true);
  chassis.wait_drive();


  chassis.set_swing_pid(ez::LEFT_SWING, -40, -DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_until(4);

  chassis.set_turn_pid(-60, TURN_SPEED); 
  chassis.wait_drive();

  chassis.set_turn_pid(-147, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-50, DRIVE_SPEED, true);
  chassis.wait_until(26);

  pros::delay(400);

  //reset imu to 0 heading
  chassis.reset_gyro(0);

  pros::delay(200);

  WingL.set(true);

  chassis.set_turn_pid(-55, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(50, DRIVE_SPEED, true);
  chassis.wait_until(14);

  chassis.set_swing_pid(LEFT_SWING, 0, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(30, DRIVE_SPEED, true);
  chassis.wait_until(10);

  WingR.set(false);
  WingL.set(false);

  chassis.set_drive_pid(-20, 127, true);
  chassis.wait_drive();

  WingL.set(true);

  chassis.set_drive_pid(50, 127);
  chassis.wait_until(18);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  WingL.set(false);
  WingR.set(false);

  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 90, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-38, DRIVE_SPEED);
  chassis.wait_drive();

  WingR.set(true);
  WingL.set(true);

  chassis.set_turn_pid(60, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(20, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 0, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(50, DRIVE_SPEED, true);
  chassis.wait_until(14);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  WingR.set(false);
  WingL.set(false);

  chassis.set_drive_pid(-20, 127, true);
  chassis.wait_drive();

  WingR.set(true);
  WingL.set(true);

  chassis.set_drive_pid(50, 127);
  chassis.wait_until(18);

  pros::delay(400);

  chassis.set_drive_pid(-30, DRIVE_SPEED, true);
  chassis.wait_drive();

  WingR.set(false);
  WingL.set(false);
}

void Tuning(){


}
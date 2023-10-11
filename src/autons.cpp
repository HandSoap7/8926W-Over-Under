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


const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
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

void OLDdefault_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void one_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void two_mogo_constants() {
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
  /*while(chassis.get_mode() == ez::DRIVE){
  printf("%i,%i,%i,%i,%i\n", counter, chassis.right_sensor(), chassis.right_velocity(), chassis.left_sensor(), chassis.left_velocity() );
  pros::delay(util::DELAY_TIME);
  counter = counter + 20;
  }*/
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



///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();
}



///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}



///
// Auto that tests everything
///
void combining_movements() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Interference example
///
void tug (int attempts) {
  for (int i=0; i<attempts-1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered) {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees. 
// If interfered, robot will drive forward and then attempt to drive backwards. 
void interfered_example() {
 chassis.set_drive_pid(24, DRIVE_SPEED, true);
 chassis.wait_drive();

 if (chassis.interfered) {
   tug(3);
   return;
 }

 chassis.set_turn_pid(90, TURN_SPEED);
 chassis.wait_drive();
}



// . . .
// Make your own autonomous functions here!
// . . .

void SixBallOffensive() {
  //basic initialization
  //pros::Task Reload_Limit(catapult_reload_limit_task);
  intakeHold();
  WingL.set(false);
  WingR.set(false);
  intakeActuate.set(false);

  //start of auton
  intakeActuate.set(true);
  intake_in(600);

  //drive to the first ball
  chassis.set_drive_pid(3.5, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_stop();

  //backup toawrds matchloader
  chassis.set_drive_pid(-36, DRIVE_SPEED, true);
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

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();
  
  intake_coast();

  chassis.set_turn_pid(120, TURN_SPEED);
  chassis.wait_drive();

  intake_out(300);

  pros::delay(250);

  intake_stop();

  chassis.set_turn_pid(23, TURN_SPEED);
  chassis.wait_drive();

  intake_hold();

  intake_in(350);

  //drive towards triball
  chassis.set_drive_pid(27, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_stop();

  //backup, turn towards goal, and out take
  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_coast();

  chassis.set_turn_pid(145, TURN_SPEED);
  chassis.wait_drive();

  intake_out(500);

  pros::delay(250);

  //turn towards middle triball
  chassis.set_turn_pid(75, TURN_SPEED);
  chassis.wait_drive();

  intakeHold();


  //drive towards middle triball while intaking
  intake_in(350);

  chassis.set_drive_pid(27, DRIVE_SPEED, true);
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
  intakeActuate.set(false);
  chassis.wait_drive();

  //go back and forth to knock triballs in

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
  intakeActuate.set(false);

  //start of auton
  intakeActuate.set(true);    
  intake_in(400);    //outtake immediately to get rid of preloaded alliance ball

  chassis.set_drive_pid(45, DRIVE_SPEED, true);  //drive about 2/5 the way there
  chassis.wait_drive();

  pros::delay(400);

  chassis.set_drive_pid(-10, 50, true); //reverse to avoid getting entangled
  chassis.wait_drive();

  intake_stop(); //stop intaking

  chassis.set_turn_pid(-115, 60); //turn towards the goal
  chassis.wait_drive();

  pros::delay(250); //let triball settle

  catapult_fire(); //HOPEFULLY fire into goal

  pros::Task Reload_Limit(catapult_reload_limit_task); //start reloading catapult

  chassis.set_swing_pid(ez::RIGHT_SWING, 37, TURN_SPEED);
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

  catapult_fire(); //HOPEFULLY fire into goal

  intake_in(400);

  chassis.set_turn_pid(-165, DRIVE_SPEED); //turn towards pole
  chassis.wait_drive();

  chassis.set_drive_pid(35, DRIVE_SPEED, true); //drive towards pole
  chassis.wait_drive();
}

void AWPattempt(){

}
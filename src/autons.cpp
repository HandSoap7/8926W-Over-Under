#include "autons.hpp"
#include "catapult.hpp"
#include "intake.hpp"
#include "main.h"
#include "pistons.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <set>
#include "definitions.hpp"


//LEM
#include "lemlib/api.hpp"


//Paths for Pure Pursuit

//ASSET(testyty.txt);
//ASSET(bar_txt);
//ASSET(sillyskills_txt);






///////////////////////////////////////////////////////////////////////

//LEM LIBRARY

///////////////////////////////////////////////////////////////////////



//Drive motors
pros::Motor left_front_motor(7, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed
pros::Motor left_middle_motor(17, pros::E_MOTOR_GEARSET_06, true); // port 2, blue gearbox, not reversed
pros::Motor left_back_motor(8, pros::E_MOTOR_GEARSET_06, true); // port 3, blue gearbox, not reversed
pros::Motor right_front_motor(16, pros::E_MOTOR_GEARSET_06, false); // port 4, blue gearbox, reversed
pros::Motor right_middle_motor(3, pros::E_MOTOR_GEARSET_06, false); // port 3, blue gearbox, reversed
pros::Motor right_back_motor(1, pros::E_MOTOR_GEARSET_06, false); // port 4, blue gearbox, reversed



// Motor groups
pros::MotorGroup LeftyMotors({left_front_motor, left_back_motor});
pros::MotorGroup RightyMotors({right_front_motor, right_back_motor});


//Drivetrain constructor
lemlib::Drivetrain_t drivetrain {
    &LeftyMotors, // left drivetrain motors
    &RightyMotors, // right drivetrain motors
    10, // track width
    2.75, // wheel diameter
    450, // wheel rpm
    4 //Chase Power
};


//Future ODOM
pros::Rotation rot1(21, false); // 
pros::Rotation rot2(21, false); // 

// uses "enc" as the encoder. 2.75" wheel diameter, 4.3" offset from tracking center, 1:1 gear ratio
lemlib::TrackingWheel Horizontal_tracking(&rot1, 2.75, 4.3, 1);
// uses "enc" as the encoder. 2.75" wheel diameter, 4.3" offset from tracking center, 1:1 gear ratio
lemlib::TrackingWheel Vertical_tracking(&rot2, 2.75, 4.3, 1);

// inertial sensor
pros::Imu Inertial_sensy(15); // port 2
 
// odometry struct
lemlib::OdomSensors_t sensors {
    nullptr, // horizontal tracking wheel
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    nullptr, // vertical tracking wheel
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &Inertial_sensy // inertial sensor
};


// forward/backward PID
lemlib::ChassisController_t lateralController {
    12, // kP
    35, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    10 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    8, // kP 4
    65, // kD 40
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    10 // slew rate
};




// create the chassis
lemlib::Chassis LemChassis(drivetrain, lateralController, angularController, sensors);


//Screen for printing odom values

void LemScreen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = LemChassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}


void ChassisCoast(){
  LeftyMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  RightyMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}

void ChassisHold(){
  LeftyMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
  RightyMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
}




/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


/*

void SixBallOffensive() {
  //basic initialization
  //pros::Task Reload_Limit(catapult_reload_limit_task);
  intakeHold();
  WingL.set(false);
  WingR.set(false);

  //start of auton
  intake_in(600);

  pros::delay(250);

  //drive to the first ball
  chassis.set_drive_pid(5, DRIVE_SPEED, true);
  chassis.wait_drive();

  //backup toawrds matchloader
  chassis.set_drive_pid(-35, DRIVE_SPEED, true);
  chassis.wait_drive();
  
  intake_stop();

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
  chassis.set_drive_pid(7, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(4, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();
  
  intake_coast();

  chassis.set_turn_pid(130, TURN_SPEED);
  chassis.wait_drive();

  intake_out(250);

  pros::delay(300);

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

  intake_out(300);

  pros::delay(150);

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

void AWPattempt(){
  intake_coast();
  WingL.set(false);
  WingR.set(false);
  TaskState(true);

  WingR.set(true);

  intake_in(500);
  pros::delay(200);

  WingR.set(false);

  chassis.set_drive_pid(47, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake_stop();

  chassis.set_turn_pid(82, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_until(11);

  //reset imu to 0 heading
  // chassis.reset_gyro(0);

  pros::delay(200);

  WingL.set(true);

  pros::delay(300);

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

  chassis.set_drive_pid(10, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 130, DRIVE_SPEED);
  chassis.wait_drive();

  WingR.set(true);

  chassis.set_drive_pid(6, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 85, DRIVE_SPEED);
  chassis.wait_drive();



  chassis.set_drive_pid(36.75, 120);
  pros::delay(300);
  WingR.set(false);
  intake_out(600);
  chassis.wait_drive();
}

void HighScoringShooting(){
  intake_coast();
  WingL.set(false);
  WingR.set(false);
  TaskState(true);

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
  chassis.wait_until(12);

  //reset imu to 0 heading
  // chassis.reset_gyro(0);

  pros::delay(200);

  WingL.set(true);

  pros::delay(300);

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

  chassis.set_drive_pid(-15, DRIVE_SPEED, true);
  chassis.wait_until(-5);

  chassis.set_drive_pid(18, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 130, DRIVE_SPEED);
  chassis.wait_drive();

  WingR.set(true);

  chassis.set_drive_pid(4, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 88, DRIVE_SPEED);
  chassis.wait_drive();



  chassis.set_drive_pid(35.5, 120);
  pros::delay(300);
  WingR.set(false);
  intake_out(600);
  chassis.wait_drive();
}

void Auton_Skills(){
  intake_coast();
  WingL.set(false);
  WingR.set(false);
  TaskState(false);

  cata_move(12000);
  pros::delay(30000); //30000
  intake_out(200);
  pros::delay(4000); //4000 msec
  intake_in(400);
  pros::delay(1000);

  catapult_stop();

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
*/

void SixBall(){


}


void AWP(){

  WingR.set(false);
  WingL.set(false);
  Blocker.set(false);
  SetStopDegree(2);
  LemChassis.setPose(-53, -52, 315); // X: 5.2, Y: 10.333, Heading: 87


  WingL.set(true);
  

  LemChassis.moveTo(-42, 18, 315, 1000, 120);
  WingL.set(false);

  LemChassis.moveTo(-40, -58, 180, 9999, true);





}



void EliminationClose(){


}



void Auton_Skills(){


}




void LemTest(){

  //Basic start of LEM auton

  //pros::lcd::initialize(); // initialize brain screen
  //pros::Task screenTask(LemScreen); // create a task to print the position to the screen

  LemChassis.setPose(0.0, 0, 0); // X: 5.2, Y: 10.333, Heading: 87

  ////////////////////////////////////////////////////////////////////////////////////////

  //LemChassis.moveTo(0, 20, 0, 1000, 120); // move to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50

  LemChassis.turnTo(5, 5, 1000, 100, false); // turn to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50

}
#include "autons.hpp"
#include "puncher.hpp"
#include "intake.hpp"
#include "main.h"
#include "pistons.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <set>
#include "definitions.hpp"
#include "gif-pros/gifclass.hpp"

//LEM
#include "lemlib/api.hpp"


//Paths for Pure Pursuit

ASSET(ClosePushOver1_txt);
ASSET(ClosePushOver2_txt);
ASSET(FarMiddleTop_txt);






///////////////////////////////////////////////////////////////////////

//LEM LIBRARY

///////////////////////////////////////////////////////////////////////



//Drive motors
pros::Motor left_front_motor(10, pros::E_MOTOR_GEARSET_06, true); // port 7, blue gearbox, reversed
pros::Motor left_middle_motor(7, pros::E_MOTOR_GEARSET_06, true); // port 17, blue gearbox, reversed
pros::Motor left_back_motor(2, pros::E_MOTOR_GEARSET_06, true); // port 8, blue gearbox, reversed
pros::Motor right_front_motor(20, pros::E_MOTOR_GEARSET_06, false); // port 16, blue gearbox, not reversed
pros::Motor right_middle_motor(17, pros::E_MOTOR_GEARSET_06, false); // port 3, blue gearbox, not reversed
pros::Motor right_back_motor(11, pros::E_MOTOR_GEARSET_06, false); // port 1, blue gearbox, not reversed



// Motor groups
pros::MotorGroup LeftyMotors({left_front_motor, left_middle_motor, left_back_motor});
pros::MotorGroup RightyMotors({right_front_motor, right_middle_motor, right_back_motor});


//Drivetrain constructor
lemlib::Drivetrain drivetrain {
    &LeftyMotors, // left drivetrain motors
    &RightyMotors, // right drivetrain motors
    11.6, // track width
    lemlib::Omniwheel::NEW_325, // wheel diameter
    450, // wheel rpm
    4 //Chase Power
};


//Future ODOM
pros::Rotation rotVert(16, false); // 
pros::Rotation rotHoriz(12, false); // 

// uses "enc" as the encoder. 2.75" wheel diameter, 4.3" offset from tracking center, 1:1 gear ratio
lemlib::TrackingWheel Horizontal_tracking(&rotVert, lemlib::Omniwheel::NEW_275_HALF, -2.75, 1);
// uses "enc" as the encoder. 2.75" wheel diameter, 4.3" offset from tracking center, 1:1 gear ratio
lemlib::TrackingWheel Vertical_tracking(&rotHoriz, lemlib::Omniwheel::NEW_275_HALF, .01, 1);

// inertial sensor
pros::Imu Inertial_sensy(14); // port 2
 
// odometry struct
lemlib::OdomSensors sensors {
    nullptr, // horizontal tracking wheel
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    nullptr, // vertical tracking wheel
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &Inertial_sensy // inertial sensor
};


// forward/backward PID
lemlib::ControllerSettings lateralController {
    10, // kP
    0, // kI
    30, // kD
    0, // AntiWindup
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    20 // slew rate
};




// turning PID
lemlib::ControllerSettings angularController {
    2, // kP 8 14
    0, // kI
    10, // kD 65 115
    0, // AntiWindup
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};


/**/
//TUNING CONTROLLERS
/*
lemlib::ChassisController_t lateralController {
    10, // kP 10
    30, // kD 30
    0, // smallErrorRange
    100, // smallErrorTimeout
    0, // largeErrorRange
    500, // largeErrorTimeout
    10 // slew rate
};
*/
/*
// turning PID
lemlib::ChassisController_t angularController {
    4, // kP 4
    40, // kD 40
    0, // smallErrorRange0
    100, // smallErrorTimeout
    0, // largeErrorRange
    500, // largeErrorTimeout
    10 // slew rate
};
*/


// create the chassis (MAIN FUNCTION)
lemlib::Chassis LemChassis(drivetrain, lateralController, angularController, sensors, 0);





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










///////////////////////////////////////////////////////////////////////

//EZ TEMPLATE

///////////////////////////////////////////////////////////////////////

int DRIVE_SPEED = 120;
int TURN_SPEED = 115;
int SWING_SPEED = 110;










/////////////////////////////////////////////////////////////////////////////////////////////

//                                  Autonomous Routes                                     //
 
/////////////////////////////////////////////////////////////////////////////////////////////


void SuperSimpleAWP(){

}



void SixBallCounterMiddle(){
  
  }



void SixBallCounterTop(){
  LemChassis.setPose(-53, -52, 315); // X: 5.2, Y: 10.333, Heading: 87

}


void SixBallSafe(){
  //basic initialization
  //pros::Task Reload_Limit(catapult_reload_limit_task);
  intakeHold();
  HorizWingL.set(false);
  HorizWingR.set(false);

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

  SideHang.set(false);
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


void ClosePushOver(){
  LemChassis.setPose(-53, -52, 315); // X: 5.2, Y: 10.333, Heading: 87


}



void CloseDisrupt(){

}



void Auton_Skills(){
  //printf("running skills/n");
  LemChassis.setPose(-53, -52, 315); // X: 5.2, Y: 10.333, Heading: 87

}



void LemTest(){

  //Basic start of LEM auton

  //pros::lcd::initialize(); // initialize brain screen
  //pros::Task screenTask(LemScreen); // create a task to print the position to the screen

  LemChassis.setPose(0.0, 0, 0); // X: 5.2, Y: 10.333, Heading: 87

  ////////////////////////////////////////////////////////////////////////////////////////

  //LemChassis.moveToPose(0, 10, 0, 99999, 127); // move to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50

  LemChassis.turnTo(30, 0, 99999, false); // turn to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50

}


void MakeAuton(){

  LemChassis.setPose(0.0, 0, 0); // Plain but can be configured
  
  LemScreen();

}
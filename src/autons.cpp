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












/////////////////////////////////////////////////////////////////////////////////////////////

//                                  Autonomous Routes                                     //
 
/////////////////////////////////////////////////////////////////////////////////////////////


void SuperSimpleAWP(){
  LemChassis.setPose(0, 0, 0); // X: 5.2, Y: 10.333, Heading: 87



  LemChassis.moveToPose(0, -8, 0, 2000);
  VertWingR.set(true); 
  LemChassis.waitUntilDone();

  LemChassis.moveToPose(0, -2, 0, 9999);
  LemChassis.waitUntilDone();


  LemChassis.turnTo(-22, 40, 1200); 
  LemChassis.waitUntilDone();

  VertWingR.set(false);

  LemChassis.moveToPose(-12, 15, -30, 9999);
  LemChassis.waitUntilDone();

  LemChassis.moveToPose(-28, 32, -42, 9999);
  LemChassis.waitUntilDone();
}



void SixBallCounterMiddle(){
  LemChassis.setPose(0, 0, 0); // X: 5.2, Y: 10.333, Heading: 87


  intake_in(500);
  LemChassis.moveToPose(-6, 45,-8, 9999);
  LemChassis.waitUntilDone();


  LemChassis.turnTo(9.5, 47, 9999);
  LemChassis.waitUntilDone();
  intake_out(600);
  pros::delay(350);

  intake_in(500);
  LemChassis.moveToPose(-25, 55, -90, 9999);
  LemChassis.waitUntilDone();
  pros::delay(50);

  LemChassis.moveToPose(10, 49, -266, 9999);
  LemChassis.waitUntil(16);
  intake_out(600);
  VertWingR.set(true);
  VertWingL.set(true);
  LemChassis.waitUntilDone();


  pros::delay(600);

  intake_in(500);
  VertWingR.set(false);
  VertWingL.set(false);
  

  LemChassis.moveToPose(-24, 32, -105, 9999);
  LemChassis.waitUntilDone();

  pros::delay(250);
  
  LemChassis.moveToPose(11, 8, -300, 9999);
  LemChassis.waitUntilDone();

  intake_out(600);  
  pros::delay(350);

  intake_in(450); 

  LemChassis.moveToPose(-32, -5, -85, 9999);
  LemChassis.waitUntilDone();

  pros::delay(350);

  LemChassis.moveToPose(3, -14, -275, 9999);
  LemChassis.waitUntilDone();

  LemChassis.moveToPose(17, -9, -315, 9999);
  LemChassis.waitUntilDone();

  LemChassis.turnTo(15, 50, 9999);
  LemChassis.waitUntilDone();

  LemChassis.moveToPose(30, 3, -355, 9999);
  LemChassis.waitUntilDone();

  LemChassis.moveToPose(30, 9.5, -355, 9999);
  LemChassis.waitUntilDone();

  LemChassis.moveToPose(30, 3, -355, 9999);
  LemChassis.waitUntilDone();

  }



void SixBallCounterTop(){
  LemChassis.setPose(-53, -52, 315); // X: 5.2, Y: 10.333, Heading: 87

}


void SixBallSafe(){
  LemChassis.setPose(-53, -52, 315); // X: 5.2, Y: 10.333, Heading: 87


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
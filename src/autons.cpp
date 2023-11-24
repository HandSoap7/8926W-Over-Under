#include "autons.hpp"
#include "catapult.hpp"
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

//ASSET(testyty.txt);
//ASSET(bar_txt);
//ASSET(sillyskills_txt);






///////////////////////////////////////////////////////////////////////

//LEM LIBRARY

///////////////////////////////////////////////////////////////////////



//Drive motors
pros::Motor left_front_motor(7, pros::E_MOTOR_GEARSET_06, true); // port 7, blue gearbox, reversed
pros::Motor left_middle_motor(17, pros::E_MOTOR_GEARSET_06, true); // port 17, blue gearbox, reversed
pros::Motor left_back_motor(8, pros::E_MOTOR_GEARSET_06, true); // port 8, blue gearbox, reversed
pros::Motor right_front_motor(16, pros::E_MOTOR_GEARSET_06, false); // port 16, blue gearbox, not reversed
pros::Motor right_middle_motor(3, pros::E_MOTOR_GEARSET_06, false); // port 3, blue gearbox, not reversed
pros::Motor right_back_motor(1, pros::E_MOTOR_GEARSET_06, false); // port 1, blue gearbox, not reversed



// Motor groups
pros::MotorGroup LeftyMotors({left_front_motor, left_middle_motor, left_back_motor});
pros::MotorGroup RightyMotors({right_front_motor, right_middle_motor, right_back_motor});


//Drivetrain constructor
lemlib::Drivetrain_t drivetrain {
    &LeftyMotors, // left drivetrain motors
    &RightyMotors, // right drivetrain motors
    11.6, // track width
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

/*
// forward/backward PID
lemlib::ChassisController_t lateralController {
    10, // kP
    30, // kD
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
*/

/**/
//TUNING CONTROLLERS

lemlib::ChassisController_t lateralController {
    10, // kP
    30, // kD
    0, // smallErrorRange
    100, // smallErrorTimeout
    0, // largeErrorRange
    500, // largeErrorTimeout
    10 // slew rate
};

// turning PID
lemlib::ChassisController_t angularController {
    8, // kP 4
    65, // kD 40
    0, // smallErrorRange
    100, // smallErrorTimeout
    0, // largeErrorRange
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






/////////////////////////////////////////////////////////////////////////////////////////////

//                                  Autonomous Routes                                     //
 
/////////////////////////////////////////////////////////////////////////////////////////////


void SuperSimpleAWP(){

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



void SixBallMiddleMiddle(){


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
  printf("running skills/n");

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
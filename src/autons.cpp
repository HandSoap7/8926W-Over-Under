#include "autons.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "catapult.hpp"
#include "intake.hpp"
#include "main.h"
#include "pistons.hpp"
#include "pros/rtos.hpp"
#include <set>

//LEM
#include "lemlib/api.hpp"






// LEM LIBRARY

//Drive motors
pros::Motor left_front_motor(1, pros::E_MOTOR_GEARSET_06, false); // port 1, blue gearbox, not reversed
pros::Motor left_middle_motor(2, pros::E_MOTOR_GEARSET_06, false); // port 2, blue gearbox, not reversed
pros::Motor left_back_motor(3, pros::E_MOTOR_GEARSET_06, false); // port 3, blue gearbox, not reversed
pros::Motor right_front_motor(4, pros::E_MOTOR_GEARSET_06, true); // port 4, blue gearbox, reversed
pros::Motor right_middle_motor(3, pros::E_MOTOR_GEARSET_06, true); // port 3, blue gearbox, reversed
pros::Motor right_back_motor(4, pros::E_MOTOR_GEARSET_06, true); // port 4, blue gearbox, reversed


// Motor groups
pros::MotorGroup LeftyMotors({left_front_motor, left_back_motor});
pros::MotorGroup RightyMotors({right_front_motor, right_back_motor});


//Drivetrain constructor
lemlib::Drivetrain_t drivetrain {
    &LeftyMotors, // left drivetrain motors
    &RightyMotors, // right drivetrain motors
    10, // track width
    2.75, // wheel diameter
    450 // wheel rpm
};


//Future ODOM
pros::Rotation rot1(21, false); // 
pros::Rotation rot2(21, false); // 

// uses "enc" as the encoder. 2.75" wheel diameter, 4.3" offset from tracking center, 1:1 gear ratio
lemlib::TrackingWheel Horizontal_tracking(&rot1, 2.75, 4.3, 1);
// uses "enc" as the encoder. 2.75" wheel diameter, 4.3" offset from tracking center, 1:1 gear ratio
lemlib::TrackingWheel Vertical_tracking(&rot2, 2.75, 4.3, 1);

// inertial sensor
pros::Imu Inertial_sensy(21); // port 2
 
// odometry struct
lemlib::OdomSensors_t sensors {
    &Horizontal_tracking, // horizontal tracking wheel
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &Vertical_tracking, // vertical tracking wheel
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &Inertial_sensy // inertial sensor
};


// forward/backward PID
lemlib::ChassisController_t lateralController {
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};




// create the chassis
lemlib::Chassis LemChassis(drivetrain, lateralController, angularController, sensors);




void LemCalibrate(){
    LemChassis.calibrate();
}



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
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.50, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.50, 0, 5, 0);
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
<<<<<<< Updated upstream
=======
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
>>>>>>> Stashed changes
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

void LemTest(){

  //Basic start of LEM auton

  pros::lcd::initialize(); // initialize brain screen
  pros::Task screenTask(LemScreen); // create a task to print the position to the screen


  LemChassis.setPose(5.2, 10.333, 87); // X: 5.2, Y: 10.333, Heading: 87

  ////////////////////////////////////////////////////////////////////////////////////////

  LemChassis.moveTo(10, 0, 1000, 50); // move to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50

  LemChassis.turnTo(10, 0, 1000, false, 50); // turn to the point (10, 0) with a timeout of 1000 ms, and a maximum speed of 50

}
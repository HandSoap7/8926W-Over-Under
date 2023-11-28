#include "main.h"
#include "autons.hpp"
#include "definitions.hpp"
#include "catapult.hpp"
#include "intake.hpp"
#include "pistons.hpp"
#include "lemlib/api.hpp"
#include "EZ-Template/auton.hpp"
#include "EZ-Template/sdcard.hpp"
#include "gif-pros/gifclass.hpp"


//#include "lemlib/api.hpp"


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-7, -8, 19}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{1, 3 , 15}

  // IMU Port
  ,14

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.746

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.35

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);





/**
 * Runs initialization code. This occurs as soon as the program is started.
 *  
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

  ez::print_WIGGLE_template();
  // print odom values to the brain
  //pros::Task odomScreenTask(LemScreen);
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!



  ez::as::auton_selector.add_autons({
    Auton("1 AWP", SuperSimpleAWP),
    Auton("2 FAR MidMid", SixBallMiddleMiddle),
    Auton("3 FAR MidTop", SixBallMiddleTop),
    Auton("4 FAR Safe", SixBallSafe),
    Auton("4 CLOSE Over", CloseMiddleOver),
    Auton("5 CLOSE OverWait", CloseMiddleOverWait),
    Auton("6 CLOSE TopMid", CloseTopMiddle), 
    //Auton("Test File (NO COMP)", LemTest),
    //Auton("Odom tracking", MakeAuton),
    Auton("Auton Skills", Auton_Skills),
  });

  master.clear();
	pros::delay(50);
	master.set_text(0, 0, "      LETS GET");
	pros::delay(50);
	master.set_text(1, 0, "     THIS DUBYA");
  
  LemChassis.calibrate(); // calibrate sensors
  chassis.initialize();
  ez::as::auton_selector.ImuInitializeGif(3300, "/usd/AircraftTakeoff.gif");
  //ez::as::auton_selector.ImuInitializeGif(5250, "/usd/NeverBackDown.gif"); //Sarah

  //printf("imu initialized and startup gif shown\n")
  ez::as::initialize( "/usd/whynotshine.gif", "/usd/WiggleMain.gif");
  //ez::as::initialize( "/usd/WiggleAuton.gif", "/usd/SFWdancingdog.gif");    //Sarah
  pros::screen::erase();

  master.clear();
	pros::delay(50);
  master.set_text(0, 0, ez::as::auton_selector.Autons[ez::as::auton_selector.selected_auton].Name);

  pros::delay(200); // Wait for auton selector to finish

  pros::Task Reload_Rotation(catapult_reload_rotation_task);
  
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    
    // print odom values to the brain
    ///*
    pros::lcd::initialize(); // initialize brain screen
    pros::Task screenTask([&]() {
        while (true) {
            lemlib::Pose pose = LemChassis.getPose(); // get chassis position
            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Theta: %f", pose.theta);
            pros::delay(20);
        }
    }); //*/
  ChassisCoast();
  SuperSimpleAWP();
  //ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void opcontrol() {
  // This is preference to what you like to drive on.
  // We use coast because it increases the time before motor burnout
   
   uint32_t counterVar = 0; 

   ChassisCoast(); //Sets all drivetrain motors to coast (low friction)

   intake_hold(); // Sets the intake to coast mode (no brake)

   SetStopDegree(1); //Set cata stop to intake blocking (best for intaking)

   OdomRetraction.set(true); //Retract the horizontal odom wheel
   Blocker.set(false); //Lower the blocker
   AuxHang.set(false); //Don't deploy auxHang
   WingL.set(false); //Retract the left wing
   WingR.set(false); //Retract the right wing

   while (true) {

    /////////////////////////////////////////////////////////////
    
    //Drive Code

    //////////////////////////////////////////////////////////////
   
    // get left y and right y positions
    int leftY = master.get_analog(LeftY);
    int rightY = master.get_analog(RightY);
    int rightX = master.get_analog(RightX);

    //Willy Drive
    LemChassis.tank(leftY, rightY, 1);

    //Sarah Drive
    //LemChassis.arcade(leftY, rightX, 4);

    //Test Drive
    //LemChassis.curvature(leftY, rightX, 2.5);
    


    //////////////////////////////////////////////////////////////

    //Catapult code

    //////////////////////////////////////////////////////////////

    //Catapult fires if R1 is being pressed
    if (master.get_digital(R1)) {
      FastFireState(true);
    }

    //Catapult switches to Hang Mode
    else if (master.get_digital(A)) {
      SetStopDegree(3);
    }

    else if (master.get_digital(X)) {
      SetStopDegree(2);
    }

    //If needed to switch back to Normal Cata Rack Position
    else if (master.get_digital(B)) {
      SetStopDegree(1);
    }

    //Last Resort Cata Cut off
    else if (master.get_digital(Down) && master.get_digital(B) && master.get_digital(Left) && master.get_digital(A)) { 
      ManualOverrideState(true);
      cata_move(0);
    }

    else {
     FastFireState(false);

    }




    /////////////////////////////////////////////////////////////
    
    //Intake Code

    //////////////////////////////////////////////////////////////

    //intake into the robot if L2 is being pressed
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake_in(600);
      //pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "."); 
    }

    //intake into the robot if R2 is being pressed
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
     intake_out(600);
    }

    // intake stops if neither L1 or L2 are being pressed
    else {
      intake_stop();
        }


    /////////////////////////////////////////////////////////////

    //Pneumatic Code

    //////////////////////////////////////////////////////////////


    WingL.button(master.get_digital_new_press(Right));
    //WingL.button(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT));
    WingR.button(master.get_digital_new_press(Y));


    Blocker.button(master.get_digital_new_press(L1));

    AuxHang.button(master.get_digital_new_press(Up));

    pros::delay(15); // This is used for timer calculations!
  }
}

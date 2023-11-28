#include "main.h"
#include "EZ-Template/auton.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "display/lv_objx/lv_btnm.h"
#include "display/lv_objx/lv_imgbtn.h"
#include "catapult.hpp"
#include "intake.hpp"
#include "pistons.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <sys/types.h>
#include "definitions.hpp"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-7, -8, -20}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{1, 3, 15}

  // IMU Port
  ,14

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.745

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.345

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
  // Print our branding over your terminal :D
  ez::print_WIGGLE_template();

  // Initialize the catapult
  
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.


  // Start the catapult reload tasks
  //pros::Task Reload_Rotation(catapult_reload_rotation_task);

  pros::Task Reload_Rotation(catapult_reload_rotation_task);

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(2.5, 7.5); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("1 AWP", SuperSimpleAWP),
    Auton("2 FAR MidMid", SixBallMiddleMiddle),
    //Auton("3 FAR MidTop", SixBallMiddleTop),
    Auton("4 FAR Safe", SixBallSafe),
    //Auton("5 CLOSE Over", CloseMiddleOver),
    Auton("6 CLOSE OverWait", CloseMiddleOverWait),
    // Auton("7 CLOSE TopMid", CloseTopMiddle), 
    //Auton("Tuning", Tuning),
    Auton("Auton Skills", Auton_Skills),
  });

  master.clear();
	pros::delay(50);
	master.set_text(0, 0, "      YOU HAVE");
	pros::delay(50);
	master.set_text(1, 0, "     NO ENEMIES");

	// Initialize chassis and auton selector
	chassis.initialize(3500, "/usd/AircraftTakeoff.gif");
	ez::as::initialize("/usd/whynotshine.gif", "/usd/WiggleMain.gif");

	// Clear the LCD for the auton selector
	pros::screen::erase();

	master.clear();
	pros::delay(50);
	master.set_text(0, 0, ez::as::auton_selector.Autons[ez::as::auton_selector.selected_auton].Name);

	pros::delay(200); // Wait for auton selector to finish
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
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(pros::E_MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.


  //SuperSimpleAWP();

  // Run the selected auton
  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
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

  //Weird code our drivers don't use
  // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
  // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade
  // chassis.arcade_standard(ez::SINGLE); // Standard single arcade

void opcontrol() {
  // This is preference to what you like to drive on.
  // We use coast because it increases the time before motor burnout
   uint32_t counterVar = 0; 

   chassis.set_drive_brake(pros::E_MOTOR_BRAKE_COAST); // Sets the drive to coast mode (no brake)

   intake_coast(); // Sets the intake to coast mode (no brake)

   SetStopDegree(1); //Set cata stop to intake blocking (best for intaking)

   OdomRetraction.set(true); //Retract the horizontal odom wheel
   Blocker.set(false); //Lower the blocker
   AuxHang.set(false); //Don't deploy auxHang
   WingL.set(false); //Retract the left wing
   WingR.set(false); //Retract the right wing

   //Gif gif("/usd/WiggleMain.gif", lv_scr_act()); // Create a gif object

   while (true) { 

    /////////////////////////////////////////////////////////////
    
    //Drive Code

    //////////////////////////////////////////////////////////////
    

    chassis.tank(); // Will Drive
   
    //chassis.arcade_standard(ez::SPLIT); // Sarah Drive


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
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake_in(600);
      //pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "."); 
    }

    //intake into the robot if R2 is being pressed
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
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

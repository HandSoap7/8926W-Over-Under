#include "main.h"
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
#include "pros/rtos.hpp"
#include <sys/types.h>

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-2,-8,9}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{13,-16,18}

  // IMU Port
  ,21

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.75

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,0.75

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
  ez::print_ez_template();

  // Initialize the catapult
  
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.


  // Start the catapult reload tasks
  //pros::Task Reload_Rotation(catapult_reload_rotation_task);
  pros::Task Reload_Limit(catapult_reload_limit_task);


  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(2.5, 5.5); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Example Drive\n\nDrive forward and come back.", drive_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
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
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

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
   chassis.set_drive_brake(MOTOR_BRAKE_COAST);
   uint32_t counterVar = 0; 


   bool MatchLoadSpam = false;

   while (true) {

    //drive code using ez template

    //chassis.tank(); // Tank control for Will (match driver)
    chassis.arcade_standard(ez::SPLIT); // Standard split arcade for Sarah (skills driver)
    


    intake_coast(); // Sets the intake to coast mode (no brake)


    if(pros::E_CONTROLLER_DIGITAL_DOWN & pros::E_CONTROLLER_DIGITAL_B){
      MatchLoadSpam = true;
    }
    else{
      MatchLoadSpam = false;
    }

    if(MatchLoadSpam == false){
    //catapult code

    // When a new press is detected on R1, the catapult will override the reload task and move a little more, deactivting the slip gear and releases the catapult
    // This also ensures if accidentally pressed while reloading, the catapult will continue to reload
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      catapult_fire();                                       //catapult_reload_limit_task(); // Reloads the catapult
      pros::Task Reload_Limit(catapult_reload_limit_task);
    } else {
      // catapult reload task is already running so it will automatically override the catapult_fire velocity of 0 after the void ends
    } 

    }

    if(MatchLoadSpam == true){
      RapidFire();
    }

    //intake code

    //intake into the robot if L1 is being pressed
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake_in(7500);
      pros::c::controller_rumble(CONTROLLER_MASTER, "."); 
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
     intake_out(11000);
    }
    
    else {
      intake_stop(); 
    }

    // intake stops if neither L1 or L2 are being pressed

    //pneumatic code
    WingL.button(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT));
    //WingL.button(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT));
    WingR.button(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y));

    intakeActuate.button(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2));

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

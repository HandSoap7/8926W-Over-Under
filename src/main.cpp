#include "main.h"
#include "EZ-Template/auton.hpp"
#include "EZ-Template/util.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "autons.hpp"
#include "display/lv_objx/lv_btnm.h"
#include "display/lv_objx/lv_imgbtn.h"
#include "puncher.hpp"
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
#include "lemlib/api.hpp"
#include "gif-pros/gifclass.hpp"


//#include "lemlib/api.hpp"


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-2, -7, -10}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{11, 17, 20}

  // IMU Port
  ,8

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.3333333

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

  pros::delay(500);

  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Sets default constants for PID and other things
  exit_condition_defaults(); // Sets default exit conditions for PID and other things


  ez::as::auton_selector.add_autons({
    //Auton("Odom tracking", MakeAuton),
    Auton("1 AWP", SuperSimpleAWP),
    Auton("2 SixBallSafe", SixBallSafe),
    Auton("Test File (NOT COMP)", LemTest),
    Auton("Auton Skills", Auton_Skills),
  });

  master.clear();
	pros::delay(50);
	master.set_text(0, 0, "      LETS GET");
	pros::delay(50);
	master.set_text(1, 0, "     THIS DUBYA");
  
  LemChassis.calibrate(); // calibrate sensors
  chassis.initialize(3300, "/usd/AircraftTakeoff.gif");
  //ez::as::auton_selector.ImuInitializeGif(5250, "/usd/NeverBackDown.gif"); //Sarah

  //printf("imu initialized and startup gif shown\n")
  ez::as::initialize( "/usd/whynotshine.gif", "/usd/WiggleMain.gif");
  //ez::as::initialize( "/usd/WiggleAuton.gif", "/usd/SFWdancingdog.gif");    //Sarah
  pros::screen::erase();

  master.clear();
	pros::delay(50);
  master.set_text(0, 0, ez::as::auton_selector.Autons[ez::as::auton_selector.selected_auton].Name);

  pros::delay(200); // Wait for auton selector to finish

  pros::Task Reload_Distance(puncher_reload_distance_task);
  
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
  chassis.set_drive_brake(MOTOR_BRAKE_COAST); // Set motors to hold.  This helps autonomous consistency.


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

   intake_coast(); // Sets the intake to coast mode (no brake)

   PistonHang.set(false); //Don't deploy Piston hang
   SideHang.set(false); //Don't deploy Side hang
   HorizWingL.set(false); //Retract the Horizontal left wing
   HorizWingR.set(false); //Retract the Horizontal right wing
   VertWingL.set(false); //Retract the Vertical left wing
   VertWingR.set(false); //Retract the Vertical right wing

   while (true) {

    /////////////////////////////////////////////////////////////
    
    //Drive Code

    //////////////////////////////////////////////////////////////
   
    // get left y and right y positions
    int leftY = master.get_analog(LeftY);
    int rightY = master.get_analog(RightY);
    int rightX = master.get_analog(RightX);

    //Willy Drive
    //LemChassis.tank(leftY, rightY, 1);
    chassis.tank();

    //Sarah Drive
    //LemChassis.arcade(leftY, rightX, 4);
    //chassis.arcade_standard(ez::SPLIT);

    //Test Drive
    //LemChassis.curvature(leftY, rightX, 2.5);
    


    //////////////////////////////////////////////////////////////

    //Puncher code

    //////////////////////////////////////////////////////////////

    //Puncher fires if R1 is being pressed
    if (master.get_digital(B)) {
      DistanceFromSensorState(true);
    }
    //Last Resort Puncher Cut off
    else if (master.get_digital(Down) && master.get_digital(B) && master.get_digital(Left) && master.get_digital(A)) { 
      ManualOverrideState(true);
    }

    else {
     DistanceFromSensorState(false);
    }




    /////////////////////////////////////////////////////////////
    
    //Intake Code

    //////////////////////////////////////////////////////////////

    //intake into the robot if L2 is being pressed
    if (master.get_digital(L2)) {
      intake_in(600);
      //pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "."); 
    }

    //intake into the robot if R2 is being pressed
    else if (master.get_digital(R2)) {
     intake_out(600);
    }

    // intake stops if neither L1 or L2 are being pressed
    else {
      intake_stop();
        }


    /////////////////////////////////////////////////////////////

    //Pneumatic Code

    //////////////////////////////////////////////////////////////

    //Horizontal Wings
    HorizWingL.button(master.get_digital_new_press(Right));
    HorizWingR.button(master.get_digital_new_press(Y));

    //Verical Wings
    VertWingL.button(master.get_digital_new_press(L1));
    VertWingR.button(master.get_digital_new_press(R1));

    //Piston and Side Hang
    PistonHang.button(master.get_digital_new_press(Up));
    SideHang.button(master.get_digital_new_press(X));


    pros::delay(15); // This is used for timer calculations!
  }
}

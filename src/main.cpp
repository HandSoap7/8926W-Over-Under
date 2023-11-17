#include "main.h"
#include "autons.hpp"
#include "definitions.hpp"
#include "display/lv_objx/lv_btnm.h"
#include "display/lv_objx/lv_imgbtn.h"
#include "catapult.hpp"
#include "intake.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pistons.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <sys/types.h>
#include "definitions.hpp"
#include "lemlib/api.hpp"


//#include "lemlib/api.hpp"


pros::Controller master(pros::E_CONTROLLER_MASTER); // master controller

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *  
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

  pros::lcd::initialize(); // initialize brain screen
  LemChassis.calibrate(); // calibrate sensors

  // print odom values to the brain
  pros::Task odomScreenTask(LemScreen);
    
  

  pros::Task Reload_Rotation(catapult_reload_rotation_task);

  pros::delay(500); // wait for sensors to calibrate
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

  void TestAuton();
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

   ChassisCoast();

   intake_coast(); // Sets the intake to coast mode (no brake)

   while (true) {
   
    // get left y and right y positions
    int leftY = master.get_analog(LeftY);
    int rightY = master.get_analog(RightY);
    int rightX = master.get_analog(RightX);

    //Willy Drive
    LemChassis.tank(leftY, rightY, 3);

    //Sarah Drive
    //LemChassis.arcade(leftY, rightX, 5);

    //Test Drive
    //LemChassis.curvature(leftY, rightX, 5);
    

    /*
     // When a new press is detected on R1, the catapult will override the reload task and move a little more, deactivting the slip gear and releases the catapult
    // This also ensures if accidentally pressed while reloading, the catapult will continue to reload
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      catapult_fire();                                       //catapult_reload_limit_task(); // Reloads the catapult
      pros::Task Reload_Limit(catapult_reload_limit_task);
    }

    //if DOWN and B are pressed, then cata goes into nonstop, rapid-fire, mode
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
      RapidFire();
    }

    //This is only used if rapid-fire is initiated
    else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
    //restart catapult task
    catapult_stop();
    TaskState(true);
    pros::Task Reload_Limit(catapult_reload_limit_task);
    }

    else {
      //do nothing
    }
    */
    //printf("MatchLoadSpam: %d\n", MatchLoadSpam);




    //intake code

    //intake into the robot if L1 is being pressed
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake_in(500);
      pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "."); 
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
     intake_out(600);
    }
    
    else {
      intake_stop(); 
    }

    // intake stops if neither L1 or L2 are being pressed

    //pneumatic code
    WingL.button(master.get_digital_new_press(Right));
    //WingL.button(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT));
    WingR.button(master.get_digital_new_press(Y));

    Blocker.button(master.get_digital(A) && master.get_digital(Left));

    pros::delay(10); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

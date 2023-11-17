#ifndef DEFINITIONS
#define DEFINITIONS
#include "main.h"

//Controller Inputs
#define L1 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1
#define L2 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2
#define R1 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1
#define R2 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2
#define A pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A
#define B pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B
#define X pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X
#define Y pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_Y
#define Up pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP
#define Down pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN
#define Right pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_RIGHT
#define Left pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_LEFT
#define LeftX pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_X
#define LeftY pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y
#define RightX pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X
#define RightY pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y


//Gearbox Definitions
#define TORQUEBOX pros::motor_gearset_e_t::E_MOTOR_GEARSET_36
#define REGBOX pros::motor_gearset_e_t::E_MOTOR_GEARSET_18
#define SPEEDBOX pros::motor_gearset_e_t::E_MOTOR_GEARSET_06

//Unit Definitions
#define DEGREES pros::rotation_units_e_t::E_DEGREES
#define ROTATIONS pros::rotation_units_e_t::E_ROTATIONS


//Motor Port Definitions
#define CATA_PORT 7 //Defines a macro for the catapult motor port and sets it to port 7
#define INTAKE_PORT 4 //Defines a macro for the intake motor port and sets it to port 4
#define LD1 8  //Defines a macro for the front left drive motor port and sets it to port 8 and sets the motor as reversed
#define LD2 9 //Defines a macro for the bottom back left drive motor port and sets it to port 9 and sets the motor as reversed
#define LD3 20 //Defines a macro for the top back left drive motor port and sets it to port 20
#define RD1 15 //Defines a macro for the front right drive motor port and sets it to port 15 
#define RD2 2 //Defines a macro for the bottom back right drive motor port and sets it to port 2 
#define RD3 1 //Defines a macro for the top back right drive motor port and sets it to port 1 and sets the motor as reversed

//Misc Port Definitions
#define EXPAND_PORT 6 //Defines a macro for the expansion sensor port and sets it to port 6
#define IMU_PORT 14 //Defines a macro for the gps sensor port and sets it to port 14





#endif /* DEFINITIONS */
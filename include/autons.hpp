#pragma once
#include "autons.hpp"
#include "lemlib/api.hpp"
#include "pros/misc.hpp"


extern lemlib::Chassis LemChassis;
extern pros::Imu Inertial_sensy;


//Wiggle autons
void SuperSimpleAWP();
void SixBallMiddleMiddle();
void SixBallMiddleTop();
void SixBallSafe();
void CloseMiddleOver();
void CloseMiddleOverWait();
void CloseTopMiddle();
void CloseMiddleOverTouchHang();
void Auton_Skills();


//LEM library
void LemScreen();

void default_constants();
void tuning_constants();
void exit_condition_defaults();


void ChassisCoast();
void ChassisHold();

void LemTest();
void MakeAuton();
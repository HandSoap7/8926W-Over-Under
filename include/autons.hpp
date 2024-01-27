#pragma once

#include "EZ-Template/drive/drive.hpp"

extern Drive chassis;

void drive_example();
void turn_example();

void default_constants();
void exit_condition_defaults();
void modified_exit_condition();

void fullForceMove(int time);

//Wiggle autons
void SuperSimpleAWP();
void SixBallMiddleMiddle();
void SixBallMiddleTop();
void SixBallSafe();
void CloseMiddleOver();
void CloseMiddleOverWait();
void CloseTopMiddle();
void CloseMiddleOverTouchHang();
void CloseMidRush();
void Auton_Skills();
void Driver_Skills_Preloads();


void EZ_Tune();
void SarahSkills();
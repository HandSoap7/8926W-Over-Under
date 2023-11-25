#pragma once

#include "main.h"

//Functions are written here to be used in other files when called (Ex. #include catapult.hpp )

void cata_move(int speed);

void catapult_reload_rotation_task(void* param); // void* param is a pointer to a void (nothing) that is used to pass data to the task
void catapult_reload_limit_task(void* param);    // this allows us to pass integers, strings, etc. to the task
// void catapult_init();
void catapult_fire();
void catapult_stop();
double catapult_get_rotation();
bool catapult_get_limit();
bool catapult_get_limit2();
void rotation_reset();
void RapidFire();
void TaskState(bool State);
void FastFireState(bool State);
void ManualOverrideState(bool State);
void SetStopDegree(int State);
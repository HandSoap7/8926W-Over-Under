#pragma once

#include "main.h"

//Functions are written here to be used in other files when called (Ex. #include catapult.hpp )

void cata_move(int speed);

void catapult_reload_rotation_task(void* param); // void* param is a pointer to a void (nothing) that is used to pass data to the task
void RapidFire();
void ManualOverride(bool state);
void SetStopDegree(int State);
void FastFireState(bool state);
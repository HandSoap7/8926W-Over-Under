#pragma once

#include "main.h"

//Functions are written here to be used in other files when called (Ex. #include catapult.hpp )
void puncher_move(int speed);



void puncher_reload_rotation_task(void* param); // void* param is a pointer to a void (nothing) that is used to pass data to the task
void puncher_reload_distance_task(void* param); // this allows us to pass integers, strings, etc. to the task



void FastFireState(bool State);
void ManualOverrideState(bool State);
void SetStopDegree(int State);
void DistanceFromSensorState(bool state);
void DeployIntake();
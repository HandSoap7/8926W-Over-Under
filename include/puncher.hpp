#pragma once

#include "main.h"

//Functions are written here to be used in other files when called (Ex. #include puncher.hpp )

void puncher_reload_rotation_task(void* param); // void* param is a pointer to a void (nothing) that is used to pass data to the task
void puncher_reload_limit_task(void* param);    // this allows us to pass integers, strings, etc. to the task
// void puncher_init();
void puncher_fire();
void puncher_stop();
double puncher_get_rotation();
bool puncher_get_limit();
bool puncher_get_limit2();
void rotation_reset();
#pragma once

#include "main.h"

// Functions are written here to be used in other files when called (Ex. #include intake.hpp)

void intake_in(int volt);
void intake_out(int volt);
void intake_stop();

void intake_hold();
void intake_coast();
#pragma once

#include "pistongroup.hpp"


// The first parameter is the port of the piston
// The boolean is the default state for the piston
// Ex. inline PistonGroup NameOfPiston('Port'(A-H), true/false)

// The default state should be what makes the robot in size
inline PistonGroup WingR('D', false);
inline PistonGroup WingL('H', false);
inline PistonGroup Blocker('C', false);


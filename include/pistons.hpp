#pragma once

#include "pistongroup.hpp"

// The boolean is the default state for the piston
// The default state should be what makes the robot in size
inline PistonGroup liftDeploy('A', false);
inline PistonGroup liftPullUp('B', false);

#pragma once

#include "pistongroup.hpp"


// The first parameter is the port of the piston
// The boolean is the default state for the piston
// Ex. inline PistonGroup NameOfPiston('Port'(A-H), true/false)




// The default state should be what makes the robot in size

//Horizontal Wings
inline PistonGroup HorizWingL('A', false);
inline PistonGroup HorizWingR('B', false);

//Vertical Wings
inline PistonGroup VertWingL('A', false);
inline PistonGroup VertWingR('A', false);


//              HANGS               //
//Side
inline PistonGroup SideHang('D', false);

//Piston
inline PistonGroup PistonHang('D', false);

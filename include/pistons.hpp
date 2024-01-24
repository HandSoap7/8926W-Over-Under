#pragma once

#include "pistongroup.hpp"


// The first parameter is the port of the piston
// The boolean is the default state for the piston
// Ex. inline PistonGroup NameOfPiston('Port'(A-H), true/false)




// The default state should be what makes the robot in size

//Horizontal Wings
inline PistonGroup HorizWingL('F', false);
inline PistonGroup HorizWingR('G', false);

//Vertical Wings
inline PistonGroup VertWingL('D', false);
inline PistonGroup VertWingR('H', false);


//              HANGS               //
//Side
inline PistonGroup SideHang('E', false);

//Piston
inline PistonGroup PistonHang('A', false);

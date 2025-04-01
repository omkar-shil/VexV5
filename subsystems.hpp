#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor intake(11);
inline pros::Motor arm(8);

// **Pneumatics**
inline pros::ADIDigitalOut clamp_piston('H');  // Clamp piston
inline pros::ADIDigitalOut doinker('F'); // Doinker piston

// **Controller**
inline pros::Controller master(pros::E_CONTROLLER_MASTER);

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

// Sensors
inline pros::Rotation armrotation(10);


// PID control for the arm
inline void armcontrol(int input) {
  arm.move(input);
}

inline ez::PID armpid{0.45, 0, 0, 0, "Arm"};

// Arm Control Variables
inline float armlevel = armrotation.get_position();
inline float idlearm = 359.91; // Degree of rotation when the arm is down and idle
inline float loadarm = 49.92; // Degree of rotation when the arm is to be loaded with a ring to score
inline float scorearm = 241.62; // Degree of rotation to score; the arm will go to this degree for both alliance and neutral wall stakes
inline int armpos = 0; // Level of the arm stored in an int; 0 is idle, 1 is load, and 2 is scoring

// Arm Control
void armscore() {
    // Changes the level of the arm; R2 increases and R2 decreases. The different levels are above
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        if (armpos == 0) { 
            arm.move_absolute(loadarm,127);
            armpos = 1;
        } 
        else if (armpos == 1) { 
            arm.move_absolute(scorearm,127);
            armpos = 2;
        }
        pros::delay(200); // Prevent multiple triggers
    }

    // Decrease arm level with R1
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        if (armpos == 2) { 
            arm.move_absolute(loadarm,127);
            armpos = 1;
        } 
        else if (armpos == 1) { 
            arm.move_absolute(idlearm,127);
            armpos = 0;
        }
        pros::delay(200); // Prevent multiple triggers
    }
}

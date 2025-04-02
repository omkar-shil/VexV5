/*#include "main.cpp"
#include "main.h"



float armlevel = armrotation.get_position();
float idlearm = 359.91; // Degree of rotation when the arm is down and idle
float loadarm = 49.92 // Degree of rotation when the arm is to be loaded with a ring to score
float scorearm = 241.62; // Degree of rotation to score; the arm will go to this degree for both alliance and neutral wall stakes
int armpos = 0; // Level of the arm stored in an int; 0 is idle, 1 is load, and 2 is scoring

void armscore() {
    // Changes the level of the arm; R2 increases and R2 decreases. The different levels are above
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        if (armpos == 0) { 
            arm.set_angle(loadarm);
            armpos = 1;
        } 
        else if (armpos == 1) { 
            arm.set_angle(scorearm);
            armpos = 2;
        }
        pros::delay(200); // Prevent multiple triggers
    }

    // Decrease arm level with R1
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        if (armpos == 2) { 
            arm.set_angle(loadarm);
            armpos = 1;
        } 
        else if (armpos == 1) { 
            arm.set_angle(idlearm);
            armpos = 0;
        }
        pros::delay(200); // Prevent multiple triggers
    }
}
*/
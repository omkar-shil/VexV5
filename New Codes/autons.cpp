#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 127;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}


///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .
void eliminationred(){
  chassis.pid_swing_set(ez::RIGHT_SWING,30_deg,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-85_in,127,true);
  chassis.pid_wait();
  clamp_piston.set_value(true);
  chassis.pid_turn_set(90_deg,100);
  chassis.pid_wait();
  rollers.move(127);
  intake.move(-110);
  chassis.pid_drive_set(50_in,127,true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(85_deg,100);
  chassis.pid_wait();
  chassis.pid_drive_set(50_in,127,true);
  chassis.pid_wait();
  chassis.pid_drive_set(-25_in,127,true);
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg,100);
  chassis.pid_wait();
  chassis.pid_drive_set(25_in,127,true);
  chassis.pid_wait();
  chassis.pid_drive_set(-25_in,127,true);
  chassis.pid_turn_set(90_deg,100);
  chassis.pid_drive_set(75_in,127,true);
  chassis.pid_wait();
}

void test(){
  chassis.pid_drive_set(1_in,127);
  chassis.pid_wait();
  chassis.pid_turn_set(360_deg,127);
  chassis.pid_wait();
}


void qualifyingblue(){
  // Starts 24 away from the alliance stake on the negative side, the IMU starts on the foam tile line with the outer set of wheels on the border of the right most tile. Rollers forwards 
 chassis.pid_turn_set(360_deg, TURN_SPEED, true);
 chassis.pid_wait_quick();
 chassis.pid_drive_set(24_in, 76, true);
 chassis.pid_wait();
 chassis.pid_turn_set(90_deg, TURN_SPEED, true);
 chassis.pid_wait();
 chassis.pid_drive_set(-6_in, DRIVE_SPEED, true);
 chassis.pid_wait();
 intake.move(-127);
 chassis.pid_wait();
 chassis.pid_wait();
 chassis.pid_wait_quick();
 chassis.pid_wait();
 intake.move(0);
 chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
 chassis.pid_wait_quick();
 chassis.pid_turn_set(-53_deg, TURN_SPEED, true);
 chassis.pid_wait_quick();
 chassis.pid_drive_set(-37.5_in, 66, true);
 chassis.pid_wait();
 chassis.pid_drive_set(-1.5_in, DRIVE_SPEED);
 clamp_piston.set(true);
 chassis.pid_wait();
 intake.move(-79);
 rollers.move(-127);
 chassis.pid_turn_set(182_deg, TURN_SPEED, true);
 chassis.pid_wait();
 chassis.pid_drive_set(25_in, DRIVE_SPEED);
 chassis.pid_wait_quick();
 chassis.pid_turn_set(80_deg, TURN_SPEED);
 chassis.pid_wait_quick();
 chassis.pid_drive_set(14.25, 63);
 chassis.pid_wait();
 chassis.pid_drive_set(-5.25_in, DRIVE_SPEED, false);
 chassis.pid_wait();
 chassis.pid_turn_set(10_deg, TURN_SPEED, true);
 chassis.pid_wait();
 chassis.pid_drive_set(5.25_in, DRIVE_SPEED, false);
 chassis.pid_wait();
 chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
 pros::delay(500);
 intake.move(0);
 rollers.move(0);
}
void bluenegative(){  
  chassis.pid_drive_set(-24_in,115,true);
  chassis.pid_wait();
  clamp_piston.set_value(true);
  pros::delay(750);
  chassis.pid_turn_set(-90_deg,90,true);
  chassis.pid_wait();
  rollers.move(127);
  intake.move(-110);
  chassis.pid_drive_set(24_in,127,true);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_turn_set(-80_deg,90,true);
}

void bluepositive(){
// Blue positive starts 32 inches away from the positive blue mobile goal in a straight line. Clamp forwards
  chassis.pid_drive_set(-20.5_in, 76, true);
  chassis.pid_wait_quick();
  clamp_piston.set(true);
  intake.move(-100);
  pros::delay(500);
  chassis.pid_drive_set(-5_in, 66, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, 76, true);
  clamp_piston.set(false);
  chassis.pid_wait_quick();
  rollers.move(-127);
  chassis.pid_drive_set(25_in, 67, true);
  chassis.pid_wait_quick();
  intake.move(0);
  chassis.pid_turn_set(-360_deg, 67, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-20_in, 50, true);
  chassis.pid_wait_quick();
  clamp_piston.set(true);
  intake.move(-100);
  pros::delay(750);
  intake.move(0);
  rollers.move(127);
  chassis.pid_drive_set(44_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-135_deg, 67, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-14.5_in, 67, true);
  chassis.pid_wait();
  clamp_piston.set(false);
  chassis.pid_drive_set(60_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
}
void skills(){
  chassis.pid_drive_set(-12_in, 63, false);
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_wait();
  chassis.pid_wait();
  chassis.pid_wait_quick();
  intake.move(0);
  chassis.pid_drive_set(24_in, 63, false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-90_deg, TURN_SPEED, false);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  clamp_piston.set(true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-5_in, 63, false);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(36_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-135_deg, TURN_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  clamp_piston.set(false);
  intake.move(0);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-45_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(48_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  clamp_piston.set(true);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, 63, true);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(36_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(36_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-48_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  clamp_piston.set(false);
  intake.move(0);
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-90_deg, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  intake.move(127);
  pros::delay(100);
  intake.move(0);
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  clamp_piston.set(true);
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(22.5_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  clamp_piston.set(false); 
  intake.move(0);
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-19_in, TURN_SPEED, true);
  chassis.pid_wait();
  clamp_piston.set(true);
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(-22.5, TURN_SPEED, true);
  intake.move(-127);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-36_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  clamp_piston.set(false);
  pros::delay(100);
  chassis.pid_drive_set(41_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(22.5, TURN_SPEED, true);
}
void qualired(){
  chassis.pid_drive_set(-24_in, 76, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, 67, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-4.5, 67, true);
  chassis.pid_wait();
  intake.move(-110);
  pros::delay(750);
  chassis.pid_drive_set(4.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-135_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  clamp_piston.set(true);
  rollers.move(-127);
  chassis.pid_drive_set(-5_in, 67, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-135_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(85_deg, TURN_SPEED, true);
  chassis.pid_drive_set(23.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_drive_set(-11.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(5_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(11.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(72_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
}

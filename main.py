
# ------------------------------------------
# 
# 	Project:      Main
#	Author:       Omkar Shil
#	Created:      12/6/2024
#	Description:  Basic framework/test for final codes
# 
# ------------------------------------------


#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
left_motor_a = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
left_motor_b = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_6_1, False)
right_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)
ChainIntake = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
Arm_Score = Motor(Ports.PORT4, GearSetting.RATIO_36_1, False)
rollers = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
controller_1 = Controller(PRIMARY)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration


# Begin project code
drivetrain.set_stopping(BRAKE)
drivetrain.set_drive_velocity(100, PERCENT)
drivetrain.set_turn_velocity(100, PERCENT)

def autonomous():
    pass

def driver_control():
    
    #low battery warning
    while brain.battery.capacity() > 5:
        if brain.battery.capacity() < 5:
            brain.screen.print("LOW BATTERY")
    
    pass
    #intake
    if controller_1.buttonX.pressing():
        ChainIntake.set_max_torque(100, PERCENT)
        rollers.set_max_torque(100, PERCENT)
        ChainIntake.set_drive_velocity(100, PERCENT)
        rollers.set_drive_velocity(100, PERCENT)
    pass


package com.team687.frc2018;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {

    public static final int kRightMasterTalonID = 2;
    public static final int kLeftMasterTalonID = 1;

    // practice bot
    public static final int kRightSlaveTalon1ID = 4;
    public static final int kLeftSlaveTalon1ID = 5;

    // // comp bot
    // public static final int kRightSlaveVictorID = 4;
    // public static final int kLeftSlaveVictorID = 5;

    public static final int kArmID = 7;
    public static final int kWristID = 2;
    public static final int kIntakeRollers1ID = 30;
    public static final int kIntakeRollers2ID = 70;

    public static final int kClimberWheelLeftID = 0;

    public static final int kClimberWheelRightID = 3;
    // pneumies
    public static final int kIntakeClawID1 = 0;
    public static final int kIntakeClawID2 = 2;
    
    public static final int kAntiFoulThingID = 10;

}

package com.team687.frc2018.constants;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.modifiers.TankModifier;
public class AutoConstants {

    // pathfinder constants
    public static final double dt = 0.02;
    // max speed and accel
    public static final double kAcceleration = 13;
    public static final double kCruiseVelocity = 13;
    // Jerk is set to a high number since jerk barely matters, poofs don't jerk anymore
    public static final double kJerk = 100;

    private static Config test_config = new Config(Trajectory.FitMethod.HERMITE_CUBIC, Config.SAMPLES_HIGH, dt, kCruiseVelocity/3, kAcceleration/5, kJerk);
    private static Waypoint[] test_points = new Waypoint[] {
        new Waypoint(0, 0, 0),
        new Waypoint(5, 5, 0)  
    };

    private static Config testConfig = new Config(Trajectory.FitMethod.HERMITE_CUBIC, Config.SAMPLES_HIGH, dt, kCruiseVelocity, kAcceleration, kJerk);
    private static Waypoint[] testPoints = new Waypoint[] {
        new Waypoint(0, 12, 0),
        new Waypoint(7, 8, 0) 
        // new Waypoint(5, 7, Pathfinder.d2r(90)) 

    };
    private static Waypoint[] rightToRightSwitchPointsPart1 = new Waypoint[] {
        new Waypoint(0, 3.5, 0), new Waypoint(11, 3.5, 0), new Waypoint(14, 7, Pathfinder.d2r(90))
    };
     private static Waypoint[] rightToRightSwitchPointsPart2 = new Waypoint[] {
        new Waypoint(14, 7, Pathfinder.d2r(90)), new Waypoint(16, 5, 0), new Waypoint(21, 7.5, 0)
    };
    public static Trajectory testTraj = Pathfinder.generate(testPoints, testConfig);
    public static Trajectory rightToRightSwitchTrajPart1 = Pathfinder.generate(rightToRightSwitchPointsPart1, testConfig);
}
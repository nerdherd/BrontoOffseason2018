package com.team687.frc2018;

import com.nerdherd.lib.AbstractOI;
import com.team687.frc2018.commands.drive.auto.DriveDistanceMotionMagic;
import com.team687.frc2018.commands.drive.auto.ResetDriveEncoders;
import com.team687.frc2018.commands.drive.auto.ResetGyro;
import com.team687.frc2018.commands.drive.characterization.DriveCharacterizationTest;
import com.team687.frc2018.commands.drive.characterization.OpenLoopDrive;
import com.team687.frc2018.commands.intake.ClawClose;
import com.team687.frc2018.commands.intake.ClawOpen;
import com.team687.frc2018.commands.intake.SetIntakeRollerPower;
import com.team687.frc2018.constants.DriveConstants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI extends AbstractOI {

    public Joystick driveJoyLeft = new Joystick(0);
    public Joystick driveJoyRight = new Joystick(1);
    public Joystick operatorJoy = new Joystick(2);

    // public Joystick gamepadJoy = new Joystick(0);

    public JoystickButton intake_1;
    public JoystickButton outtake_2;
    public JoystickButton stopIntake_3;
    public JoystickButton deployClimberWheels_4;
    // public JoystickButton intakePosition_4;

    public JoystickButton switchPosition_11;
    public JoystickButton forwardsToStow_9;
    public JoystickButton stowToForwards_7;
    public JoystickButton adjustMiddle_8;
    public JoystickButton defaultStow_10;

    public JoystickButton openClaw_6;
    public JoystickButton retractClimberWheels_5;

    public JoystickButton flipCube_12;

    public OI() {
        // SmartDashboard.putData("Intake Rollers, 20%", new SetIntakeRollerPower(0.2, 0.2));
        // SmartDashboard.putData("Intake Rollers, 40%", new SetIntakeRollerPower(0.4, 0.4));
        // SmartDashboard.putData("Intake Rollers, 60%", new SetIntakeRollerPower(0.6, 0.6));
        // SmartDashboard.putData("Intake Rollers, 80%", new SetIntakeRollerPower(0.8, 0.8));
        // SmartDashboard.putData("Intake Rollers, 100%", new SetIntakeRollerPower(1.0, 1.0));

        // SmartDashboard.putData("Intake Rollers, -20%", new SetIntakeRollerPower(-0.2, -0.2));
        // SmartDashboard.putData("Intake Rollers, -40%", new SetIntakeRollerPower(-0.4, -0.4));
        // SmartDashboard.putData("Intake Rollers, -60%", new SetIntakeRollerPower(-0.6, -0.6));
        // SmartDashboard.putData("Intake Rollers, -80%", new SetIntakeRollerPower(-0.8, -0.8));
        // SmartDashboard.putData("Intake Rollers, -100%", new SetIntakeRollerPower(-1.0, -1.0));

        // SmartDashboard.putData("Deploy Climber Wheels", new DeployClimberWheels());
        // SmartDashboard.putData("Deploy Climber Wheel Power 0.3", new SetClimberWheelPower(0.3));
        // SmartDashboard.putData("Deploy Climber Wheel Power 0.6", new SetClimberWheelPower(0.6));
        // SmartDashboard.putData("Retract Climber Wheel Power -0.1", new SetClimberWheelPower(-0.1));
        // SmartDashboard.putData("Reset Climber Wheel Encoders", new ResetClimberEncoders());
        // SmartDashboard.putData("Piston push", new ClawClose());
        // SmartDashboard.putData("Piston pull", new ClawOpen());
        
        

        // SmartDashboard.putData("");

        intake_1 = new JoystickButton(operatorJoy, 1);
        intake_1.whenPressed(new SetIntakeRollerPower(0.5, 0.5));
        outtake_2 = new JoystickButton(operatorJoy, 2);
        outtake_2.whenPressed(new SetIntakeRollerPower(-0.4, -0.4));
        stopIntake_3 = new JoystickButton(operatorJoy, 3);
        stopIntake_3.whenPressed(new SetIntakeRollerPower(0.0, 0.0));
        // deployClimberWheels_4 = new JoystickButton(operatorJoy, 4);
        // deployClimberWheels_4.whenPressed(new DeployClimberWheels(0.75));
        // retractClimberWheels_5 = new JoystickButton(operatorJoy, 5);
        // retractClimberWheels_5.whenPressed(new DeployClimberWheels(-0.75));

        SmartDashboard.putData("Drive Characterize", new DriveCharacterizationTest(0.25));
        SmartDashboard.putData("Drive Open Loop 6 V", new OpenLoopDrive(0.5));
        SmartDashboard.putData("Drive Motion Magic", new DriveDistanceMotionMagic(15000, DriveConstants.kLeftAcceleration*0.3 , DriveConstants.kLeftCruiseVelocity * 0.3));
        SmartDashboard.putData("Reset Drive Encoders", new ResetDriveEncoders());
        SmartDashboard.putData("Reset Gyro", new ResetGyro());
    }


    /**
     * @return input power from left drive joystick Y (-1.0 to +1.0)
     */
    public double getDriveJoyLeftY() {
        // return -gamepadJoy.getRawAxis(1);
        return -driveJoyLeft.getY();
    }

    /**
     * @return input power from right drive joystick Y (-1.0 to +1.0)
     */
    public double getDriveJoyRightY() {
        // return -gamepadJoy.getRawAxis(3);
        return -driveJoyRight.getY();
    }

    /**
     * @return input power from left drive joystick X (-1.0 to +1.0)
     */
    public double getDriveJoyLeftX() {
        // return gamepadJoy.getRawAxis(0);
        return driveJoyLeft.getX();
    }

    /**
     * @return input power from right drive joystick X (-1.0 to +1.0)
     */
    public double getDriveJoyRightX() {
        // return gamepadJoy.getRawAxis(2);
        return driveJoyRight.getX();
    }

    @Override
    public double getOperatorJoyX() {
        return operatorJoy.getX();
    }

    @Override
    public double getOperatorJoyY() {
        return operatorJoy.getY();
    }

}

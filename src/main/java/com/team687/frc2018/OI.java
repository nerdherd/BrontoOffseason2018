package com.team687.frc2018;

import com.team687.frc2018.commands.arm.ResetArmEncoder;
import com.team687.frc2018.commands.drive.auto.DrivePurePursuit;
import com.team687.frc2018.commands.drive.auto.ResetDriveEncoders;
import com.team687.frc2018.commands.drive.auto.ResetGyro;
import com.team687.frc2018.commands.drive.characterization.DriveCharacterizationTest;
import com.team687.frc2018.commands.drive.characterization.OpenLoopDrive;
import com.team687.frc2018.commands.drive.characterization.VelocityTest;
import com.team687.frc2018.commands.intake.ClawClose;
import com.team687.frc2018.commands.intake.ClawOpen;
import com.team687.frc2018.commands.intake.SetIntakeRollerPower;
import com.team687.frc2018.commands.superstructure.AdjustForwardsScale;
import com.team687.frc2018.commands.superstructure.DefaultIntake;
import com.team687.frc2018.commands.superstructure.DefaultStow;
import com.team687.frc2018.commands.superstructure.FlipCube;
import com.team687.frc2018.commands.superstructure.ForwardsScaleToStow;
import com.team687.frc2018.commands.superstructure.IntakeSequenceCurrent;
import com.team687.frc2018.commands.superstructure.StowToForwardsScale;
import com.team687.frc2018.commands.superstructure.SwitchScorePositionTeleop;
import com.team687.frc2018.commands.wrist.ResetWristEncoder;
import com.team687.frc2018.constants.AutoConstants;
import com.team687.frc2018.constants.SuperstructureConstants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {

    public Joystick driveJoyLeft = new Joystick(0);
 	public Joystick driveJoyRight = new Joystick(1);
    public Joystick driveJoyArtic = new Joystick(2);

    // public Joystick gamepadJoy = new Joystick(0);

    public JoystickButton intake_1;
    public JoystickButton outtake_2;
    public JoystickButton stopIntake_3;
    public JoystickButton openCloseClaw_4;
    // public JoystickButton intakePosition_4;

    public JoystickButton switchPosition_11;
    public JoystickButton forwardsToStow_9;
    public JoystickButton stowToForwards_7;
    public JoystickButton adjustMiddle_8;
    public JoystickButton defaultStow_10;

    public JoystickButton openClaw_6;
    public JoystickButton closeClaw_5;

    public JoystickButton flipCube_12;

    public OI() {
	intake_1 = new JoystickButton(driveJoyArtic, 1);
	intake_1.whenPressed(new DefaultIntake());
	outtake_2 = new JoystickButton(driveJoyArtic, 2);
	outtake_2.whenPressed(new SetIntakeRollerPower(-0.8));
	stopIntake_3 = new JoystickButton(driveJoyArtic, 3);
    stopIntake_3.whenPressed(new SetIntakeRollerPower(0));
    openCloseClaw_4 = new JoystickButton(driveJoyArtic, 4);
    openCloseClaw_4.whenPressed(new IntakeSequenceCurrent());
	// intakePosition_4 = new JoystickButton(driveJoyArtic, 4);
    // intakePosition_4.whenPressed(new IntakePosition());
    


	forwardsToStow_9 = new JoystickButton(driveJoyArtic, 9);
	forwardsToStow_9.whenPressed(new ForwardsScaleToStow());
	stowToForwards_7 = new JoystickButton(driveJoyArtic, 7);
	stowToForwards_7.whenPressed(new StowToForwardsScale());

	adjustMiddle_8 = new JoystickButton(driveJoyArtic, 8);
	adjustMiddle_8.whenPressed(new AdjustForwardsScale(SuperstructureConstants.kArmMiddleScalePosition));
	defaultStow_10 = new JoystickButton(driveJoyArtic, 10);
	defaultStow_10.whenPressed(new DefaultStow());

	switchPosition_11 = new JoystickButton(driveJoyArtic, 11);
	switchPosition_11.whenPressed(new SwitchScorePositionTeleop());


	openClaw_6 = new JoystickButton(driveJoyArtic, 5);
	openClaw_6.whenPressed(new ClawOpen());
	closeClaw_5 = new JoystickButton(driveJoyArtic, 6);
    closeClaw_5.whenPressed(new ClawClose());
    flipCube_12 = new JoystickButton(driveJoyArtic, 12);
	flipCube_12.whenPressed(new FlipCube());
	
	SmartDashboard.putData("Arm Reset Encoder", new ResetArmEncoder());
	SmartDashboard.putData("Wrist Reset Encoder", new ResetWristEncoder());
	SmartDashboard.putData("Drive Reset Encoders", new ResetDriveEncoders());
    SmartDashboard.putData("Drive Reset Gyro", new ResetGyro());    
	SmartDashboard.putData("Superstructure Stow", new DefaultStow());

	SmartDashboard.putData("DT Char Test", new DriveCharacterizationTest(0.25));
	SmartDashboard.putData("12 V drivetrain open loop", new OpenLoopDrive(1));
	SmartDashboard.putData("9 V drivetrain open loop", new OpenLoopDrive(0.75));
	SmartDashboard.putData("6 V drivetrain open loop", new OpenLoopDrive(0.5));
	SmartDashboard.putData("3 V drivetrain open loop", new OpenLoopDrive(0.25));
	
	SmartDashboard.putData("Pure Pursuit Test", new DrivePurePursuit(AutoConstants.testTraj, 4, true));
    SmartDashboard.putData("Velocity Test", new VelocityTest(1000, 5));
    // test velocity at around 1/4 - 1/3 of max velocity
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

    /**
     * @return input throttle from right drive joystick (0 to +1.0)
     */
    public double getThrottleR() {
	return (driveJoyRight.getThrottle() + 1) / 2;
    }

    /**
     * @return input throttle from left drive josytick
     */
    public double getThrottleL() {
	return (driveJoyLeft.getThrottle() + 1) / 2;
    }

}

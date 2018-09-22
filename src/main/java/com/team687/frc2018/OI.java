package com.team687.frc2018;

import com.team687.frc2018.commands.arm.ResetArmEncoder;
import com.team687.frc2018.commands.arm.SetArmVoltage;
import com.team687.frc2018.commands.arm.SetArmPosition;
import com.team687.frc2018.commands.auto.CenterToLeftSwitchAuto;
import com.team687.frc2018.commands.auto.CenterToRightSwitchAuto;
import com.team687.frc2018.commands.auto.DriveStraightAuto;
import com.team687.frc2018.commands.auto.LeftToLeftScaleAuto;
import com.team687.frc2018.commands.auto.LeftToLeftSwitchAuto;
import com.team687.frc2018.commands.auto.LeftToRightScaleAuto;
import com.team687.frc2018.commands.auto.RightToLeftScaleAuto;
import com.team687.frc2018.commands.auto.RightToRightCompatibleScaleAuto;
import com.team687.frc2018.commands.auto.RightToRightScaleAuto;
import com.team687.frc2018.commands.auto.RightToRightSwitchAuto;
import com.team687.frc2018.commands.auto.TestBackwardsBezier;
import com.team687.frc2018.commands.drive.ResetDriveEncoders;
import com.team687.frc2018.commands.drive.ResetGyro;
import com.team687.frc2018.commands.drive.TurnToAngle;
import com.team687.frc2018.commands.intake.ClawClose;
import com.team687.frc2018.commands.intake.ClawOpen;
import com.team687.frc2018.commands.intake.SetIntakeRollerPower;
import com.team687.frc2018.commands.superstructure.AdjustForwardsScale;
import com.team687.frc2018.commands.superstructure.FlipCube;
import com.team687.frc2018.commands.superstructure.BackwardsScaleToStow;
import com.team687.frc2018.commands.superstructure.DefaultIntake;
import com.team687.frc2018.commands.superstructure.DefaultStow;
import com.team687.frc2018.commands.superstructure.ForwardsScaleToStow;
import com.team687.frc2018.commands.superstructure.StowToBackwardsScale;
import com.team687.frc2018.commands.superstructure.StowToForwardsScale;
import com.team687.frc2018.commands.superstructure.SwitchScorePositionAuto;
import com.team687.frc2018.commands.superstructure.SwitchScorePositionTeleop;
import com.team687.frc2018.commands.wrist.ResetWristEncoder;
import com.team687.frc2018.commands.wrist.SetWristPosition;
import com.team687.frc2018.commands.wrist.SetWristVoltage;
import com.team687.frc2018.constants.SuperstructureConstants;
import com.team687.frc2018.commands.superstructure.IntakeSequenceCurrent;

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
    public JoystickButton intakePosition_4;

    public JoystickButton switchPosition_11;
    public JoystickButton forwardsToStow_9;
    public JoystickButton stowToForwards_7;
    public JoystickButton adjustMiddle_8;
    public JoystickButton defaultStow_10;

    public JoystickButton openClaw_6;
    public JoystickButton closeClaw_5;

    public JoystickButton openCloseClaw_4;

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

    SmartDashboard.putData("Backwards Scale", new StowToBackwardsScale()
    );

	SmartDashboard.putData("Arm Reset Encoder", new ResetArmEncoder());
	SmartDashboard.putData("Wrist Reset Encoder", new ResetWristEncoder());
	SmartDashboard.putData("Drive Reset Encoders", new ResetDriveEncoders());
    SmartDashboard.putData("Drive Reset Gyro", new ResetGyro());
    
    SmartDashboard.putData("3 V arm open loop", new SetArmVoltage(3));
    SmartDashboard.putData("6 V arm open loop", new SetArmVoltage(6));
    SmartDashboard.putData("9 V arm open loop", new SetArmVoltage(9));
    SmartDashboard.putData("12 V arm open loop", new SetArmVoltage(12));

    SmartDashboard.putData("-3 V arm open loop", new SetArmVoltage(-3));
    SmartDashboard.putData("-6 V arm open loop", new SetArmVoltage(-6));
    SmartDashboard.putData("-9 V arm open loop", new SetArmVoltage(-9));
    SmartDashboard.putData("-12 V arm open loop", new SetArmVoltage(-12));

    SmartDashboard.putData("0 V arm open loop", new SetArmVoltage(0));

    // SmartDashboard.putData("3 V wrist open loop", new SetWristVoltage(3));
    // SmartDashboard.putData("6 V wrist open loop", new SetWristVoltage(6));
    // SmartDashboard.putData("9 V wrist open loop", new SetWristVoltage(9));
    // SmartDashboard.putData("12 V wrist open loop", new SetWristVoltage(12));

    // SmartDashboard.putData("-3 V wrist open loop", new SetWristVoltage(-3));
    // SmartDashboard.putData("-6 V wrist open loop", new SetWristVoltage(-6));
    // SmartDashboard.putData("-9 V wrist open loop", new SetWristVoltage(-9));
    // SmartDashboard.putData("-12 V wrist open loop", new SetWristVoltage(-12));

    SmartDashboard.putData("0 V wrist open loop", new SetWristVoltage(0));
    
    // SmartDashboard.putData("Wrist Hopefully back", new SetWristPosition(-100));
    // SmartDashboard.putData("Wrist Vertical Up", new SetWristPosition(-2070));
    // SmartDashboard.putData("Wrist Horizontal", new SetWristPosition(-4600));
    // SmartDashboard.putData("Wrsit Vertical Down", new SetWristPosition(-7200));

	// SmartDashboard.putData("Drive Straight Test", new TestDriveSubsystem());
	//
	// SmartDashboard.putData("Arm Voltage 0", new SetArmVoltage(0));
	//  SmartDashboard.putData("Arm Position Vertical", new
	//  SetArmPosition(SuperstructureConstants.kArmVerticalPos));
	//  SmartDashboard.putData("Arm Position Horizontal",
	//  new SetArmPosition(SuperstructureConstants.kArmHorizontalPos));
    SmartDashboard.putData("Arm Position Offset", new
    SetArmPosition(SuperstructureConstants.kArmOffsetPos));
     
	// //
	// // SmartDashboard.putData("Wrist Voltage 0", new SetWristPercentOutput(0));
	//  SmartDashboard.putData("Wrist Position Intake", new
	//  SetWristPosition(Robot.wrist.angleAbsoluteToTicks(0)));
	//  SmartDashboard.putData("Wrist Position Offset Stow",
	//  new SetWristPosition(SuperstructureConstants.kWristStowArmOffsetPos));
	//  SmartDashboard.putData("Wrist Position Stow", new
	//  SetWristPosition(Robot.wrist.angleAbsoluteToTicks(90)));
	//
	// SmartDashboard.putData("Set Intake Power 1", new SetIntakeRollerPower(1));
	// SmartDashboard.putData("Set Intake Power -1", new SetIntakeRollerPower(-1));
	// SmartDashboard.putData("Set Intake Power 0", new SetIntakeRollerPower(0));
	// SmartDashboard.putData("Outtake", new SetIntakeRollerPower(0.4));
	SmartDashboard.putData("Open Intake Claw", new ClawOpen());
	SmartDashboard.putData("Close Intake Claw", new ClawClose());

	//  SmartDashboard.putData("Superstructure Stow to Backwards Scale", new
	//  StowToBackwardsScale());
	//  SmartDashboard.putData("Superstructure Stow to Forwards Scale", new
	//  StowToForwardsScale());
	//  SmartDashboard.putData("Superstructure Backwards Scale To Stow", new
	//  BackwardsScaleToStow());
	//  SmartDashboard.putData("Superstructure Forwards Scale to Stow", new
	//  ForwardsScaleToStow());
	 
	//  SmartDashboard.putData("Superstructure Scale Adjust Mid", new AdjustForwardsScale(SuperstructureConstants.kArmMiddleScalePosition));
	//  SmartDashboard.putData("Superstructure Scale Adjust Low", new AdjustForwardsScale(SuperstructureConstants.kArmLowScalePosition));
	//  SmartDashboard.putData("Superstructure Scale Adjust Lower", new AdjustForwardsScale(SuperstructureConstants.kArmLowerScalePosition));
	
	//  SmartDashboard.putData("Superstructure Stow", new DefaultStow());
	//  SmartDashboard.putData("Superstructure Intake", new DefaultIntake());
	//  SmartDashboard.putData("Superstructure Switch Teleop Pos", new SwitchScorePositionTeleop());
	// SmartDashboard.putData("Superstructure Intake Position", new
	// IntakePosition());

	// SmartDashboard.putData("Turn To 0", new TurnToAngle(0, 2, 2));
	// SmartDashboard.putData("Turn To 90", new TurnToAngle(90, 2, 2));
	// SmartDashboard.putData("Turn To -90", new TurnToAngle(-90, 2, 2));
	// SmartDashboard.putData("Turn To 5", new TurnToAngle(5, 2, 2));
	// SmartDashboard.putData("Turn To -5", new TurnToAngle(-5, 2, 2));
	// SmartDashboard.putData("Turn To 20", new TurnToAngle(20, 2, 2));
	// SmartDashboard.putData("Turn To -20", new TurnToAngle(-20, 2, 2));

	// SmartDashboard.putData("Center To Left Switch", new CenterToLeftSwitchAuto());
	// SmartDashboard.putData("Center To Right Switch", new CenterToRightSwitchAuto());
	// SmartDashboard.putData("Test Backwards Bezier", new TestBackwardsBezier());
	
	// SmartDashboard.putData("Left To Left Scale", new LeftToLeftScaleAuto());
	// SmartDashboard.putData("Left To Right Scale", new LeftToRightScaleAuto());
	// SmartDashboard.putData("Left To Left Switch", new LeftToLeftSwitchAuto());
	// SmartDashboard.putData("Right To Left Scale", new RightToLeftScaleAuto());
	// SmartDashboard.putData("Right To Right Scale", new RightToRightScaleAuto());
	// SmartDashboard.putData("Right To Right Compatible Scale", new RightToRightCompatibleScaleAuto());
	// SmartDashboard.putData("Right To Right Switch", new RightToRightSwitchAuto());
	// SmartDashboard.putData("Open Claw", new ClawOpen());
	// SmartDashboard.putData("Close Claw", new ClawClose());

	// SmartDashboard.putData("Superstructure Stow to Forwards Scale", new StowToForwardsScale());
	// SmartDashboard.putData("Superstructure Forwards Scale to Stow", new ForwardsScaleToStow());	


	// SmartDashboard.putData("Drive Straight Auto", new DriveStraightAuto());

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
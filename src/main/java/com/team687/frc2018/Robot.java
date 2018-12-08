package com.team687.frc2018;

import com.team687.frc2018.subsystems.Arm;
import com.team687.frc2018.subsystems.Drive;
import com.team687.frc2018.subsystems.Intake;
import com.team687.frc2018.subsystems.Wrist;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {

    public static final String kDate = "2018_11_30_";

    public static Drive drive;
    public static Arm arm;
    public static Wrist wrist;
    public static Intake intake;

    public static DriverStation ds;
    public static PowerDistributionPanel pdp;
    public static Compressor compressor;

    public static OI oi;

    SendableChooser<String> sideChooser;
    public static Command autonomousCommand;
    public static String startingPosition;
    public static boolean switchOnLeft, scaleOnLeft;

    @Override
    public void robotInit() {
	pdp = new PowerDistributionPanel();
	LiveWindow.disableTelemetry(pdp);
	compressor = new Compressor();
	compressor.start();

	arm = new Arm();
	arm.setVoltage(0);
	arm.resetEncoder();

	wrist = new Wrist();
	wrist.setPercentOutput(0);
	wrist.resetEncoder();

	intake = new Intake();
	intake.setRollerPower(0);

	drive = new Drive();
	drive.resetEncoders();
	drive.resetYaw();

	oi = new OI();
	ds = DriverStation.getInstance();
	// CameraServer.getInstance().startAutomaticCapture();
    }

    @Override
    public void disabledInit() {
	Scheduler.getInstance().removeAll();

	drive.reportToSmartDashboard();
	arm.reportToSmartDashboard();
	wrist.reportToSmartDashboard();
	intake.reportToSmartDashboard();
	drive.stopLog();
	arm.stopLog();
	wrist.stopLog();

	// drive.stopOdometry();
	// drive.resetXY();
	// drive.startOdometry();
    }

    @Override
    public void disabledPeriodic() {
    Scheduler.getInstance().removeAll();
    Scheduler.getInstance().run();
	// drive.resetEncoders();
	// drive.resetYaw();
	drive.calcXY();
	drive.reportToSmartDashboard();
	arm.reportToSmartDashboard();
	wrist.reportToSmartDashboard();
	intake.reportToSmartDashboard();
    }

    @Override
    public void autonomousInit() {
	// Scheduler.getInstance().removeAll();


	// drive.startLog();
	// arm.startLog();
	// wrist.startLog();
    }

    @Override
    public void autonomousPeriodic() {
	Scheduler.getInstance().run();

	drive.reportToSmartDashboard();
	arm.reportToSmartDashboard();
	wrist.reportToSmartDashboard();
	intake.reportToSmartDashboard();
	// drive.logToCSV();
	// arm.logToCSV();
	// wrist.logToCSV();
    }

    @Override
    public void teleopInit() {
	drive.reportToSmartDashboard();
	arm.reportToSmartDashboard();
	wrist.reportToSmartDashboard();
	intake.reportToSmartDashboard();
	
	drive.calcXY();
	drive.startLog();
	arm.startLog();
    wrist.startLog();
    Scheduler.getInstance().run();
    }

    @Override
    public void teleopPeriodic() {
	drive.calcXY();
	Scheduler.getInstance().run();

	drive.reportToSmartDashboard();
	arm.reportToSmartDashboard();
	wrist.reportToSmartDashboard();
	intake.reportToSmartDashboard();
	drive.logToCSV();
	arm.logToCSV();
	wrist.logToCSV();
	}

    @Override
    public void testPeriodic() {
    }
}

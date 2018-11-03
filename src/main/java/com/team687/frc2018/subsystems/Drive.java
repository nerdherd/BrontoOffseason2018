package com.team687.frc2018.subsystems;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.team687.frc2018.RobotMap;
import com.team687.frc2018.commands.drive.characterization.VelocityPIDF;
import com.team687.frc2018.commands.drive.teleop.ArcadeDrive;
import com.team687.frc2018.constants.DriveConstants;
import com.team687.frc2018.utilities.NerdyTalon;
import com.team687.frc2018.Robot;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

/**
 *
 */
public class Drive extends Subsystem {

	private final NerdyTalon m_leftMaster, m_leftSlave1;
	private final NerdyTalon m_rightMaster, m_rightSlave1;
	private final AHRS m_nav;
	
	private double m_previousDistance, m_currentX, m_currentY, m_angleOffset, m_xOffset, m_yOffset;
    
    private String m_filePath1 = "/media/sda1/logs/";
	private String m_filePath2 = "/home/lvuser/logs/";
	private String m_fileName = Robot.kDate + "pathfinder_test_drive";
    private File m_file;
    public FileWriter m_writer;
    private boolean writeException = false;
	private double m_logStartTime = 0;
	private double m_leftDesiredVel, m_rightDesiredVel;
	private VelocityPIDF m_velocityPIDF;
	private Notifier m_velocityNotifier;
    
	public Drive() {
		
		m_nav = new AHRS(SPI.Port.kMXP);
		
		m_leftMaster = new NerdyTalon(RobotMap.kLeftMasterTalonID);
		m_leftSlave1 = new NerdyTalon(RobotMap.kLeftSlaveTalon1ID);
		
		m_rightMaster = new NerdyTalon(RobotMap.kRightMasterTalonID);
		m_rightSlave1 = new NerdyTalon(RobotMap.kRightSlaveTalon1ID);
		
		
		m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		m_rightSlave1.follow(m_rightMaster);
		
		m_leftSlave1.follow(m_leftMaster);
				
		m_leftMaster.setInverted(false);
		m_leftSlave1.setInverted(false);
		
		m_rightMaster.setInverted(true);
		m_rightSlave1.setInverted(true);

		m_leftMaster.setSensorPhase(true);
		m_rightMaster.setSensorPhase(true);
		
		// m_rightMaster.configPIDF(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD, DriveConstants.kRightF, 0);
		// m_leftMaster.configPIDF(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD, DriveConstants.kLeftF, 0);

		// m_leftMaster.configMotionMagic(DriveConstants.kLeftAcceleration, DriveConstants.kLeftCruiseVelocity);
		// m_rightMaster.configMotionMagic(DriveConstants.kRightAcceleration, DriveConstants.kRightCruiseVelocity);

		m_leftMaster.setNeutralMode(NeutralMode.Brake);
		m_leftSlave1.setNeutralMode(NeutralMode.Brake);
		
		m_rightMaster.setNeutralMode(NeutralMode.Brake);
		m_rightSlave1.setNeutralMode(NeutralMode.Brake);
		
		m_rightMaster.configDefaultSettings();
		m_rightSlave1.configDefaultSettings();

		m_leftMaster.configDefaultSettings();
		m_leftSlave1.configDefaultSettings();

		m_rightMaster.configPIDF(DriveConstants.kRightVelocityP, 0, DriveConstants.kRightVelocityD, DriveConstants.kRightV, 0);
		m_leftMaster.configPIDF(DriveConstants.kLeftVelocityP, 0, DriveConstants.kLeftVelocityD, DriveConstants.kLeftV, 0);
	}
	
	public void setPower(double leftPower, double rightPower) {

		m_leftMaster.set(ControlMode.PercentOutput, leftPower);
		m_rightMaster.set(ControlMode.PercentOutput, rightPower);
    }
	
	public void setVoltage(double leftVoltage, double rightVoltage) {
		m_leftMaster.set(ControlMode.PercentOutput, leftVoltage/12);
		m_rightMaster.set(ControlMode.PercentOutput, rightVoltage/12);
	}

	public void setPowerZero() {
		m_leftMaster.set(ControlMode.PercentOutput, 0);
		m_rightMaster.set(ControlMode.PercentOutput, 0);
	}
	
	public void addDesiredVelocities(double leftVel, double rightVel) {
		m_leftDesiredVel = leftVel;
		m_rightDesiredVel = rightVel;
	}

	public void setPositionMotionMagic(double leftPosition, double rightPosition) {
		m_leftMaster.set(ControlMode.MotionMagic, leftPosition);
		m_rightMaster.set(ControlMode.MotionMagic, rightPosition);
	}
	
	public void setVelocity(double leftVel, double rightVel) {
		m_rightMaster.set(ControlMode.Velocity, rightVel);
		m_leftMaster.set(ControlMode.Velocity, leftVel);
		
	}
	
	public void resetEncoders() {
		m_leftMaster.setSelectedSensorPosition(0, 0, 0);
		m_rightMaster.setSelectedSensorPosition(0, 0, 0);
	}
	public double getLeftOutputVoltage() {
		return m_leftMaster.getMotorOutputVoltage();
	}
	
	public double getLeftMasterCurrent() {
		return m_leftMaster.getOutputCurrent();
	}
	
	public double getLeftMasterPosition() {
		return m_leftMaster.getSelectedSensorPosition(0);
	}
	
	public double getLeftMasterSpeed() {
		return m_leftMaster.getSelectedSensorVelocity(0);
	}
	
	
	public double getRightOutputVoltage() {
		return m_rightMaster.getMotorOutputVoltage();
	}
	
	public double getRightMasterCurrent() {
		return m_rightMaster.getOutputCurrent();
	}
	
	public double getRightMasterPosition() {
		return m_rightMaster.getSelectedSensorPosition(0);
	}
	
	public double getRightMasterSpeed() {
		return m_rightMaster.getSelectedSensorVelocity(0);
	}
	
	
	public double getRawYaw() {
		return Pathfinder.boundHalfDegrees(m_nav.getYaw());
	}
	
	public void resetYaw() {
		m_nav.reset();
	}
	
	public double getAverageEncoderPosition() {
		return (getRightMasterPosition() + getLeftMasterPosition())/2;
	}
	
	public double getAngle() {
//		converts angle from -180 to 180 to 0 to 360	
//		sets positive y as 0 deg, robot's front is 0 deg
		return (360 - getRawYaw()) % 360;
		
	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new ArcadeDrive());
    }   
    
	public void resetXY() {
		m_currentX = 0;
		m_currentY = 0;
	}
	
    public void calcXY() {
    	// calculate x,y coordinates when moving in straight lines and turning in place, DOES NOT WORK
    	double m_currentDistance = (getRightPositionFeet() +getLeftPositionFeet())/2;
    	double m_distanceTraveled = (m_currentDistance - m_previousDistance);
    	double angle = getRawYaw();
    	m_currentX = m_currentX + m_distanceTraveled * Math.sin(Math.toRadians(angle));
    	m_currentY = m_currentY + m_distanceTraveled * Math.cos(Math.toRadians(angle));
    	m_previousDistance = m_currentDistance;
    }
    
    public double getXpos() {
    	return m_currentX;
    }
    
    public double getYpos() {
    	return m_currentY;
    }
	
	public double ticksToFeet(double ticks) {
		return ticks / DriveConstants.kTicksPerFoot;
	}
	
	public double feetToTicks(double feet) {
		return feet * DriveConstants.kTicksPerFoot;
	}

	public double getLeftVelocityFeet() {
		return ticksToFeet(m_leftMaster.getSelectedSensorVelocity(0) / 0.1);
	}

	public double getRightVelocityFeet() {
		return ticksToFeet(m_rightMaster.getSelectedSensorVelocity(0) / 0.1);
	}

	public double getLeftPositionFeet() {
		return ticksToFeet(m_leftMaster.getSelectedSensorPosition(0));
	}

	public double getRightPositionFeet() {
		return m_rightMaster.getSelectedSensorPosition(0) / DriveConstants.kTicksPerFoot;
	}

	public double fpsToTalonVelocityUnits(double fps) {
		return feetToTicks(fps)/10;
	}

	public void setVelocityFPS(double leftVel, double rightVel) {
		setVelocity(fpsToTalonVelocityUnits(leftVel), fpsToTalonVelocityUnits(rightVel));
	}


    public void reportToSmartDashboard() {
    	SmartDashboard.putNumber("Left Master Voltage", getLeftOutputVoltage());
    	SmartDashboard.putNumber("Right Master Voltage", getRightOutputVoltage());
    	
    	SmartDashboard.putNumber("Left Master Position", getLeftMasterPosition());
    	SmartDashboard.putNumber("Right Master Position", getRightMasterPosition());
		
		SmartDashboard.putNumber("Left Master Position Feet", getLeftPositionFeet());
		SmartDashboard.putNumber("Right Master Position Feet", getRightPositionFeet());		
		SmartDashboard.putNumber("Yaw", getRawYaw());
    	SmartDashboard.putNumber("X pos", m_currentX);
    	SmartDashboard.putNumber("Y pos", m_currentY);
    	
    }
    
    public void startLog() {
		writeException = false;
		// Check to see if flash drive is mounted.
		File logFolder1 = new File(m_filePath1);
		File logFolder2 = new File(m_filePath2);
		Path filePrefix = Paths.get("");
		if (logFolder1.exists() && logFolder1.isDirectory())
			filePrefix = Paths.get(logFolder1.toString(), m_fileName);
		else if (logFolder2.exists() && logFolder2.isDirectory())
			filePrefix = Paths.get(logFolder2.toString(),
					SmartDashboard.getString("log_file_name", m_fileName));
		else
			writeException = true;

		if (!writeException) {
			int counter = 0;
			while (counter <= 99) {
				m_file = new File(filePrefix.toString() + String.format("%02d", counter) + ".csv");
				if (m_file.exists()) {
					counter++;
				} else {
					break;
				}
				if (counter == 99) {
					System.out.println("file creation counter at 99!");
				}
			}
			try {
				m_writer = new FileWriter(m_file);
				m_writer.append("Time,RightPosition,LeftPosition,RightVelocity,LeftVelocity,RightDesiredVel,LeftDesiredVel,RightVoltage,LeftVoltage,"
						+ "RightMasterCurrent,LeftMasterCurrent,RightSlaveCurrent,LeftSlaveCurrent,BusVoltage,Yaw\n");
				m_writer.flush();
				m_logStartTime = Timer.getFPGATimestamp();
			} catch (IOException e) {
				e.printStackTrace();
				writeException = true;
			}
		}
	}

	public void stopLog() {
		try {
			if (null != m_writer)
				m_writer.close();
		} catch (IOException e) {
			e.printStackTrace();
			writeException = true;
		}
	}

	public void logToCSV() {
		if (!writeException) {
			try {
				double timestamp = Timer.getFPGATimestamp() - m_logStartTime;
				m_writer.append(String.valueOf(timestamp) + "," + String.valueOf(getRightMasterPosition()) + ","
						+ String.valueOf(getLeftMasterPosition()) + "," + String.valueOf(m_rightMaster.getSelectedSensorVelocity(0)) + ","
						+ String.valueOf(m_leftMaster.getSelectedSensorVelocity(0)) + "," + String.valueOf(m_rightDesiredVel) + "," + String.valueOf(m_leftDesiredVel)
						+ "," + String.valueOf(m_rightMaster.getMotorOutputVoltage())
						+ "," + String.valueOf(m_leftMaster.getMotorOutputVoltage()) + ","
						+ String.valueOf(m_rightMaster.getOutputCurrent()) + ","
						+ String.valueOf(m_leftMaster.getOutputCurrent()) + ","
						+ String.valueOf(m_rightSlave1.getOutputCurrent()) + ","
						+ String.valueOf(m_leftSlave1.getOutputCurrent()) + "," 
						+ String.valueOf(getRawYaw()) + "\n");
				m_writer.flush();
			} catch (IOException e) {
				e.printStackTrace();
				writeException = true;
			}
		}
	}
}


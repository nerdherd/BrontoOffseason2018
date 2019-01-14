package com.team687.frc2018.commands.drive.auto;

import com.team687.frc2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveDistanceMotionMagic extends Command {

	private double m_distance;
    private double m_error;
    private int m_accel, m_cruiseVel;
    
	
    public DriveDistanceMotionMagic(double distance, double accel, double cruiseVel) {
        m_distance = distance;
        m_accel = (int) accel;
        m_cruiseVel = (int) cruiseVel;
       requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_error = m_distance - Robot.drive.getAverageEncoderPosition();
    	Robot.drive.setPositionMotionMagic(m_distance, m_distance, m_accel, m_cruiseVel);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        // return NerdyMath.errorTolerance(m_error, DriveConstants.kDriveTolerance);
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drive.setPowerZero();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}

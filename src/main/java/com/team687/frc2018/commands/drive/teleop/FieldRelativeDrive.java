package com.team687.frc2018.commands.drive.teleop;

import com.team687.frc2018.constants.DriveConstants;
import com.team687.frc2018.utilities.NerdyMath;
import com.team687.frc2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Field Relative Drive:
 * Based on standard 2 stick arcade, 1 stick control's robot's turning relative to the field/navx, other controls straight power
 */
public class FieldRelativeDrive extends Command {

	private double m_desiredAngle;
	private double m_error;
	private double m_currentAngle;
	private double m_rotPower;
	private double m_straightPower;
	
    public FieldRelativeDrive() {
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	m_desiredAngle = (Math.toDegrees(Math.atan2(Robot.oi.getDriveJoyRightY(), Robot.oi.getDriveJoyRightX())) - 90) * -1; //convert from radians to degrees, amd then from 0-360 to -180 to 180 so the PID loop can use it
    	m_currentAngle = Robot.drive.getRawYaw();
    	m_error = -m_desiredAngle - m_currentAngle;
    	
    	if (m_error >= 180) {
    		m_error -= 360;
    	}
    	if (m_error <= -180) {
    		m_error += 360;
    	}
    	
    	m_rotPower = m_error * DriveConstants.kRotP;
    	// if (!Robot.oi.isLeftTriggerPulled()) {//If the trigger isn't pulled, straight power is limited at 0.7 to prevent power from saturating. This leaves enough power for the PID to take effect
    	// 	m_straightPower = NerdyMath.threshold(Robot.oi.getLeftY(), 0, 0.7);
    	// }
    	// else {
    		m_straightPower = Robot.oi.getDriveJoyLeftY();
    	// }
    	
    	Robot.drive.setPower(m_straightPower - m_rotPower, m_straightPower + m_rotPower);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

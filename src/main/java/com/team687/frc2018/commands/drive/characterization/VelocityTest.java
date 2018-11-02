/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team687.frc2018.commands.drive.characterization;

import com.team687.frc2018.constants.DriveConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import com.team687.frc2018.Robot;

public class VelocityTest extends Command {

  private double m_desiredVel, m_time, m_prevTime, m_startTime;
  private double m_leftError, m_rightError, m_leftPrevError, m_rightPrevError;
  private double m_leftVoltage, m_rightVoltage, m_desiredTime;

  public VelocityTest(double desired_vel, double desired_time) {
    m_desiredVel = desired_vel;
    m_desiredTime = desired_time;
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_prevTime = Timer.getFPGATimestamp();
    m_leftPrevError = m_desiredVel - Robot.drive.getLeftMasterSpeed();
    m_rightPrevError = m_desiredVel - Robot.drive.getRightMasterSpeed();
    m_desiredVel = Robot.drive.feetToTicks(m_desiredVel) * 0.1;

    // Robot.drive.startVelocityController();
    // Robot.drive.setTargetVelocities(m_desiredVel, m_desiredVel);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_time = Timer.getFPGATimestamp() - m_startTime;
    m_leftError = m_desiredVel - Robot.drive.getLeftMasterSpeed();
    m_rightError = m_desiredVel - Robot.drive.getRightMasterSpeed();

    m_leftVoltage = DriveConstants.kLeftStatic * Math.signum(m_desiredVel) + DriveConstants.kLeftV * m_desiredVel 
      + m_leftError * DriveConstants.kLeftVelocityP + DriveConstants.kLeftVelocityD
      * (m_leftError - m_leftPrevError)/(m_time - m_prevTime);
    m_rightVoltage = DriveConstants.kRightStatic * Math.signum(m_desiredVel) + DriveConstants.kRightV * m_desiredVel 
    + m_rightError * DriveConstants.kRightVelocityP + DriveConstants.kRightVelocityD
    * (m_rightError - m_rightPrevError)/(m_time - m_prevTime);
    Robot.drive.setVoltage(m_leftVoltage, m_rightVoltage);
    Robot.drive.addDesiredVelocities(m_desiredVel, m_desiredVel);
    m_prevTime = m_time;
    m_leftPrevError = m_leftError;
    m_rightPrevError = m_rightError;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
        // m_time = Timer.getFPGATimestamp() - m_startTime;
      return m_time > m_desiredTime;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.setPowerZero();
    // Robot.drive.stopVelocityPIDF();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

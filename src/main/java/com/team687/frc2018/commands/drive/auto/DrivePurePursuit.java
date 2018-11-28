/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team687.frc2018.commands.drive.auto;

import com.team687.frc2018.constants.DriveConstants;
import com.team687.frc2018.utilities.PurePursuitController;
import com.team687.frc2018.utilities.AdaptivePurePursuitController;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import com.team687.frc2018.Robot;


public class DrivePurePursuit extends Command {

  // private PurePursuitController m_controller;
  private AdaptivePurePursuitController m_controller;
  private double m_leftVelocity, m_rightVelocity;

  public DrivePurePursuit(Trajectory traj, int lookahead, boolean goingForward) {
    // m_controller = new PurePursuitController(traj, lookahead, goingForward, DriveConstants.kDrivetrainWidth);
    m_controller = new AdaptivePurePursuitController(traj, lookahead, goingForward, DriveConstants.kDrivetrainWidth);
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_controller.calculate(Robot.drive.getXpos(), Robot.drive.getYpos(), Robot.drive.getRawYaw());
    m_leftVelocity = m_controller.getLeftVelocity();
    m_rightVelocity = m_controller.getRightVelocity();
    Robot.drive.setVelocityFPS(m_leftVelocity, m_rightVelocity);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_controller.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.setPowerZero();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

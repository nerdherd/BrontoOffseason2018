/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team687.frc2018.commands;

import com.team687.frc2018.Robot;
import com.team687.frc2018.constants.SuperstructureConstants;

import edu.wpi.first.wpilibj.command.Command;

public class DeployClimberWheels extends Command {

  private double m_power;

  // 30% open loop works to deploy
  public DeployClimberWheels() {
    // m_power = power;
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Robot.climber.setClimberWheelPower(m_power);
    if (Robot.climber.getClimberPosition() < 1024) {
      Robot.climber.setClimberWheelPower(0.3);
    } else {
      Robot.climber.setClimberPosition(SuperstructureConstants.kLeftClimberWheelDeployPos, SuperstructureConstants.kRightClimberWheelDeployPos);
    }
    // Robot.climber.setClimberPosition(SuperstructureConstants.kLeftClimberWheelDeployPos, SuperstructureConstants.kRightClimberWheelDeployPos);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

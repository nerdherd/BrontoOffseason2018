/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team687.frc2018.commands.superstructure;

import com.team687.frc2018.Robot;
import com.team687.frc2018.constants.SuperstructureConstants;

import edu.wpi.first.wpilibj.command.Command;

public class FlipCube extends Command {

    private boolean m_hasTurned = false;

  public FlipCube() {
	requires(Robot.arm);
	requires(Robot.wrist);
    requires(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_hasTurned = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if (Math.abs(Robot.drive.getLeftMasterVoltage() - Robot.drive.getRightMasterVoltage()) > 5) {
          m_hasTurned = true;
      }
    if (m_hasTurned) { 
        Robot.wrist.setPosition(SuperstructureConstants.kWristIntakePos);
        Robot.intake.setRollerPower(1);
    } 
    else {  
        Robot.wrist.setPosition(SuperstructureConstants.kWristFlipCubePos);
    }
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
    end();
  }
}

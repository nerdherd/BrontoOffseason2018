/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team687.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team687.frc2018.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ClimberWheels extends Subsystem {
  private TalonSRX m_climberLeft, m_climberRight;

  public ClimberWheels() {
    m_climberLeft = new TalonSRX(RobotMap.kClimberWheelLeftID);
    m_climberRight = new TalonSRX(RobotMap.kClimberWheelRightID);

    m_climberLeft.setInverted(true);
    m_climberRight.setInverted(false);

  }

   public void setClimberWheelPower(double power) {
    m_climberLeft.set(ControlMode.PercentOutput, power);
    m_climberRight.set(ControlMode.PercentOutput, power);
   }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

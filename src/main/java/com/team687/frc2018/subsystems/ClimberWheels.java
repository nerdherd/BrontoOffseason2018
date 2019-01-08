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
import com.team687.frc2018.constants.SuperstructureConstants;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    m_climberLeft.setSensorPhase(true);
    m_climberRight.setSensorPhase(false);

    m_climberLeft.config_kP(0, SuperstructureConstants.kLeftClimberP);
    m_climberLeft.config_kD(0, SuperstructureConstants.kLeftClimberD);

    m_climberRight.config_kP(0, SuperstructureConstants.kRightClimberP);
    m_climberRight.config_kD(0, SuperstructureConstants.kRightClimberD);
  }

   public void setClimberWheelPower(double power) {
    m_climberLeft.set(ControlMode.PercentOutput, power);
    m_climberRight.set(ControlMode.PercentOutput, power);
   }

   public void setClimberPosition(double leftPos, double rightPos) {
     m_climberLeft.set(ControlMode.Position, leftPos);
     m_climberRight.set(ControlMode.Position, rightPos);
   }


   public void reportToSmartDashboard() {
     SmartDashboard.putNumber("Climber Left Current", m_climberLeft.getOutputCurrent());
     SmartDashboard.putNumber("Climber Right Current", m_climberRight.getOutputCurrent());

     SmartDashboard.putNumber("Climber Left Voltage", m_climberLeft.getMotorOutputVoltage());
     SmartDashboard.putNumber("Climber Right Voltage", m_climberRight.getMotorOutputVoltage());
    
     SmartDashboard.putNumber("Climber Left Pos", m_climberLeft.getSelectedSensorPosition());
     SmartDashboard.putNumber("Climber Right Pos", m_climberRight.getSelectedSensorPosition());

   
    }

   public void resetClimberEncoders() {
     m_climberLeft.setSelectedSensorPosition(0);
     m_climberRight.setSelectedSensorPosition(0);
   }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

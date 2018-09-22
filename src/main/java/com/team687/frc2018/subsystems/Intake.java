package com.team687.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team687.frc2018.RobotMap;
import com.team687.frc2018.constants.SuperstructureConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Intake subsystem
 */

public class Intake extends Subsystem {

    private final TalonSRX m_rollers1, m_rollers2;
    private final DoubleSolenoid m_claw;

    public Intake() {
    m_rollers1 = new TalonSRX(RobotMap.kIntakeRollers1ID);
    m_rollers2 = new TalonSRX(RobotMap.kIntakeRollers2ID);

	m_rollers1.setNeutralMode(NeutralMode.Coast);
	m_rollers1.setStatusFramePeriod(StatusFrame.Status_1_General, 20, 0);

	m_rollers1.setInverted(true);

	m_rollers1.configPeakOutputForward(1, 0);
	m_rollers1.configPeakOutputReverse(-1, 0);
	m_rollers1.enableCurrentLimit(false);

    m_rollers2.setNeutralMode(NeutralMode.Coast);
	m_rollers2.setStatusFramePeriod(StatusFrame.Status_1_General, 20, 0);

	m_rollers2.setInverted(false);

	m_rollers2.configPeakOutputForward(1, 0);
	m_rollers2.configPeakOutputReverse(-1, 0);
    m_rollers2.enableCurrentLimit(false);
    
	m_claw = new DoubleSolenoid(RobotMap.kIntakeClawID1, RobotMap.kIntakeClawID2);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void openClaw() {
	m_claw.set(DoubleSolenoid.Value.kForward);
    }

    public void closeClaw() {
	m_claw.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isClawOpen() {
	return m_claw.get() == DoubleSolenoid.Value.kReverse;
    }

    public boolean hasCube() {
	// return !m_switch.get();
	return isMaxCurrent();
    }

    public boolean isMaxCurrent() {
	return getCurrent() > SuperstructureConstants.kRollerMaxCurrent;
    }

    public void setRollerPower(double power) {
    m_rollers1.set(ControlMode.PercentOutput, power);
    m_rollers2.set(ControlMode.PercentOutput, power);
    }

    public double getVoltage() {
	return m_rollers1.getMotorOutputVoltage();
    }

    public double getCurrent() {
	return m_rollers1.getOutputCurrent();
    }

    public void reportToSmartDashboard() {
	// ----- COMMENT THESE OUT WHEN GOING TO FIELD ----- //
//	SmartDashboard.putNumber("Roller Voltage", getVoltage());
//	SmartDashboard.putNumber("Roller Current", getCurrent());
//	SmartDashboard.putBoolean("Has Cube", hasCube());
//	SmartDashboard.putBoolean("Reached Max Current", isMaxCurrent());
	// ----- COMMENT THESE OUT WHEN GOING TO FIELD ----- //

	SmartDashboard.putBoolean("Claw Open", isClawOpen());
    }

}

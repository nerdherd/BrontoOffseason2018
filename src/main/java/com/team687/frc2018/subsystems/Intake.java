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

    private final TalonSRX m_intakeTop, m_intakeBottom;
    private final DoubleSolenoid m_claw;

    public Intake() {
    m_intakeTop = new TalonSRX(RobotMap.kIntakeRollers1ID);
    m_intakeBottom = new TalonSRX(RobotMap.kIntakeRollers2ID);

	m_intakeTop.setNeutralMode(NeutralMode.Coast);
	m_intakeTop.setStatusFramePeriod(StatusFrame.Status_1_General, 20, 0);

	m_intakeTop.setInverted(true);

	m_intakeTop.configPeakOutputForward(1, 0);
    m_intakeTop.configPeakOutputReverse(-1, 0);
    m_intakeTop.configContinuousCurrentLimit(20);
	m_intakeTop.enableCurrentLimit(true);

    m_intakeBottom.setNeutralMode(NeutralMode.Coast);
	m_intakeBottom.setStatusFramePeriod(StatusFrame.Status_1_General, 20, 0);

	m_intakeBottom.setInverted(true);

	m_intakeBottom.configPeakOutputForward(1, 0);
    m_intakeBottom.configPeakOutputReverse(-1, 0);
    m_intakeTop.configContinuousCurrentLimit(20);
    m_intakeBottom.enableCurrentLimit(true);
    
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

    public void setRollerPower(double power1, double power2) {
    m_intakeTop.set(ControlMode.PercentOutput, power1);
    m_intakeBottom.set(ControlMode.PercentOutput, power2);

    }

    public double getVoltage() {
	return m_intakeTop.getMotorOutputVoltage();
    }

    public double getCurrent() {
	return m_intakeTop.getOutputCurrent();
    }

    public void reportToSmartDashboard() {
	// ----- COMMENT THESE OUT WHEN GOING TO FIELD ----- //
//	SmartDashboard.putNumber("Roller Voltage", getVoltage());
//	SmartDashboard.putNumber("Roller Current", getCurrent());
//	SmartDashboard.putBoolean("Has Cube", hasCube());
//	SmartDashboard.putBoolean("Reached Max Current", isMaxCurrent());
	// ----- COMMENT THESE OUT WHEN GOING TO FIELD ----- //

    // SmartDashboard.putBoolean("Claw Open", isClawOpen());
    SmartDashboard.putNumber("Intake top current", m_intakeTop.getOutputCurrent());
    SmartDashboard.putNumber("Intake bottom current", m_intakeBottom.getOutputCurrent());

    }

}

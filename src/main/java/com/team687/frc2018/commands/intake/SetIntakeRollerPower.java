package com.team687.frc2018.commands.intake;

import com.team687.frc2018.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetIntakeRollerPower extends Command {

    private double m_power1, m_power2;


    public SetIntakeRollerPower(double power1, double power2) {
    m_power1 = power1;
    m_power2 = power2;
	requires(Robot.intake);
    }

    @Override
    protected void initialize() {
	// SmartDashboard.putString("Current Intake Command", "SetIntakeRollerPower: " + m_power);
    }

    @Override
    protected void execute() {
	Robot.intake.setRollerPower(m_power1, m_power2);
    }

    @Override
    protected boolean isFinished() {
	return false;
    }

    @Override
    protected void end() {
	Robot.intake.setRollerPower(0, 0);
    }

    @Override
    protected void interrupted() {
	end();
    }
}
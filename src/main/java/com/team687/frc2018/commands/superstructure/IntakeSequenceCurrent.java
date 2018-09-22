package com.team687.frc2018.commands.superstructure;

import com.team687.frc2018.Robot;
import com.team687.frc2018.constants.SuperstructureConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSequenceCurrent extends Command {

    private boolean m_isFirstPartDone;
    private double m_timerStart;

    public IntakeSequenceCurrent() {
	// requires(Robot.arm);
	// requires(Robot.wrist);
	requires(Robot.intake);
    }

    @Override
    protected void initialize() {
	SmartDashboard.putString("Current Command", "IntakePosition");
    // m_isFirstPartDone = false;
    m_timerStart = Timer.getFPGATimestamp();
    Robot.intake.openClaw();
    }

    @Override
    protected void execute() {
	// if (!m_isFirstPartDone) {
	//     Robot.wrist.setPosition(SuperstructureConstants.kWristIntakePos);
	//     Robot.arm.setPosition(SuperstructureConstants.kArmOffsetPos);
	//     if (Robot.wrist.getPosition() < SuperstructureConstants.kWristIntakePos + 500) {
	// 	Robot.intake.setRollerPower(0.66);
	// 	Timer.delay(.330);
	//     }
	// } else if (m_isFirstPartDone) {
	//     Robot.intake.setRollerPower(0.1);
	//     Robot.wrist.setPosition(SuperstructureConstants.kWristStowArmOffsetPos);
	// }
	// if (Robot.intake.hasCube()) {
	//     Timer.delay(.254);
	//     m_isFirstPartDone = true;
    // }
        if (Timer.getFPGATimestamp() > (m_timerStart + 0.5)) {
            Robot.intake.closeClaw();
        }
    }

    @Override
    protected boolean isFinished() {
	return Timer.getFPGATimestamp() > (m_timerStart + 0.5);
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
	end();
    }

}
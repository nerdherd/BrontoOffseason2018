package com.team687.frc2018.commands.drive.characterization;

import com.team687.frc2018.constants.DriveConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team687.frc2018.Robot;

public class VelocityPIDF implements Runnable {

    private double m_leftDesiredVel, m_rightDesiredVel, m_time, m_prevTime, m_startTime;
    private double m_leftError, m_rightError, m_leftPrevError, m_rightPrevError;
    private double m_leftVoltage, m_rightVoltage;
    private boolean m_started;

    public VelocityPIDF() {
        m_started = false;
    }

    public void setVelocity(double leftVel, double rightVel) {
        m_leftDesiredVel = leftVel;
        m_rightDesiredVel = rightVel;
    }

    @Override
    public void run() {
        // basically the initialize function in a command
        if (!m_started) {
            m_startTime = Timer.getFPGATimestamp();
            m_prevTime = Timer.getFPGATimestamp();
            m_leftPrevError = m_leftDesiredVel - Robot.drive.getLeftMasterSpeed();
            m_rightPrevError = m_rightDesiredVel - Robot.drive.getRightMasterSpeed();
            m_started = true;
        }

        // execute function
        m_time = Timer.getFPGATimestamp() - m_startTime;
        m_leftError = m_leftDesiredVel - Robot.drive.getLeftMasterSpeed();
        m_rightError = m_rightDesiredVel - Robot.drive.getRightMasterSpeed();

        // m_leftVoltage = DriveConstants.kLeftStatic * Math.signum(m_leftDesiredVel) + DriveConstants.kLeftV * m_leftDesiredVel 
        // + m_leftError * DriveConstants.kLeftVelocityP + DriveConstants.kLeftVelocityD
        // * (m_leftError - m_leftPrevError)/(m_time - m_prevTime);
        // m_rightVoltage = DriveConstants.kRightStatic * Math.signum(m_rightDesiredVel) + DriveConstants.kRightV * m_rightDesiredVel 
        // + m_rightError * DriveConstants.kRightVelocityP + DriveConstants.kRightVelocityD
        // * (m_rightError - m_rightPrevError)/(m_time - m_prevTime);
        Robot.drive.setVoltage(m_leftVoltage, m_rightVoltage);
        Robot.drive.addDesiredVelocities(m_leftDesiredVel, m_rightDesiredVel);
        m_prevTime = m_time;
        m_leftPrevError = m_leftError;
        m_rightPrevError = m_rightError;
        SmartDashboard.putNumber("VelocityPID", m_leftDesiredVel);
    }

    public void stop() {
        m_rightDesiredVel = 0;
        m_leftDesiredVel = 0;
        m_started = false;
    }

}
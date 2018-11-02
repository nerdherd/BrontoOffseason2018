package com.team687.frc2018.utilities;

import edu.wpi.first.wpilibj.Timer;

public class MotionProfile {

    private double m_cruiseVel, m_accel, m_dist, m_startTime, m_time;
	private double m_accelTime, m_maxAccelDist, m_maxAccelTime, m_cruiseDist, m_cruiseTime, m_vel;
    private boolean m_isTrapezoidal;
    
    public MotionProfile(double cruiseVel, double accel, double distance) {
        m_cruiseVel = cruiseVel;
    	m_accel = accel;
        m_dist = distance;
        m_startTime = Timer.getFPGATimestamp();
    	m_maxAccelTime = m_cruiseVel/m_accel;	
    	//d = vt + 1/2at^2
    	m_maxAccelDist = 0.5 * m_accel * Math.pow(m_maxAccelTime, 2);
    			
    	if (m_dist < 2 * m_maxAccelDist) {
    		m_isTrapezoidal = false;
    		m_accelTime = Math.sqrt(m_dist/m_accel);
    	}
    	else if (m_dist > 2 * m_maxAccelDist) {
    		m_accelTime = Math.sqrt((2*m_maxAccelDist)/m_accel);
    		m_isTrapezoidal = true;
    		m_cruiseDist = m_dist - 2 * m_maxAccelDist;
    		m_cruiseTime = (m_cruiseDist/m_cruiseVel) + m_accelTime;  		
    	}
    }

    public double getVelocity(double time) {
        m_time = time;
        if (m_isTrapezoidal) {
    		if (m_time <= m_accelTime) {
    			m_vel = m_accel * m_time;
    		}
    		else if (m_time > m_accelTime && m_time < m_cruiseTime) {
    			m_vel = m_cruiseVel;
    		}
    		else if (m_time >= m_cruiseTime) {
    			m_vel = -m_accel * (m_time - m_accelTime - m_cruiseTime);
    		}
    	}
    	else {
    		if (m_time <= m_accelTime) {
    			m_vel = m_accel * m_time;
    		}
    		else if (m_time > m_accelTime) {
    			m_vel = -m_accel * (m_time - m_accelTime) + (m_accel * m_accelTime);
    		}
        }
        return m_vel;
    }
}
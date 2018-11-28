package com.team687.frc2018.utilities;


import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import java.util.Arrays;
import java.util.List;

public class TrajectoryFollower {

    private Trajectory m_trajectory;
    private Segment m_robotSegment, m_lookaheadSegment;
    private Boolean m_goingForwards;

    private double m_robotX, m_robotY, m_targetAngle, m_goalX, m_goalY, m_velocity,
            m_error, m_leftDesiredVel,
            m_rightDesiredVel, m_kP, m_kD, m_turn, m_dT, m_lastError;
    private int m_lookaheadIndex, m_robotIndex, m_lookahead;
    private List<Segment> m_trajectoryList;




    public TrajectoryFollower(Trajectory traj, int lookahead, boolean goingForwards, double kP, double kD) {
        m_trajectory = traj;
        m_lookahead = lookahead;
        m_goingForwards = goingForwards;
        m_robotIndex = 0;
        m_kP = kP;
        m_kD = kD;
        m_trajectoryList = Arrays.asList(m_trajectory.segments);
        m_lookaheadIndex = 0;
        m_lastError = 0;

    }

    public void calculate(double robotX, double robotY, double robotTheta, double dT) {
        m_robotX = robotX;
        m_robotY = robotY;
        m_robotSegment = getClosestSegment(m_robotX, m_robotY, m_trajectory, m_robotIndex, 5);
        m_robotIndex = m_trajectoryList.indexOf(m_robotSegment);
        m_lookaheadIndex = m_robotIndex + m_lookahead;
        if (m_lookaheadIndex > m_trajectory.length() - 1) {
            m_lookaheadIndex = m_trajectory.length() - 1;
        }
        m_lookaheadSegment = m_trajectory.get(m_lookaheadIndex);
        m_goalX = m_lookaheadSegment.x;
        m_goalY = m_lookaheadSegment.y;
//        m_angle = -(360 - robotTheta) % 360;
        m_velocity = m_robotSegment.velocity;
        if (!m_goingForwards) {
            m_velocity *= -1;
            robotTheta += 180;
        }
        m_targetAngle = Math.toDegrees(Math.atan2(m_goalX - m_robotX, m_goalY - m_robotY));
        m_error = Pathfinder.boundHalfDegrees(m_targetAngle - robotTheta);
        m_turn = m_error * m_kP + (m_error - m_lastError)/dT * m_kD;
        m_leftDesiredVel = m_velocity + m_turn;
        m_rightDesiredVel = m_velocity - m_turn;
    }

    public boolean isFinished() {
        return m_trajectory.length() - 3 == m_robotIndex;
    }

    public double getLeftVelocity() {
        return m_leftDesiredVel;
    }

    public double getRightVelocity() {
        return m_rightDesiredVel;
    }

    public static Trajectory.Segment getClosestSegment(double x, double y, Trajectory trajectory, int index, int range) {
        double min = 1000000;
        int counter = index - range;
        int max = index + range;
        if (max > trajectory.length() - 1) {
            max = trajectory.length() - 1;
        }
        Trajectory.Segment seg;
        Trajectory.Segment closestSeg = trajectory.get(0);
        double dist;
        while (counter != max) {
            if (counter < 0) {
                counter = 0;
            }
            seg = trajectory.get(counter);
            dist = NerdyMath.distanceFormula(x, y, seg.x, seg.y);
            if (dist < min) {
                min = dist;
                closestSeg = seg;
            }
            counter += 1;
        }
        return closestSeg;
    }
}
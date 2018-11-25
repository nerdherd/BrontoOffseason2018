package com.team687.frc2018.utilities;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import java.util.Arrays;
import java.util.List;

public class AdaptivePurePursuitController {

    private Trajectory m_trajectory;
    private Segment m_robotSegment, m_lookaheadSegment;
    private Boolean m_goingForwards;

    private double m_robotX, m_robotY, m_angle, m_targetAngle, m_goalX, m_goalY, m_xOffset, m_velocity,
            m_innerVelocity, m_error, m_driveRadius, m_width, m_leftDesiredVel,
            m_rightDesiredVel, m_lookaheadDistance;
    private int m_lookaheadIndex, m_robotIndex, m_lookahead;
    private List m_trajectoryList;



    public AdaptivePurePursuitController(Trajectory traj, int lookahead, boolean goingForwards, double width) {
        m_trajectory = traj;
        m_lookahead = lookahead;
        m_goingForwards = goingForwards;
        m_width = width;
        m_robotIndex = 0;
        m_trajectoryList = Arrays.asList(m_trajectory.segments);
        m_lookaheadIndex = 0;

    }

    public void calculate(double robotX, double robotY, double robotTheta) {
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
        m_angle = -(360 - robotTheta) % 360;
        m_targetAngle = Math.toDegrees(Math.atan2(m_goalX - m_robotX, m_goalY - m_robotY));
        m_error = Pathfinder.boundHalfDegrees(m_targetAngle - m_angle);
        m_lookaheadDistance = NerdyMath.distanceFormula(m_robotX, m_robotY, m_goalX, m_goalY);
        m_xOffset = Math.abs(m_lookaheadDistance * Math.cos(Math.toRadians(m_error)));
        m_velocity = m_robotSegment.velocity;
        if (!m_goingForwards) {
            m_velocity *= -1;
        }
        if (m_xOffset > 0) {
            m_driveRadius = (Math.pow(m_lookaheadDistance, 2)) / (2 * m_xOffset);
            m_innerVelocity = m_velocity * (m_driveRadius - (m_width / 2)) / (m_driveRadius + (m_width / 2));
            if (m_goingForwards) {
                if (Math.signum(m_error) == -1) {
                    m_leftDesiredVel = m_innerVelocity;
                    m_rightDesiredVel = m_velocity;
                } else {
                    m_leftDesiredVel = m_velocity;
                    m_rightDesiredVel = m_innerVelocity;
                }
            } else {
                if (Math.signum(m_error) == 1) {
                    m_leftDesiredVel = m_innerVelocity;
                    m_rightDesiredVel = m_velocity;
                } else {
                    m_leftDesiredVel = m_velocity;
                    m_rightDesiredVel = m_innerVelocity;
                }
            }
        }
        else {
            m_leftDesiredVel = m_velocity;
            m_rightDesiredVel = m_velocity;
        }
//        System.out.println(m_robotIndex);
//        System.out.println(Math.cos(Math.toRadians(m_error)));
        System.out.println(m_lookaheadDistance);

    }
    public boolean isFinished() {
        return m_trajectory.length()-2 == m_robotIndex;
    }

    public double getLeftVelocity() {
        return m_leftDesiredVel;
    }

    public double getRightVelocity() {
        return m_rightDesiredVel;
    }

    public static Trajectory.Segment getClosestSegment(double x, double y, Trajectory trajectory, int index, int range) {
        double min = 1000000;
        int segIndex = 0;
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
package com.team687.frc2018.utilities;

import java.util.Arrays;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

public class PurePursuitController {

    private Trajectory m_trajectory;
    private Segment m_currentSegment, m_segment2;
    private int m_index;
    private Boolean m_goingForwards;

    private double m_lookahead, m_x1, m_x2, m_y1, m_y2,
            m_a, m_b, m_c, m_goalX1, m_goalX2, m_goalY1, m_goalY2,
            m_startTime, m_time, m_robotX, m_robotY, m_slope,
            m_yInt, m_error1, m_error2, m_angle, m_targetAngle1,
            m_targetAngle2, m_goalX, m_goalY, m_xOffset, m_velocity,
            m_innerVelocity, m_error, m_driveRadius, m_width, m_leftDesiredVel, m_rightDesiredVel;


    public PurePursuitController(Trajectory traj, double lookahead, boolean goingForwards, double width) {
        m_trajectory = traj;
        m_lookahead = lookahead;
        m_goingForwards = goingForwards;
        m_width = width;
        m_index = 1;
    }

    public void calculate(double robotX, double robotY, double robotTheta) {
        m_robotX = robotX;
        m_robotY = robotY;
        m_currentSegment = getClosestSegment(m_robotX, m_robotY, m_trajectory, m_index, 5);
        m_index = Arrays.asList(m_trajectory.segments).indexOf(m_currentSegment);
//        System.out.println(m_index);
        if (m_index < 1) {
            m_index = 1;
        }
        m_segment2 = getCloserSegment(m_robotX, m_robotY, m_trajectory.get(m_index - 1), m_trajectory.get(m_index + 1));
        m_x1 = m_currentSegment.x;
        m_y1 = m_currentSegment.y;
        m_x2 = m_segment2.x;
        m_y2 = m_segment2.y;
        if (m_x2 == m_x1) {
            m_a = 1;
            m_b = -2 * m_robotY;
            m_c = (m_robotY * m_robotY) - (m_lookahead * m_lookahead) + Math.pow(m_x1 - m_robotX, 2);
            m_goalY1 = (-m_b - Math.sqrt(m_b * m_b - 4 * m_a * m_c)) / (2 * m_a);
            m_goalY2 = (-m_b + Math.sqrt(m_b * m_b - 4 * m_a * m_c)) / (2 * m_a);
            m_goalX1 = m_x1;
            m_goalX2 = m_x2;
        }
        else {
            m_slope = (m_y2 - m_y1) / (m_x2 - m_x1);
            m_yInt = m_y2 - m_slope * m_x2;

            // quadratic formula, solve for intercept of line from path and lookahead circle from robot
            m_a = (1 + m_slope * m_slope);
            m_b = (-2 * m_robotX) + (2 * m_slope * (m_yInt - m_robotY));
            m_c = (m_robotX * m_robotX) + Math.pow((m_yInt - m_robotY), 1) - m_lookahead * m_lookahead;

            m_goalX1 = (-m_b - Math.sqrt(m_b * m_b - 4 * m_a * m_c)) / (2 * m_a);
            m_goalX2 = (-m_b + Math.sqrt(m_b * m_b - 4 * m_a * m_c)) / (2 * m_a);
            m_goalY1 = m_slope * m_goalX1 + m_yInt;
            m_goalY2 = m_slope * m_goalX2 + m_yInt;
        }


        m_angle = -(360 - robotTheta) % 360;

        m_targetAngle1 = Math.toDegrees(Math.atan2(m_goalX1 - m_robotX, m_goalY1 - m_robotY));

        m_error1 = m_targetAngle1 - m_angle;
        if (m_error1 >= 180) {
            m_error1 -= 360;
        }
        if (m_error1 <= -180) {
            m_error1 += 360;
        }

        m_targetAngle2 = Math.toDegrees(Math.atan2(m_goalX2 - m_robotX, m_goalY2 - m_robotY));

        m_error2 = m_targetAngle2 - m_angle;
        if (m_error2 >= 180) {
            m_error2 -= 360;
        }
        if (m_error2 <= -180) {
            m_error2 += 360;
        }

        if (Math.abs(m_error1) <= Math.abs(m_error2)) {
            m_error = m_error1;
            m_goalX = m_goalX1;
            m_goalY = m_goalY1;
        } else {
            m_error = m_error2;
            m_goalX = m_goalX2;
            m_goalY = m_goalY2;
        }

        m_xOffset = NerdyMath.distanceFormula(m_robotX, m_robotY, m_goalX, m_goalY)
                * Math.cos(Math.toRadians(m_error));


        m_velocity = m_currentSegment.velocity;
        if (!m_goingForwards) {
            m_velocity *= -1;
        }
        if (m_xOffset > 0) {
            m_driveRadius = (m_lookahead * m_lookahead) / (2 * m_xOffset);
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
        m_index++;
    }

    public boolean isFinished() {
        return m_trajectory.length()-2 == m_index;
    }

    public double getLeftVelocity() {
        return m_leftDesiredVel;
    }

    public double getRightVelocity() {
        return m_rightDesiredVel;
    }

    private Segment getCloserSegment(double x, double y, Segment seg1, Segment seg2) {
        double dist1 = NerdyMath.distanceFormula(x, y, seg2.x, seg2.y);
        double dist2 = NerdyMath.distanceFormula(x, y, seg1.x, seg1.y);
        if (dist1 <= dist2) {
            return seg1;
        }
        else {
            return seg2;
        }
    }

    public static Segment getClosestSegment(double x, double y, Trajectory trajectory, int index, int range) {
        double min = 1000000;
        int segIndex = 0;
        int counter = index - range;
        int max = index + range;
        if (max > trajectory.length() - 1) {
            max = trajectory.length() - 1;
        }
        Segment seg;
        Segment closestSeg = trajectory.get(0);
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

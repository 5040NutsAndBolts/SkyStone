package org.firstinspires.ftc.teamcode.PurePursuit;


import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.competition.helperclasses.PID;
import org.firstinspires.ftc.teamcode.competition.helperclasses.Point;

import java.util.ArrayList;


public class PurePursuit {

    private Hardware robot;
    private MecanumDrive drive;
    // PID controller for movement
    public PID pos;

    // Has PID been started
    private boolean toPID = false;
    // Last position selected on path
    public double[] lastGoal = new double[3];
    // Goal point the robot is headed towards
    public double[] pointToMoveTo;

    public PurePursuit(Hardware r) {
        robot = r;
        drive = new MecanumDrive(robot);
    }

    /**
     * Determines how far a point is along the path
     *
     * @param location Point on Pure Pursuit path
     * @param p        List of pure pursuit paths
     * @return Distance between points
     */
    private double distanceAlongPath(Point location, Point p) {
        // Distance formula to determine total distance
        return Math.sqrt(Math.pow(location.x - p.x, 2) + Math.pow(location.y - p.y, 2));
    }

    /**
     * Looks at a path and finds the intersections of the look ahead circle
     *
     * @param iRel              Index of the relevant segment
     * @param p                 Path for Pure Pursuit to follow
     * @param lookAheadDistance Radius of the look ahead circle
     * @return The point the robot will move to
     */
    private double[] goalPoint(int iRel, ArrayList<WayPoint> p, double lookAheadDistance) {
        try {
            double x1 = p.get(iRel - 1).x - robot.x;
            double y1 = p.get(iRel - 1).y - robot.y;
            double x2 = p.get(iRel).x - robot.x;
            double y2 = p.get(iRel).y - robot.y;

            // If 2 points next to each other on the same path share x/y values, offset them
            if (x1 == x2)
                x2 += .000001;
            if (y1 == y2)
                y2 += .000001;

            double dx = x2 - x1;
            double dy = y2 - y1;
            double dr = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
            double D = (x1 * y2) - (x2 * y1);
            int sgn = 1;
            if (dy < 0)
                sgn = -1;
            else if (dy == 0)
                sgn = 0;

            //if the last segment is selected and there is no goal on that segment go to the end of the path
            if (iRel == p.size() - 2 && Math.sqrt(Math.pow(y2, 2) + Math.pow(x2, 2)) <= lookAheadDistance) {
                return goalPoint(iRel, p, lookAheadDistance - 1);
            }
            double radicand = Math.pow(lookAheadDistance, 2) * Math.pow(dr, 2) - Math.pow(D, 2);

            //if no points on the line intersect the circle where the robot is looking move to the next line 
            if (radicand < 0) {
                return goalPoint(iRel - 1, p, lookAheadDistance);
            }

            //finding the 2 intersections of the line and line ahead circle
            double sqrt = Math.sqrt(radicand);
            double g1x = (D * dy + sgn * dx * sqrt) / Math.pow(dr, 2);
            double g2x = (D * dy - sgn * dx * sqrt) / Math.pow(dr, 2);
            double g1y = (-D * dx + Math.abs(dy) * sqrt) / Math.pow(dr, 2);
            double g2y = (-D * dx - Math.abs(dy) * sqrt) / Math.pow(dr, 2);
            double xLower = x1;
            double xUpper = x2;
            if (x2 < x1) {
                xLower = x2;
                xUpper = x1;
            }

            //determines which of the points is further along the path then for that intersection if it falls on the line segment that is the goal point otherwise move to the next line segment
            if (distanceAlongPath(new Point(g1x + robot.x, g1y + robot.y), new Point(p.get(iRel).x, p.get(iRel).y)) < distanceAlongPath(new Point(g2x + robot.x, g2y + robot.y), new Point(p.get(iRel).x, p.get(iRel).y))) {
                if (g1x > xLower && g1x < xUpper) {
                    return new double[]{g1x + robot.x, g1y + robot.y, iRel};
                }
                return goalPoint(iRel - 1, p, lookAheadDistance);
            } else {
                if (g2x > xLower && g2x < xUpper) {
                    return new double[]{g2x + robot.x, g2y + robot.y, iRel};
                }
                return goalPoint(iRel - 1, p, lookAheadDistance);
            }
        }
        //if there are no intersections on any line go to the last goal point
        catch (IndexOutOfBoundsException e) {
            return lastGoal;
        }
    }

    /**
     * Moves the robot to a specific point
     *
     * @param goalPoint         Specific point the robot will move to
     * @param p                 List of Pure Pursuit points
     * @param iRel              Index of the relevant segment
     * @param lookAheadDistance Radius of look ahead circle
     * @param speed             Max speed robot will move at
     */
    private void goToPosition(Point goalPoint, ArrayList<WayPoint> p, int iRel, double lookAheadDistance, double speed, double turnSpeed) {
        double distanceToTarget = Math.hypot(goalPoint.x - robot.x, goalPoint.y - robot.y);
        double distanceToFinal = Math.hypot(p.get(p.size() - 1).x - robot.x, p.get(p.size() - 1).y - robot.y);
        double absoluteAngleToTarget = Math.atan2(goalPoint.y - robot.y, goalPoint.x - robot.x);
        double relativeAngleToPoint = absoluteAngleToTarget - (HelperMethods.angleWrap(robot.theta - Math.PI / 2));

        // The X/Y components of a vector pointing to the goal
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        // Calculates the power to move at in the X/Y direction
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

        double movementTurn;

        // Wat percent of the speed to move at, 1 if the robot is not heading toward the final point
        double speedPercentage = 1;

        // If the robot is approaching the final point, slow down using PID
        if (distanceToFinal < lookAheadDistance || toPID) {
            speedPercentage = pos.getPID();
            toPID = true;
        } else
            pos.resetPid();

        // Turn with power proportional to angle of WayPoint
        movementTurn = robot.theta - p.get(iRel).angle;
        if (movementTurn > Math.PI)
            movementTurn = Math.PI - movementTurn;
        else if (movementTurn < -Math.PI)
            movementTurn = -movementTurn - Math.PI;

        // Cap the turning speed
        if (movementTurn > 1.5)
            movementTurn = 1.5;
        else if (movementTurn < -1.5)
            movementTurn = -1.5;

        // If the robot is sufficiently close to the goal angle increase the speed to allow the robot to hit the angle
        if (Math.abs(movementTurn) < .5)
            movementTurn *= 1.1;


        // Drive towards point
        drive.drive(-movementYPower * speedPercentage / speed, movementXPower * speedPercentage / speed, movementTurn / 6 / turnSpeed);

    }

    //follow path with pure pursuit

    /**
     * Initializes the Pure Pursuit path
     *
     * @param path List of points for Pure Pursuit path
     */
    public void initPath(ArrayList<WayPoint> path) {

        toPID = false;

        // Add a point ofr calculations on the last segment
        path.add(path.get(path.size() - 1));

        // Create the PID controller
        pos = new PID(Math.hypot(path.get(path.size() - 1).x - robot.x, path.get(path.size() - 1).y - robot.y), .06, .005, .05);

        // Sets the point where the robot moves to when the path moves outside its FOV
        lastGoal[0] = path.get(0).x;
        lastGoal[1] = path.get(0).y;
    }

    /**
     * Initializes the Pure Pursuit path with specific P.I.D. values
     *
     * @param path List of points for Pure Pursuit path
     * @param P    Proportional of PID
     * @param I    Integral of PID
     * @param D    Derivative of PID
     */
    public void initPath(ArrayList<WayPoint> path, double P, double I, double D) {
        // initializes the current path
        initPath(path);

        //create the PID controller
        pos = new PID(Math.hypot(path.get(path.size() - 1).x - robot.x, path.get(path.size() - 1).y - robot.y), P, I, D);
    }

    /**
     * Makes the robot follow the Pure Pursuit path
     *
     * @param p                 List of points for Pure Pursuit path
     * @param lookAheadDistance Radius of look ahead circle
     * @param speed             Max speed the robot will move at
     */
    public void followPath(ArrayList<WayPoint> p, double lookAheadDistance, double speed, double turnSpeed) {
        // Update PID
        pos.update(Math.hypot(p.get(p.size() - 1).x - robot.x, p.get(p.size() - 1).y - robot.y));

        double[] pointToMoveTo = goalPoint(p.size() - 2, p, lookAheadDistance);
        lastGoal = pointToMoveTo;
        Point g = new Point(pointToMoveTo[0], pointToMoveTo[1]);
        goToPosition(g, p, (int) pointToMoveTo[2], lookAheadDistance, speed, turnSpeed);
    }

}

package org.firstinspires.ftc.teamcode.PurePursuit;



import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.competition.MecanumDrive;
import org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition;
import org.firstinspires.ftc.teamcode.opencv.core.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition.worldYPosition;


public class PurePursuit
{
    Hardware robot = new Hardware();
    MecanumDrive drive = new MecanumDrive(robot);
    
    //will possibly center the search for goal point around project location in the future
    /*public Point projectedLocation()
    {


    }
    */
    //determine how far a point is along the path
    private double distanceAlongPath(Point location, Point p)
    {

        return Math.sqrt(Math.pow(location.x-p.x,2)+Math.pow(location.y-p.y,2));

    }
//returns the point that pure pursuit will move to by looking at the line segments and finding the intersections of a circle centered at the robot with radius of the look ahead distance
    private double[] goalPoint(int iRel, ArrayList<WayPoint> p, double lookAheadDistance)
    {

        try {
            double x1 = p.get(iRel-1).x - worldXPosition;
            double y1 = p.get(iRel-1).y - worldYPosition;
            double x2 = p.get(iRel).x - worldXPosition;
            double y2 = p.get(iRel).y - worldYPosition;
            double dx = x2 - x1;
            double dy = y2 - y1;
            double dr = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
            double D = (x1 * y2) - (x2 * y1);
            int sgn = 1;
            if (dy < 0) {
                sgn = -1;
            }
            //if the last segment is selected and there is no goal on that segment go to the end of the path
            if(iRel==p.size()-2&&Math.sqrt(Math.pow(y2,2)+Math.pow(x2,2))<=lookAheadDistance)
                {return goalPoint(iRel,p,lookAheadDistance-1);}
            double radicand = Math.pow(lookAheadDistance, 2) * Math.pow(dr, 2) - Math.pow(D, 2);
            //if no points on the line intersect the circle where the robot is looking move to the next line 
            if(radicand<0)
                {

                    return goalPoint(iRel-1,p,lookAheadDistance);
                }

            //finding the 2 intersections
            double sqrt = Math.sqrt(radicand);
            double g1x = (D * dy + sgn * dx * sqrt) / Math.pow(dr, 2);
            double g2x = (D * dy - sgn * dx * sqrt) / Math.pow(dr, 2);
            double g1y = (-D * dx + Math.abs(dy) * sqrt )/ Math.pow(dr, 2);
            double g2y = (-D * dx - Math.abs(dy) * sqrt )/ Math.pow(dr, 2);
            double xLower=x1;
            double xUpper=x2;
            if(x2<x1)
            {
                xLower=x2;
                xUpper=x1;
            }

            //determines which of the points is further along the path then for that intersection if it falls on the line segment that is the goal point otherwise move to the next line segment
            if (distanceAlongPath(new Point(g1x+worldXPosition, g1y+worldYPosition), new Point(p.get(iRel).x,p.get(iRel).y)) < distanceAlongPath(new Point(g2x+worldXPosition, g2y+worldYPosition), new Point(p.get(iRel).x,p.get(iRel).y))) {
                
                if(g1x>xLower&&g1x<xUpper)
                {

                    return new double[]{g1x + worldXPosition, g1y + worldYPosition,iRel};

                }
                return goalPoint(iRel-1,p,lookAheadDistance);
            } else {


                if(g2x>xLower&&g2x<xUpper)
                {

                    return new double[]{g2x + worldXPosition, g2y + worldYPosition,iRel};

                }
                
                    return goalPoint(iRel-1,p,lookAheadDistance);}

        }
        //if there are no intersections on any line go to the first point on the path or if the last segment is reached move directly to the endpoint
        catch(IndexOutOfBoundsException e) {

                return new double[]{p.get(iRel).x, p.get(iRel).y, iRel};

               }


    }
    
    //move the robot to to a specified point
    private void goToPosition(Point goalPoint,ArrayList<WayPoint> p,int iRel)
    {

        double distanceToTarget = Math.hypot(goalPoint.x-worldXPosition,goalPoint.y-worldYPosition);
        double absoluteAngleToTarget = Math.atan2(goalPoint.y-worldYPosition,goalPoint.x-worldXPosition);
        double relativeAngleToPoint = absoluteAngleToTarget - (MathFunctions.angleWrap(worldAngle_rad-Math.PI/2));
        double relativeXToPoint = Math.cos(relativeAngleToPoint)*distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint)*distanceToTarget;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint)+Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint)+Math.abs(relativeXToPoint));
        double movementTurn=0;
        relativeAngleToPoint = absoluteAngleToTarget - (MathFunctions.angleWrap(worldAngle_rad));



        double turnPowerToGoal=0;
        if(iRel<p.size()-2) {
            try {


                    double absoluteAngleToSegment = Math.atan2(p.get(iRel).y-worldYPosition,p.get(iRel).x-worldXPosition);
                    double relativeAngleToSegment = absoluteAngleToSegment - (MathFunctions.angleWrap(worldAngle_rad));
                    while(relativeAngleToSegment>Math.PI)
                    {
                        relativeAngleToSegment-=2*Math.PI;
                    }
                    while(relativeAngleToSegment<-Math.PI)
                    {
                        relativeAngleToSegment+=2*Math.PI;
                    }
                    if(relativeAngleToSegment==0)
                        {relativeAngleToSegment=.000001;}
                    turnPowerToGoal = (relativeAngleToSegment ) / Math.sqrt(Math.abs(relativeAngleToSegment)) ;


                LineSegment nextSegment = new LineSegment(p.get(iRel).x, p.get(iRel).y, p.get(iRel+1).x, p.get(iRel+1).y);
                double distanceToPoint = distanceAlongPath(new Point(worldXPosition, worldYPosition), new Point(p.get(iRel).x,p.get(iRel).y));
                LineSegment relevantSegment = new LineSegment(p.get(iRel -1).x, p.get(iRel-1).y, p.get(iRel).x, p.get(iRel).y);
                double goalPercentage = distanceToPoint / (relevantSegment.getMagnitude()) + p.get(iRel).goalTurnPercentageBias;
                if(goalPercentage>1)
                    {goalPercentage=1;}
                else if(goalPercentage<0)
                    {goalPercentage=0;}
                double turnPowerAbsolute;
                if(iRel==p.size()-1)
                {

                    turnPowerAbsolute=turnPowerToGoal;

                }else
                {
                    double nextAngle=nextSegment.getAngle();
                    while(nextAngle>Math.PI)
                        {nextAngle-=Math.PI*2;}
                    while(nextAngle<-Math.PI)
                        {nextAngle+=Math.PI*2;}

                    double turnPowerToNextPoint = (nextAngle  - worldAngle_rad) / Math.sqrt(Math.abs(nextAngle  - worldAngle_rad));
                    turnPowerAbsolute = turnPowerToGoal * goalPercentage + turnPowerToNextPoint * (1 - goalPercentage);
                }
                /*if(MovementVars.movement_turn-turnPowerAbsolute>p.get(iRel).turnAccelerationCap)
                {

                    turnPowerAbsolute=MovementVars.movement_turn-p.get(iRel).turnAccelerationCap;

                }else if(turnPowerAbsolute-MovementVars.movement_turn>p.get(iRel).turnAccelerationCap)
                {
                    turnPowerAbsolute=MovementVars.movement_turn+p.get(iRel).turnAccelerationCap;
                }
                MovementVars.movement_turn = turnPowerAbsolute;

            */} catch (Exception e)
            {

                movementTurn = turnPowerToGoal;

            }
        }else
        {

            movementTurn = (relativeAngleToPoint) / Math.sqrt(Math.abs(relativeAngleToPoint )) ;

        }
        drive.drive(movementYPower,movementXPower,movementTurn);

    }

    //follow path with pure pursuit


    public void followPath(ArrayList<WayPoint> p,double lookAheadDistance)
    {

        p.add(p.get(p.size() - 1));

        //add a point for calculations on the last segment




            double[] d  = goalPoint(p.size() - 2, p, lookAheadDistance);
            Point g = new Point(d[0],d[1]);
            goToPosition(g,p,(int) d[2]);


    }

}

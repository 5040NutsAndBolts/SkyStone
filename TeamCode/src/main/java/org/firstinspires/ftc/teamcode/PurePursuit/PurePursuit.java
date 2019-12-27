package org.firstinspires.ftc.teamcode.PurePursuit;




import org.firstinspires.ftc.teamcode.Point;
import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.competition.helperclasses.PID;

import java.util.ArrayList;




public class PurePursuit
{


    boolean toPID=false;

    public PID pos;
    public double[] lastGoal = new double[3];
    Hardware robot;
    MecanumDrive drive;
    public PurePursuit(Hardware r)
    {

        robot=r;
        drive= new MecanumDrive(robot);

    }
    
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
            double x1 = p.get(iRel-1).x - robot.x;
            double y1 = p.get(iRel-1).y - robot.y;
            double x2 = p.get(iRel).x - robot.x;
            double y2 = p.get(iRel).y - robot.y;
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
            if (distanceAlongPath(new Point(g1x+robot.x, g1y+robot.y), new Point(p.get(iRel).x,p.get(iRel).y)) < distanceAlongPath(new Point(g2x+robot.x, g2y+robot.y), new Point(p.get(iRel).x,p.get(iRel).y))) {
                
                if(g1x>xLower&&g1x<xUpper)
                {

                    return new double[]{g1x + robot.x, g1y + robot.y,iRel};

                }
                return goalPoint(iRel-1,p,lookAheadDistance);
            } else {


                if(g2x>xLower&&g2x<xUpper)
                {

                    return new double[]{g2x + robot.x, g2y + robot.y,iRel};

                }
                
                    return goalPoint(iRel-1,p,lookAheadDistance);}

        }
        //if there are no intersections on any line go to the first point on the path or if the last segment is reached move directly to the endpoint
        catch(IndexOutOfBoundsException e) {

                return lastGoal;

               }


    }
    
    //move the robot to to a specified point
    private void goToPosition(Point goalPoint, ArrayList<WayPoint> p, int iRel,double goalTheta,double lookAheadDistance,double speed)
    {

        double distanceToTarget = Math.hypot(goalPoint.x-robot.x,goalPoint.y-robot.y);
        double distanceToFinal = Math.hypot(p.get(p.size()-1).x-robot.x,p.get(p.size()-1).y-robot.y);
        double absoluteAngleToTarget = Math.atan2(goalPoint.y-robot.y,goalPoint.x-robot.x);
        double relativeAngleToPoint = absoluteAngleToTarget - (MathFunctions.angleWrap(robot.theta-Math.PI/2));

        double relativeXToPoint = Math.cos(relativeAngleToPoint)*distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint)*distanceToTarget;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint)+Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint)+Math.abs(relativeXToPoint));
        double movementTurn=0;
        double turnSpeedPercentage=1;
        double speedPercentage = 1;


        if(distanceToFinal<lookAheadDistance||toPID)
        {
            speedPercentage=pos.getPID();
            toPID=true;
        }
        else
            pos.resetPid();


        relativeAngleToPoint = absoluteAngleToTarget - (MathFunctions.angleWrap(robot.theta));



        double turnPowerToGoal=0;
        if(iRel<p.size()-2) {
            try {


                    double absoluteAngleToSegment = Math.atan2(p.get(iRel).y-robot.y,p.get(iRel).x-robot.x);
                    double relativeAngleToSegment = absoluteAngleToSegment - (MathFunctions.angleWrap(robot.theta));
                    while(relativeAngleToSegment>Math.PI)
                    {
                        relativeAngleToSegment-=2*Math.PI;
                    }
                    while(relativeAngleToSegment<-Math.PI)
                    {
                        relativeAngleToSegment+=2*Math.PI;
                    }
                    if(relativeAngleToSegment==0)
                        {relativeAngleToSegment=.00000001;}
                    turnPowerToGoal = (relativeAngleToSegment ) / Math.sqrt(Math.abs(relativeAngleToSegment)) ;


                LineSegment nextSegment = new LineSegment(p.get(iRel).x, p.get(iRel).y, p.get(iRel+1).x, p.get(iRel+1).y);
                double distanceToPoint = distanceAlongPath(new Point(robot.x, robot.y), new Point(p.get(iRel).x,p.get(iRel).y));
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

                    double turnPowerToNextPoint = (nextAngle  - robot.theta) / Math.sqrt(Math.abs(nextAngle  - robot.theta));
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

            movementTurn = (robot.theta-goalTheta);
            movementTurn = movementTurn >Math.PI ? Math.PI-movementTurn:movementTurn<-Math.PI ? Math.PI+movementTurn:movementTurn;
            movementTurn*=5;

        }
        //drive towards point
        drive.drive(-movementYPower*speedPercentage/speed,movementXPower*speedPercentage/speed,movementTurn/5);

    }

    //follow path with pure pursuit


    public void initPath(ArrayList<WayPoint> p , double P, double I, double D)
    {

        toPID=false;
        p.add(p.get(p.size() - 1));
        //create the PID controller good starting PID .06,.005,.05
        pos = new PID(Math.hypot(p.get(p.size()-1).x-robot.x,p.get(p.size()-1).y-robot.y),P,I,D);

    }

    public void followPath(ArrayList<WayPoint> p,double lookAheadDistance,double goalTheta,double speed)
    {



        //add a point for calculations on the last segment



            pos.update(Math.hypot(p.get(p.size()-1).x-robot.x,p.get(p.size()-1).y-robot.y));
            double[] d  = goalPoint(p.size() - 2, p, lookAheadDistance);
            lastGoal=d;
            Point g = new Point(d[0],d[1]);
            goToPosition(g,p,(int) d[2],goalTheta,lookAheadDistance,speed);


    }

}

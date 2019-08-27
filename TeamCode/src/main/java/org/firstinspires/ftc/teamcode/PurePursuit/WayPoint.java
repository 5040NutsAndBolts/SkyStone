package org.firstinspires.ftc.teamcode.PurePursuit;

public class WayPoint
{


    double x;
    double y;
    double maxSpeed;
    double goalTurnPercentageBias;
    double turnAccelerationCap;
    public WayPoint(double x, double y, double maxSpeed, double goalTurnPercentageBias,double turnAccelerationCap)
    {

        this.x=x;
        this.y=y;
        this.maxSpeed=maxSpeed;
        this.goalTurnPercentageBias=goalTurnPercentageBias;
        this.turnAccelerationCap=turnAccelerationCap;

    }


}

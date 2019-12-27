package org.firstinspires.ftc.teamcode.PurePursuit;


import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;

public class CheckPoint extends Thread
{

    double x;
    double y;
    double r;

    public boolean isHit=false;

    Hardware robot;
    public CheckPoint(double x, double y, double r, Hardware robot)
    {

        this.x=x;
        this.y=y;
        this.r=r;
        this.robot = robot;

    }

    public void run()
    {

        while(!(robot.x>x-r && robot.x<x+r && robot.y>y-r && robot.y<y+r)) {
            robot.updatePositionRoadRunner();

            try {
                sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.intakeLeft.setPower(.25);
        }
        isHit=true;
        onHit();
        return;

    }

    public void onHit(){}

}

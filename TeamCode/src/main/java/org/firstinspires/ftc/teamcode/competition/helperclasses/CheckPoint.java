package org.firstinspires.ftc.teamcode.competition.helperclasses;


import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;

public class CheckPoint extends Thread
{

    double x;
    double y;
    double r;

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

        while(!(robot.y>x-r&&robot.x<x+r&&robot.y>y-r&&robot.y<y+r)) {
            try {
                sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        onHit();
        return;

    }

    public void onHit(){}

}

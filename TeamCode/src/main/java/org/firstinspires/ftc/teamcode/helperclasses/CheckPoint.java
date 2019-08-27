package org.firstinspires.ftc.teamcode.helperclasses;


import static org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.helperclasses.OdometryPosition.worldYPosition;

public class CheckPoint extends Thread
{

    double x;
    double y;
    double r;

    public CheckPoint(double x, double y,double r)
    {

        this.x=x;
        this.y=y;
        this.r=r;

    }

    public void run()
    {

        System.out.println("test");
        while(!(worldXPosition>x-r&&worldXPosition<x+r&&worldYPosition>y-r&&worldYPosition<y+r)) {
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

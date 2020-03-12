package org.firstinspires.ftc.teamcode.purepursuit;


import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.Point;

import java.util.ArrayList;

public class CheckPoint extends Thread {

    private Hardware robot;

    public double x;
    public double y;
    public double r;
    private boolean terminate = false;
    public boolean isHit = false;

    public CheckPoint(ArrayList<WayPoint> path, double r, Hardware robot) {
        this.x = path.get(path.size()-1).x;
        this.y = path.get(path.size()-1).y;
        this.r = r;
        this.robot = robot;
    }

    public void run() {
        while (!HelperMethods.withinCircle(new Point(robot.x, robot.y), new Point(x, y), r)
                && !terminate) {
            robot.updatePositionRoadRunner();

            try {
                sleep(10);
            } catch (Exception e){}
        }
        if (!terminate) {
            isHit = true;
            onHit();
        }
    }

    public void onHit() {
    }

    public void terminate() {
        terminate = true;
    }

}

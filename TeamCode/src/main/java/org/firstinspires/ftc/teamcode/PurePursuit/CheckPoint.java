package org.firstinspires.ftc.teamcode.PurePursuit;


import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class CheckPoint extends Thread {

    double x;
    double y;
    double r;
    boolean terminate = false;

    public boolean isHit = false;

    Hardware robot;

    public CheckPoint(double x, double y, double r, Hardware robot) {
        this.x = x;
        this.y = y;
        this.r = r;
        this.robot = robot;
    }

    public void run() {
        while (!(robot.x > x - r && robot.x < x + r && robot.y > y - r && robot.y < y + r && !terminate)) {
            robot.updatePositionRoadRunner();

            try {
                sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        if (!terminate) {
            isHit = true;
            onHit();
            return;
        }
    }

    public void onHit() {
    }

    public void terminate() {
        terminate = false;
    }

}

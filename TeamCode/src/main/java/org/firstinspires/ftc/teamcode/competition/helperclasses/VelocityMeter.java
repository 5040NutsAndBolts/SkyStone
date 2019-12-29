package org.firstinspires.ftc.teamcode.competition.helperclasses;

import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;

public class VelocityMeter implements Runnable {

    private Hardware robot;

    public static double velocity;
    public static double angularVelocity;
    public boolean stop = false;

    public VelocityMeter(Hardware robot) {
        this.robot = robot;
    }

    public void run() {
    }
}

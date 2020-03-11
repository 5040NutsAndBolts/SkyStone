package org.firstinspires.ftc.teamcode.competition.hardware;

public class CapstoneDropper {

    private Hardware robot;

    public boolean dropping = false;
    private int currentPWM = 1450;

    public CapstoneDropper(Hardware robot) {
        this.robot = robot;
    }

    public void reset() {
        currentPWM = 1450;
        robot.capstoneDropper.setPosition(1);
    }

    public void run() {
        if (dropping) {
            robot.capstoneDropper.setPosition(currentPWM-- / 2502.0);
        }
        if (currentPWM < 1050) {
            dropping = false;
            currentPWM = 1450;
        }
    }
}

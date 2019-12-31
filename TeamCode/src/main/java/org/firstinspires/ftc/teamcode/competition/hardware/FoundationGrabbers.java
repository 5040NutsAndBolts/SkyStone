package org.firstinspires.ftc.teamcode.competition.hardware;

public class FoundationGrabbers {

    private Hardware robot;

    public FoundationGrabbers(Hardware robot) {
        this.robot = robot;
    }

    public void grab() {
        robot.foundationGrabber1.setPosition(.1);
        robot.foundationGrabber2.setPosition(0);
    }

    public void release() {
        robot.foundationGrabber1.setPosition(1);
        robot.foundationGrabber2.setPosition(1);
    }
}

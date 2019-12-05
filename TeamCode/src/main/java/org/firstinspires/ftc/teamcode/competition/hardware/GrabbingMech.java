package org.firstinspires.ftc.teamcode.competition.hardware;

public class GrabbingMech {

    private Hardware robot;

    public GrabbingMech(Hardware hwMap) {
        robot = hwMap;
    }

    public void reset() {
        robot.grabber.setPosition(.7);
    }

    public void grab() { robot.grabber.setPosition(.05); }

}

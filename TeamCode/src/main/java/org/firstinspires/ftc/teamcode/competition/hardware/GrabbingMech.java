package org.firstinspires.ftc.teamcode.competition.hardware;

public class GrabbingMech {

    private Hardware robot;

    public GrabbingMech(Hardware hwMap) {
        robot = hwMap;
    }

    public void resetStone() { robot.grabber.setPosition(.7); }

    public void grabStone() { robot.grabber.setPosition(.7); }

    public void resetFoundation() {
        robot.foundationGrabber1.setPosition(0);
        robot.foundationGrabber2.setPosition(0);
    }

    public void grabFoundation() {
        robot.foundationGrabber1.setPosition(1);
        robot.foundationGrabber2.setPosition(1);
    }


}

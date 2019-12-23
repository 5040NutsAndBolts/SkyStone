package org.firstinspires.ftc.teamcode.competition.hardware;

public class CapstoneMech {

    private Hardware robot;

    public CapstoneMech(Hardware hwMap) {
        robot = hwMap;
    }

    public void moveSlidesUp() {
        robot.capstoneSlides.setPower(1);
    }

    public void moveSlidesDown() {
        robot.capstoneSlides.setPower(-1);
    }

    public void holdSlides() { robot.capstoneSlides.setPower(0); }

}

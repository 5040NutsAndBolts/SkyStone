package org.firstinspires.ftc.teamcode.competition.hardware;

public class IntakeMech {

    private Hardware robot;

    public IntakeMech(Hardware r) {
        robot = r;
    }

    public void intakePower(double power) {
        robot.intakeLeft.setPower(power);
        robot.intakeRight.setPower(power);
    }

    public void placeStone(){}

}

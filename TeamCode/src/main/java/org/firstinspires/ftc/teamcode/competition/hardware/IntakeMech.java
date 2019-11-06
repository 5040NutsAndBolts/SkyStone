package org.firstinspires.ftc.teamcode.competition.hardware;

public class IntakeMech {

    private Hardware robot;

    public IntakeMech(Hardware r) {
        robot = r;
    }

    public void spinIn() {
        robot.intakeLeft.setPower(1);
        robot.intakeRight.setPower(1);
    }

    public void spinOut() {
        robot.intakeLeft.setPower(-1);
        robot.intakeRight.setPower(-1);
    }

    public void hold() {
        robot.intakeLeft.setPower(0);
        robot.intakeRight.setPower(0);
    }

}

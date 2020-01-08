package org.firstinspires.ftc.teamcode.competition.hardware;

public class IntakeMech {

    private Hardware robot;
    public double intakeSpeed = 1;

    public IntakeMech(Hardware r) {
        robot = r;
    }

    public void setPower(double intakeSpeed) {
        robot.intakeLeft.setPower(intakeSpeed);
        robot.intakeRight.setPower(intakeSpeed);
    }

}

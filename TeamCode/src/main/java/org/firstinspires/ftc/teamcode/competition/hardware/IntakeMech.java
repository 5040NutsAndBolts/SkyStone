package org.firstinspires.ftc.teamcode.competition.hardware;

public class IntakeMech {

    private Hardware robot;
    public double intakeSpeed = 1;
    private boolean released = false;

    public IntakeMech(Hardware r) {
        robot = r;
    }

    public void setPower(int intaking) {
        if (intaking == 1) {
            robot.intakeLeft.setPower(intakeSpeed);
            robot.intakeRight.setPower(intakeSpeed);
        } else if (intaking == -1) {
            robot.intakeLeft.setPower(-intakeSpeed);
            robot.intakeRight.setPower(-intakeSpeed);
        } else {
            robot.intakeLeft.setPower(0);
            robot.intakeRight.setPower(0);
        }
    }

    public void releaseIntake() {
        if (!released) {
            robot.intakeBlock.setPosition(.5);
            released = true;
        }
    }

}

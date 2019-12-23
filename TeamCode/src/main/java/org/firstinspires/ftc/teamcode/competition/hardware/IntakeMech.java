package org.firstinspires.ftc.teamcode.competition.hardware;

public class IntakeMech {

    private Hardware robot;
    public double intakeSpeed = 1;

    public IntakeMech(Hardware r) {
        robot = r;
    }

    public void setPower(int intaking) {
        if (intaking == 0){
            robot.intakeLeft.setPower(intakeSpeed);
            robot.intakeRight.setPower(intakeSpeed);
        } else if (intaking == 1){
            robot.intakeLeft.setPower(-intakeSpeed);
            robot.intakeRight.setPower(-intakeSpeed);
        }
        else {
            robot.intakeLeft.setPower(0);
            robot.intakeRight.setPower(0);
        }
    }

    public void guideOut() {
        robot.stoneGuide.setPosition(0);
    }

    public void guideIn() {
        robot.stoneGuide.setPosition(1);
    }

}

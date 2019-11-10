package org.firstinspires.ftc.teamcode.competition.hardware;

public class IntakeMech {

    private Hardware robot;

    public boolean leftInward = true;
    public boolean rightInward = true;

    public IntakeMech(Hardware r) {
        robot = r;
    }

    public void spinRight(double power){
        if (rightInward)
            robot.intakeRight.setPower(power);
        else
            robot.intakeRight.setPower(-power);
    }

    public void spinLeft(double power){
        if (leftInward)
            robot.intakeLeft.setPower(power);
        else
            robot.intakeLeft.setPower(-power);
    }

}

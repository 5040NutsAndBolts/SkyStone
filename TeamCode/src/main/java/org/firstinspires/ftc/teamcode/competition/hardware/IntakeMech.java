package org.firstinspires.ftc.teamcode.competition.hardware;

public class IntakeMech {

    private Hardware robot;

    public boolean leftInward = true;
    public boolean rightInward = true;

    public IntakeMech(Hardware r) {
        robot = r;
    }

    public void setPower(double power)
    {

        robot.intakeLeft.setPower(power);
        robot.intakeRight.setPower(power);

    }


}

package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeMech
{

    private Hardware robot;


    public IntakeMech(Hardware r)
    {

        robot = r;

    }

    public void intakePower(double power)
    {

        robot.intakeLeft.setPower(power);
        robot.intakeRight.setPower(power);

    }

    public void placeStone(){}

}

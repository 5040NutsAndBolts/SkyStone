package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeMech
{

    public DcMotor intakeLeft;
    public DcMotor intakeRight;

    public Servo stoneGuide;

    public IntakeMech(HardwareMap hwMap)
    {

        intakeLeft = hwMap.dcMotor.get("intakeLeft");
        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeRight = hwMap.dcMotor.get("intakeRight");
        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);

        stoneGuide = hwMap.servo.get("stoneGuide");

    }

    public void intakePower(){}

    public void placeStone(){}

}

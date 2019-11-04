package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm
{

    public DcMotor armMotor;

    public Servo leftClaw;
    public Servo rightClaw;

    public Arm(HardwareMap hwMap)
    {
        armMotor = hwMap.dcMotor.get("armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftClaw = hwMap.servo.get("leftClaw");

        rightClaw = hwMap.servo.get("rightClaw");
    }

}

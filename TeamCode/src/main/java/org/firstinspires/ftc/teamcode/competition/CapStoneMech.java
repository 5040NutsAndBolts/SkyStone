package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CapStoneMech
{

    public Servo capstonePlacer;
    public DcMotor capstoneStoneSlides;

    public CapStoneMech(HardwareMap hwMap)
    {

        capstonePlacer = hwMap.servo.get("capstonePlacer");

        capstoneStoneSlides = hwMap.dcMotor.get("capstoneStoneSlides");
        capstoneStoneSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capstoneStoneSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void placeCpstone(){}

    public void moveSlides(){}

}

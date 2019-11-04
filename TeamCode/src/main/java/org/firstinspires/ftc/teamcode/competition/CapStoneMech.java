package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CapStoneMech {

    private Hardware robot;

    public CapStoneMech(Hardware hwMap) {
        robot = hwMap;
    }

    public void placeCpstone(){}

    public void moveSlides(double power)
    {

        robot.capstoneSlides.setPower(power);

    }

}

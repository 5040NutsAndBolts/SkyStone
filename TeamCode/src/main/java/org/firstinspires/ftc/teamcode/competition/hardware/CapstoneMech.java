package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CapstoneMech {

    private Hardware robot;

    public CapstoneMech(Hardware hwMap) {
        robot = hwMap;
    }

    public void placeCpstone(){}

    public void moveSlides(double power) {
        robot.capstoneSlides.setPower(power);
    }

}

package org.firstinspires.ftc.teamcode.helperclasses;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RPMMeter {

    private DcMotor motor;

    private int rpm;
    private double updateSpeed;

    public RPMMeter(DcMotor motor, double updateSpeed) {
        this.motor = motor;
        this.updateSpeed = updateSpeed;
    }

    public void start() {

    }

    public int getRPM() {
        return rpm;
    }
}

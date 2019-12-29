package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.competition.helperclasses.RPMMeter;

@TeleOp(group = "Auto", name = "RPM")
public class MotorRPMTest extends OpMode {
    private DcMotor motor;

    private int speed = 10;
    private boolean lbPressed = false;
    private boolean rbPressed = false;
    private boolean xPressed = false;
    private boolean aPressed = false;
    private RPMMeter rpmMeter;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        rpmMeter = new RPMMeter(motor, 1440);
        rpmMeter.start();
    }


    @Override
    public void loop() {
        speed += gamepad1.left_bumper && !lbPressed ?
                -1 :
                gamepad1.right_bumper && !rbPressed ?
                        1 :
                        gamepad1.x && !xPressed ?
                                -5 :
                                gamepad1.a && !aPressed ?
                                        5 :
                                        0;
        lbPressed = gamepad1.left_bumper;
        rbPressed = gamepad1.right_bumper;
        aPressed = gamepad1.a;
        xPressed = gamepad1.x;
        if (speed == 0)
            speed = 1;
        motor.setPower(1.0 / speed);
        telemetry.addData("speed", 1.0 / speed);
        telemetry.addData("RPM", rpmMeter.getRPM());
        telemetry.update();
    }
}

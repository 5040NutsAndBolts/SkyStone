package org.firstinspires.ftc.teamcode.demobots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.helperclasses.RobotConstants;

import static java.lang.Math.abs;

@TeleOp(name="MotorController", group="DemoBot")
public class MotorController extends OpMode
{

    private DcMotor motorOne;
    private DcMotor motorTwo;
    private boolean pressed = false;
    private double speed = 2;

    @Override
    public void init()
    {

        motorOne = hardwareMap.dcMotor.get("one");
        motorTwo = hardwareMap.dcMotor.get("two");
        motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    boolean override = false;
    

    @Override
    public void loop()
    {

        // Controller values

        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;


        if(gamepad1.x)
        {

            motorOne.setPower(1/speed);
            motorTwo.setPower(1/speed);


        }

        if(leftBumper && !pressed)
        {
            speed += 0.1;
            pressed = true;
        }
        if(rightBumper && !pressed)
        {
            speed -= 0.1;
            pressed = true;
        }
        else if(pressed && !rightBumper&&!leftBumper)
        {
            pressed = false;
        }
        if(speed < 1)
        {
            speed = 1;
        }

        telemetry.addData("speed",1/speed);
        telemetry.addData("overridde",override);

    }

    public double getOrientedDriveValue() {
        return 0;
    }
}

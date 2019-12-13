package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.competition.helperclasses.RPMMeter;

@TeleOp(group="Auto",name = "RPM")
public class MotorRPMTest extends OpMode
{
    DcMotor motor;
    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        r=new RPMMeter(motor,1440);
        r.run();
    }

    int speed=100;
    boolean lb=false;
    boolean rb=false;
    boolean x = false;
    boolean a = false;
    RPMMeter r;
    @Override
    public void loop()
    {

        speed+= gamepad1.left_bumper&&!lb ? -1:gamepad1.right_bumper&&!rb ? 1: gamepad1.x&&!x ? -5: gamepad1.a&&!a ? 5:0;
        if(!gamepad1.left_bumper)
            lb=false;
        else
            lb=true;
        if(!gamepad1.right_bumper)
            rb=false;
        else
            rb=true;
        if(!gamepad1.a)
            a=false;
        else
            a=true;
        if(!gamepad1.x)
            x=false;
        else
            x=true;
        if(speed==0)
            speed=1;
        motor.setPower(1.0/speed);
        telemetry.addData("speed",1.0/speed);
        telemetry.addData("RPM",r.getRPM());
        telemetry.update();

    }
}

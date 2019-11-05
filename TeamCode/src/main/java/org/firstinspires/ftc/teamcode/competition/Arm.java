package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helperclasses.PID;

public class Arm {

    private Hardware robot;

    PID up;
    PID down;

    public Arm(Hardware hwMap)
    {
        robot = hwMap;
        up = new PID(100,0,.2,.05,.1);
        down=new PID(0,0,.1,.05,.1);
    }
    public void moveWithPid(boolean position)
    {

        if(position)
        {

            robot.armMotor.setTargetPosition(100);
            robot.armMotor.setPower(up.getPID());
            up.update(100,robot.armMotor.getCurrentPosition());
            down.resetPid();

        }
        else
        {

            robot.armMotor.setTargetPosition(0);
            robot.armMotor.setPower(down.getPID());
            down.update(0,robot.armMotor.getCurrentPosition());
            up.resetPid();

        }

    }

}

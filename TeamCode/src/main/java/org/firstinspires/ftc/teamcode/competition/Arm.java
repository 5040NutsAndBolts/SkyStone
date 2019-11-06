package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helperclasses.PID;
import org.firstinspires.ftc.teamcode.helperclasses.RobotConstants;

public class Arm {

    private Hardware robot;

    public static PID up;
    PID down;

    public Arm(Hardware hwMap)
    {
        robot = hwMap;
        up = new PID(300,0,RobotConstants.p,RobotConstants.i,RobotConstants.d);

    }
    public void moveWithPid(boolean position)
    {

        if(robot.armMotor.getCurrentPosition()>=600)
            {robot.armMotor.setPower(0);}
        else if(position)
        {

            robot.armMotor.setPower(up.getPID());
            up.update(300,robot.armMotor.getCurrentPosition());

        }



    }

}

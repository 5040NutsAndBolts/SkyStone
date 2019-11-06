package org.firstinspires.ftc.teamcode.competition.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helperclasses.PID;
import org.firstinspires.ftc.teamcode.helperclasses.RobotConstants;

public class TowerArm {

    private Hardware robot;

    private int towerArmHeight = 1000;

    public TowerArm(Hardware hwMap)
    {
        robot = hwMap;
        up = new PID(300,0, RobotConstants.p,RobotConstants.i,RobotConstants.d);

    }

    public static PID up;
    PID down;


    public void moveWithPid(boolean position)
    {

        if(robot.towerArmMotor.getCurrentPosition()>=600)
        {robot.towerArmMotor.setPower(0);}
        else if(position)
        {

            robot.towerArmMotor.setPower(up.getPID());
            up.update(300,robot.towerArmMotor.getCurrentPosition());

        }



    }

}

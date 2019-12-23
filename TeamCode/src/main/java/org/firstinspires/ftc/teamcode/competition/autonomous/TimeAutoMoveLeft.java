package org.firstinspires.ftc.teamcode.competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;

@Autonomous(group="Auto",name="LeftAuto")
public class TimeAutoMoveLeft extends LinearOpMode {
    Hardware robot = new Hardware();
    ElapsedTime e = new ElapsedTime();
    MecanumDrive drive = new MecanumDrive(robot);
    @Override
    public void runOpMode() throws InterruptedException
    {

        robot.init(hardwareMap);
        waitForStart();
        long endTime = System.currentTimeMillis() + 270;
        long dropTime = System.currentTimeMillis() + 200;

        while(System.currentTimeMillis() < endTime && opModeIsActive())
        {
            if(System.currentTimeMillis() > dropTime);
            {
                robot.clawLeft.setPosition(.7);
                robot.clawRight.setPosition(.7);
            }
            robot.towerArmMotor.setPower(-.7);
            telemetry.addData("encoder",robot.towerArmMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.towerArmMotor.setPower(.1);
        e.startTime();
        e.reset();
        while(e.seconds() < 1.7 && opModeIsActive())
        {

            drive.drive(0,1,0);

        }
        while(e.seconds() < 2 && opModeIsActive())
        {

            drive.drive(-1,0,0);

        }

    }
}

package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.PurePursuit.WayPoint;

import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.IntakeMech;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.competition.helperclasses.CheckPoint;
import org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool.c1;

@Autonomous(name="PP",group="Auto")
public class PurePursuitTest extends LinearOpMode {

    public static boolean point = true;

    Hardware robot = new Hardware();
    PurePursuit purePursuit = new PurePursuit(robot);
    MecanumDrive drive = new MecanumDrive(robot);
    IntakeMech intake = new IntakeMech(robot);
    @Override
    public void runOpMode() throws InterruptedException
    {

        ArrayList<WayPoint> p = new ArrayList();
        p.add(new WayPoint(20,24,10,.1,10));
        p.add(new WayPoint(23,0,10,.1,10));
        robot.init(hardwareMap);
        int n = 0;
        waitForStart();
        /*long endTime = System.currentTimeMillis() + 250;
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
        robot.towerArmMotor.setPower(0.1);*/

        while(opModeIsActive())
        {

            robot.updatePositionRoadRunner();
            if(point)
            {
                double[] move = purePursuit.followPath(p, 7);
                drive.drive(move[0]*Math.cos(robot.prevHeading)+move[1]*Math.sin(robot.prevHeading), move[1]*Math.cos(robot.prevHeading)+move[0]*Math.sin(robot.prevHeading), move[2]);
                ThreadPool.pool.submit(c1);
                telemetry.addData("forward",move[0]*Math.cos(robot.prevHeading)+move[1]*Math.sin(robot.prevHeading));
                telemetry.addData("sideways",move[0]*Math.cos(robot.prevHeading)+move[1]*Math.sin(robot.prevHeading));
                telemetry.addData("sideways",move[2]);
                telemetry.addData("x",robot.x);
                telemetry.addData("y",robot.y);
                telemetry.addData("heading",robot.prevHeading);
                telemetry.update();
            }else
            {

                if(robot.prevHeading<Math.PI/4)
                    drive.drive(0,0,1);
                else if(n<100)
                {

                    intake.setPower(1);
                    drive.drive(1,0,0);
                    n++;

                }else
                    {intake.setPower(1);}


            }

        }

    }
}

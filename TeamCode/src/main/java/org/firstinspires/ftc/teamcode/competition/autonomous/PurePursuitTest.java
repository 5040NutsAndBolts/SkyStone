package org.firstinspires.ftc.teamcode.competition.Autonomous;

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



@Autonomous(name="PP",group="Auto")
public class PurePursuitTest extends LinearOpMode {

    public static boolean point = true;

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException
    {

        robot.init(hardwareMap);
        PurePursuit purePursuit = new PurePursuit(robot);
        MecanumDrive drive = new MecanumDrive(robot);
        IntakeMech intake = new IntakeMech(robot);
        ArrayList<WayPoint> p = new ArrayList();
        p.add(new WayPoint(10,-4,10,.1,10));
        p.add(new WayPoint(25,10,10,.1,10));
        p.add(new WayPoint(40,-10,10,.1,10));

        purePursuit.initPath(p);

        purePursuit.lastGoal[0]=10;
        purePursuit.lastGoal[1]=-4;
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


            purePursuit.followPath(p,4,Math.PI/2);

            telemetry.addData("pid",purePursuit.pos.getPID());
            telemetry.addData("x",robot.x);
            telemetry.addData("y",robot.y);
            telemetry.addData("heading",robot.theta);
            telemetry.update();


        }

    }
}

package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.PurePursuit.WayPoint;
import org.firstinspires.ftc.teamcode.competition.hardware.Hardware;
import org.firstinspires.ftc.teamcode.competition.hardware.IntakeMech;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.PurePursuit.CheckPoint;

import static org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool.*;

import java.util.ArrayList;

@Autonomous(name = "Foundation", group = "Auto")
public class FoundationAutoPurePursuit extends LinearOpMode {
    Hardware robot = new Hardware();
    MecanumDrive drive = new MecanumDrive(robot);
    IntakeMech intake = new IntakeMech(robot);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        PurePursuit purePursuit = new PurePursuit(robot);
        CheckPoint c1 = new CheckPoint(30.5, 13, 2, robot);
        CheckPoint c2 = new CheckPoint(15, 7, 2, robot);

        ArrayList<WayPoint> p1 = new ArrayList();
        p1.add(new WayPoint(30.5, 13, 0));

        ArrayList<WayPoint> p2 = new ArrayList();
        p2.add(new WayPoint(15, 13, Math.PI / 2));
        p2.add(new WayPoint(15, 7, Math.PI / 2));

        waitForStart();

        purePursuit.initPath(p1, .4, .005, .55);
        pool.submit(c1);
        while (opModeIsActive() && !c1.isHit) {
            robot.updatePositionRoadRunner();

            purePursuit.followPath(p1, 4, 1.5);

            telemetry.addData("pid", purePursuit.pos.getPID());
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("heading", robot.theta);
            telemetry.update();
        }


        robot.foundationGrabber1.setPosition(1);
        robot.foundationGrabber2.setPosition(1);
        ElapsedTime e = new ElapsedTime();
        e.startTime();
        while (e.seconds() < 2) ;

        purePursuit.initPath(p2, .025, .0025, .3);
        pool.submit(c2);
        while (opModeIsActive() && !c2.isHit) {
            robot.updatePositionRoadRunner();

            purePursuit.followPath(p2, 4, 5);

            telemetry.addData("pid", purePursuit.pos.getPID());
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("heading", robot.theta);
            telemetry.update();
        }
    }
}

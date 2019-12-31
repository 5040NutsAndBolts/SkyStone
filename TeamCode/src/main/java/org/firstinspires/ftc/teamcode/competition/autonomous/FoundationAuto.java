package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PurePursuit.CheckPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.PurePursuit.WayPoint;
import org.firstinspires.ftc.teamcode.competition.hardware.MecanumDrive;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool.pool;

@Autonomous(group = "Auto", name = "Foundation Auto")
public class FoundationAuto extends AutoMethods {

    private boolean startingOnBlue = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);

        // Lets us move the robot during auto (disables brakes)
        drive.softBrakeMotors();

        PurePursuit purePursuit = new PurePursuit(robot);
        CheckPoint c1 = new CheckPoint(30.5, 12, 2, robot);
        CheckPoint c2 = new CheckPoint(0, 4, 4, robot);
        CheckPoint c3 = new CheckPoint(19, 35, 1, robot);
        CheckPoint c4 = new CheckPoint(-1, 74, 2, robot);

        ArrayList<WayPoint> p1 = new ArrayList();
        p1.add(new WayPoint(31.5, 12, 0));

        ArrayList<WayPoint> p2 = new ArrayList();
        p2.add(new WayPoint(0, 12, 0));
        p2.add(new WayPoint(0, 4, 0));

        ArrayList<WayPoint> p3 = new ArrayList();

        p3.add(new WayPoint(-1, 48, 0));
        p3.add(new WayPoint(20, 36, 0));

        ArrayList<WayPoint> p4 = new ArrayList();
        p4.add(new WayPoint(1,45,0));
        p4.add(new WayPoint(-1, 70, 0));

        while (!isStarted() && !isStopRequested()) {
            updateOdometryTelemetry();
        }

        purePursuit.initPath(p1, .4, .005, .55);
        pool.submit(c1);
        while (opModeIsActive() && !c1.isHit) {
            telemetry.addData("PID", purePursuit.pos.getPID());
            updateOdometryTelemetry();

            purePursuit.followPath(p1, 4, 2, 1.2);
        }

        drive.hardBrakeMotors();

        robot.foundationGrabber1.setPosition(1);
        robot.foundationGrabber2.setPosition(1);

        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < 1.5 && opModeIsActive()) ;

        purePursuit.initPath(p2, .05, .05, 0);
        pool.submit(c2);
        while (opModeIsActive() && !c2.isHit) {
            telemetry.addData("PID", purePursuit.pos.getPID());
            updateOdometryTelemetry();

            purePursuit.followPath(p2, 4, 3.1, .5);
        }
        robot.foundationGrabber1.setPosition(0);
        robot.foundationGrabber2.setPosition(0);
        drive.hardBrakeMotors();

        c3.start();

        timer.reset();
        while (timer.seconds() < 1.5 && opModeIsActive()) ;
        purePursuit.initPath(p3,.5,.1,.01);

        timer.reset();
        while (opModeIsActive() && !c3.isHit&&timer.seconds()<3) {
            telemetry.addData("PID", purePursuit.pos.getPID());
            updateOdometryTelemetry();

            purePursuit.followPath(p3, 4, 1.7, 1);
        }

        c4.start();

        purePursuit.initPath(p4);
        while (opModeIsActive() && !c3.isHit) {
            telemetry.addData("PID", purePursuit.pos.getPID());
            updateOdometryTelemetry();

            purePursuit.followPath(p4, 4, 1.7, 1);
        }

        c4.terminate();
        c3.terminate();
        c2.terminate();
        c1.terminate();
        displayEndAuto();
    }
}
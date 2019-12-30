package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PurePursuit.CheckPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.PurePursuit.WayPoint;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.competition.helperclasses.ThreadPool.pool;

@Autonomous(group = "Auto", name = "Foundation Auto")
public class FoundationAuto extends AutoMethods {

    private boolean startingOnBlue = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Lets us move the robot during auto (disables brakes)
        drive.softBrakeMotors();

        PurePursuit purePursuit = new PurePursuit(robot);
        CheckPoint c1 = new CheckPoint(30.5, 12, 2, robot);
        CheckPoint c2 = new CheckPoint(0, 0, 2, robot);

        ArrayList<WayPoint> p1 = new ArrayList();
        p1.add(new WayPoint(30.5, 12, 0));

        ArrayList<WayPoint> p2 = new ArrayList();
        p2.add(new WayPoint(0, 12, 0));
        p2.add(new WayPoint(0, 4, 0));

        while (!isStarted() && opModeIsActive()) {
            updateOdometryTeleop();
        }

        purePursuit.initPath(p1, .4, .005, .55);
        pool.submit(c1);
        while (opModeIsActive() && !c1.isHit) {
            telemetry.addData("PID", purePursuit.pos.getPID());
            updateOdometryTeleop();

            purePursuit.followPath(p1, 4, 2, 1.2);
        }

        drive.hardBrakeMotors();

        robot.foundationGrabber1.setPosition(1);
        robot.foundationGrabber2.setPosition(1);

        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < 1.5 && opModeIsActive()) ;

        purePursuit.initPath(p2, .04, .0065, 0);
        pool.submit(c2);
        while (opModeIsActive() && !c2.isHit) {
            telemetry.addData("PID", purePursuit.pos.getPID());
            updateOdometryTeleop();

            purePursuit.followPath(p2, 4, 3.1, .5);
        }
        robot.foundationGrabber1.setPosition(0);
        robot.foundationGrabber2.setPosition(0);
        drive.hardBrakeMotors();

        timer.reset();
        while (timer.seconds() < 2 && opModeIsActive()) ;
        c2.terminate();
        c1.terminate();

        displayEndAuto();
    }
}


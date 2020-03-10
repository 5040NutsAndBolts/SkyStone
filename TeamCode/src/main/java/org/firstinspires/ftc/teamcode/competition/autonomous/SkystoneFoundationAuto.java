package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

import static org.firstinspires.ftc.teamcode.helperclasses.ThreadPool.pool;


@Autonomous(group = "Auto", name = "Skystone+Foundation Auto")
public class SkystoneFoundationAuto extends AutoMethods {

    @Override
    public void runOpMode() {
        initAuto(true);

        // Release intake
        robot.intakeBlock.setPosition(0);
        timer.reset();
        timer.startTime();
        while (timer.seconds() < .5 && opModeIsActive())
            if (timer.seconds() > .3)
                intake.setPower(1);
        intake.setPower(0);

        if (onRed) {
            robot.resetOdometry(9,-103.5,3*Math.PI/2);
            timer.reset();
            while (timer.seconds() < .5 && opModeIsActive()) {
                robot.updatePositionRoadRunner();
                drive.drive(0, -.3, 0);
            }
            if(skystonePosition!=3)
                while (opModeIsActive() && !HelperMethods.inThreshold(robot.theta, Math.PI/2, 10)) {
                    robot.updatePositionRoadRunner();
                    drive.drive(0, 0, .75);
                }
        }
        else robot.resetOdometry(9,103.5,3 * Math.PI / 2);
        if(onRed)
        {

            timer.reset();
            timer.startTime();
            while (timer.seconds() < .1 && opModeIsActive()) {
                robot.updatePositionRoadRunner();
                drive.drive(0, -.6, 0);
            }

        }else
        {

            timer.reset();
            timer.startTime();
            if(skystonePosition!=3)
                while (timer.seconds() < .3 && opModeIsActive()) {
                    robot.updatePositionRoadRunner();
                    drive.drive(0, -.3, 0);
                }
            else
                while (timer.seconds() < .2 && opModeIsActive()) {
                    robot.updatePositionRoadRunner();
                    drive.drive(0, -.3, 0);
                }
        }
        // Goes to position to grab skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos1,
                    wp_grabSkystone1_pos1,
                    1,
                    .009,
                    .2,
                    6,
                    1.3,
                    .2
            );
        }

        else if (skystonePosition == 2) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos2,
                    wp_grabSkystone1_pos2,
                    .62,
                    .019,
                    0.1,
                    8,
                    1.2,
                    .4
            );
        }
        else if (skystonePosition == 3) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos3,
                    wp_grabSkystone1_pos3,
                    1.3,
                    .04,
                    0.3,
                    4,
                    1.5,
                    .15
            );
        }
        drive.hardBrakeMotors();

        // Move forward to intake the skystone
        timer.reset();
        timer.startTime();
        while (opModeIsActive() && timer.seconds() < .5) {
            intake.setPower(-timer.seconds()*1.5+.25);
        }
        if(skystonePosition != 3) {
            timer.reset();
            timer.startTime();
            while (opModeIsActive() && timer.seconds() < .5) {
                drive.drive(.3, 0, 0);
                intake.setPower(-1);
                robot.updatePositionRoadRunner();
            }
        }
        else {
            timer.reset();
            timer.startTime();
            while (opModeIsActive() && timer.seconds() < .9) {
                drive.drive(.3, 0, 0);
                robot.updatePositionRoadRunner();
                intake.setPower(-1);
            }
            timer.reset();
            timer.startTime();
            while (opModeIsActive() && timer.seconds() < .4) {
                robot.updatePositionRoadRunner();
                drive.drive(-.3, -.05, 0);
            }
        }
        drive.hardBrakeMotors();
        // Drive backwards to be out of way with skystone
        runPurePursuitPath(
                cp_prepareForDeposit,
                wp_prepareForDeposit
        );

        // Run to build zone with the skystone

            runPurePursuitPath(
                    cp_skystoneToFoundation,
                    wp_skystoneToFoundation,
                    4,
                    .75,
                    .25
            );

        intake.setPower(0);
        carriage.closeClaw();
        timer.reset();
        timer.startTime();
        while(timer.seconds()<.85) {
            robot.updatePositionRoadRunner();
            drive.drive(-.3, 0, 0);
        }
        drive.hardBrakeMotors();
        foundationGrabbers.grab();
        while(robot.intakeRight.getCurrentPosition()>-4000&&opModeIsActive())
        {

            robot.updatePositionRoadRunner();
            robot.clawExtension1.setPower(-.75);
            robot.clawExtension2.setPower(-.75);

        }
        robot.clawExtension1.setPower(0);
        robot.clawExtension2.setPower(0);
        timer.reset();
        carriage.openClaw();
        Thread t = new Thread()
        {

            @Override
            public void run()
            {

                waitTime(.4);
                while(robot.intakeRight.getCurrentPosition()<-15&&opModeIsActive())
                {

                    robot.clawExtension1.setPower(.75);
                    robot.clawExtension2.setPower(.75);

                }
                robot.clawExtension1.setPower(0);
                robot.clawExtension2.setPower(0);
            }

        };
        pool.submit(t);


        runPurePursuitPath(
                cp_awayFromFoundation,
                wp_awayFromFoundation,
                .25,
                .003,
                .15,
                5,
                1,
                .4
        );
        drive.hardBrakeMotors();
        foundationGrabbers.release();
        timer.reset();
        timer.startTime();
        while (opModeIsActive() && timer.seconds() < .7) {
            drive.drive(0, -0.4, 0);
            robot.updatePositionRoadRunner();
        }

        drive.hardBrakeMotors();

        pointTurnToAngle(3*Math.PI/2,30);
        // Drive back to the quarry
        runPurePursuitPath(
                cp_prepareForSecondSkystone,
                wp_prepareForSecondSkystone,
                .25,
                .003,
                .15,
                8,
                .7,
                .2
        );

        // Goes to position to grab second skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_skystone2_pos1_f,
                    wp_grabSkystone2_pos1_f,
                    .75,
                    0.02,
                    .07,
                    4,
                    2,
                    .3
            );
        }
        else if (skystonePosition == 2) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos2,
                    wp_grabSkystone2_pos2,
                    .7,
                    .01,
                    0.2,
                    5,
                    1.7,
                    .2
            );
        }
        else if (skystonePosition == 3) {
            runPurePursuitPath(
                    cp_skystone2_pos3_f,
                    wp_grabSkystone2_pos3_f,
                    .9,
                    .1,
                    .35,
                    4,
                    2,
                    .1
            );
        }

        // Move forward to intake the skystone
        timer.reset();
        timer.startTime();
        if (skystonePosition != 3) {
            while (opModeIsActive() && timer.seconds() < .5) {
                robot.updatePositionRoadRunner();
                drive.drive(.3, 0, 0);
                intake.setPower(-1);
            }
        }
        else {
            while (opModeIsActive() && timer.seconds() < .75) {
                robot.updatePositionRoadRunner();
                drive.drive(.3, 0, 0);
                intake.setPower(-1);
            }
        }
        drive.hardBrakeMotors();

        // Intake for a second to ensure the block is grabbed
        waitTime(1);

        // Run to build zone with the skystone

            runPurePursuitPath(
                    cp_skystoneToFoundation_2,
                    wp_skystoneToFoundation_2,
                    .5,
                    .02,
                    .2,
                    4,
                    .5,
                    .5);

        drive.hardBrakeMotors();
        intake.setPower(0);
        carriage.closeClaw();
        waitTime(.2);
        while(robot.intakeRight.getCurrentPosition()>-4000&&opModeIsActive())
        {

            robot.updatePositionRoadRunner();
            robot.clawExtension1.setPower(-.75);
            robot.clawExtension2.setPower(-.75);

        }
        robot.clawExtension1.setPower(0);
        robot.clawExtension2.setPower(0);
        timer.reset();
        carriage.openClaw();
        Thread t1 = new Thread()
        {

            @Override
            public void run()
            {

                waitTime(.4);
                while(robot.intakeRight.getCurrentPosition()<-15&&opModeIsActive())
                {

                    robot.clawExtension1.setPower(.75);
                    robot.clawExtension2.setPower(.75);

                }
                robot.clawExtension1.setPower(0);
                robot.clawExtension2.setPower(0);
            }

        };
        pool.submit(t1);
        // Places the block out the back

        // Park
        if (parkAgainstBridge) {
            runPurePursuitPath(
                    cp_parkBridge,
                    wp_parkBridge,
                    .06,
                    .005,
                    .07,
                    4,
                    1,
                    10
            );
        }
        else {
            runPurePursuitPath(
                    cp_parkWall,
                    wp_ParkWall,
                    .06,
                    .005,
                    .07,
                    4,
                    1,
                    10
            );
        }

        displayEndAuto();

        // Kill all the threads
        ThreadPool.renewPool();
    }
}
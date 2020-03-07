package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;


@Autonomous(group = "Auto", name = "Skystone Auto")
public class SkystoneAuto extends AutoMethods {

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
            while (timer.seconds() < .6 && opModeIsActive())
                robot.updatePositionRoadRunner();
                drive.drive(0,-.45,0);
            while (opModeIsActive() && !HelperMethods.inThreshold(robot.theta, Math.PI/2, 10)) {
                robot.updatePositionRoadRunner();
                drive.drive(0, 0, .75);
            }
        }
        else robot.resetOdometry(9,103.5,3 * Math.PI / 2);

        // Goes to position to grab skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos1,
                    wp_grabSkystone1_pos1,
                    1,
                    .009,
                    .225,
                    6,
                    1.3,
                    .2
            );
        }
        else if (skystonePosition == 2) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos2,
                    wp_grabSkystone1_pos2,
                    .6,
                    .018,
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
                    1.5,
                    .04,
                    0.4,
                    4,
                    2.5,
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
            }
        }
        else {
            timer.reset();
            timer.startTime();
            while (opModeIsActive() && timer.seconds() < .9) {
                drive.drive(.3, 0, 0);
                intake.setPower(-1);
            }
            timer.reset();
            timer.startTime();
            while (opModeIsActive() && timer.seconds() < .4) {
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
        if (skystonePosition!=3) {
            runPurePursuitPath(
                    cp_depositSkystone,
                    wp_depositSkystone,
                    4,
                    .75,
                    .25
            );
        }
        else {
            runPurePursuitPath(
                    cp_depositSkystonePos3,
                    wp_depositSkystonePos3,
                    4,
                    .75,
                    .15
            );
        }

        // Places the block out the robot
        placeBlock(0, .16);

        // Drive back to the quarry
        runPurePursuitPath(
                cp_prepareForSecondSkystone,
                wp_prepareForSecondSkystone,
                .25,
                .003,
                .15,
                8,
                .01,
                .5
        );

        // Goes to position to grab second skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos1,
                    wp_grabSkystone2_pos1,
                    .75,
                    0.02,
                    .07,
                    4,
                    1.5,
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
                    1.3,
                    .2
            );
        }
        else if (skystonePosition == 3) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos3,
                    wp_grabSkystone2_pos3,
                    .9,
                    .1,
                    .35,
                    4,
                    1.75,
                    .1
            );
        }

        // Move forward to intake the skystone
        timer.reset();
        timer.startTime();
        if (skystonePosition != 3) {
            while (opModeIsActive() && timer.seconds() < .5) {
                drive.drive(.3, 0, 0);
                intake.setPower(-1);
            }
        }
        else {
            while (opModeIsActive() && timer.seconds() < .75) {
                drive.drive(.3, 0, 0);
                intake.setPower(-1);
            }
        }
        drive.hardBrakeMotors();

        // Intake for a second to ensure the block is grabbed
        waitTime(1);

        // Run to build zone with the skystone
        if(skystonePosition != 3) {
            runPurePursuitPath(
                    cp_depositSkystone_2,
                    wp_depositSkystone_2,
                    4,
                    .5,
                    .5
            );
        }
        else {
            runPurePursuitPath(
                    cp_depositSkystone_2Pos3,
                    wp_depositSkystone_2Pos3,
                    .5,
                    .02,
                    .2,
                    4,
                    .5,
                    .5
            );
        }
        drive.hardBrakeMotors();
        intake.setPower(0);

        // Places the block out the back
        placeBlock(0, .16);

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
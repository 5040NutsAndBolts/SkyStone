package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.competition.helperclasses.HelperMethods;


@Autonomous(group = "Auto", name = "Skystone Auto")
public class SkystoneAuto extends AutoMethods {

    @Override
    public void runOpMode() {


        initAuto(true, 9, 135,  3*Math.PI / 2);

        // Release intake
        robot.intakeBlock.setPosition(.5);
        timer.reset();
        timer.startTime();
        while (timer.seconds() < .5 && opModeIsActive())
            if (timer.seconds() > .3)
                intake.setPower(1);
        intake.setPower(0);

        if (onRed) {
            timer.reset();
            while (timer.seconds() < .5 && opModeIsActive())
                drive.drive(0,-.5,0);
            while (opModeIsActive() && !HelperMethods.inThreshold(robot.theta, Math.PI/2, 10)) {
                robot.updatePositionRoadRunner();
                drive.drive(0, 0, .75);
            }
            drive.hardBrakeMotors();
        }

        // Goes to position to grab skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos1,
                    wp_grabSkystone1_pos1,
                    .2,
                    .008,
                    .5,
                    4,
                    1,
                    .2
            );
        }
        else if (skystonePosition == 2) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos2,
                    wp_grabSkystone1_pos2,
                    .155,
                    .006,
                    0.05,
                    6,
                    1.1,
                    .9
            );
        }
        else if (skystonePosition == 3) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos3,
                    wp_grabSkystone1_pos3,
                    .15,
                    .006,
                    0,
                    4,
                    1.2,
                    .8
            );
        }

        // Move forward to intake the skystone
        timer.reset();
        timer.startTime();
        if (skystonePosition != 3) {
            while (opModeIsActive() && timer.seconds() < 1.2) {
                drive.drive(.3, 0, 0);
                intake.setPower(-1);
            }

        }
        else {
            while (opModeIsActive() && timer.seconds() < .7) {
                drive.drive(.35, 0, 0);
                intake.setPower(-1);
            }
        }
        drive.hardBrakeMotors();

        // Drive backwards to be out of way with skystone
        runPurePursuitPath(
                cp_prepareForDeposit,
                wp_prepareForDeposit
        );
        intake.setPower(0);
        if (skystonePosition == 3) {
            intake.setPower(0);
            lift.closeClaw();
        }

        // Run to build zone with the skystone
        if (skystonePosition != 3) {
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
                    cp_depositSkystone,
                    wp_depositSkystonePos3,
                    4,
                    .75,
                    .25
            );
        }

        // Spit out the block

            // Drop the block out the back
            lift.extendClaw();
            waitTime(1);
            lift.openClaw();

            // Pull lift back in so it doesn't hit anything
            waitTime(.5);
            lift.retractClaw();

        // Point turn to get out of way
        while (opModeIsActive() && timer.seconds()<.3) {
            robot.updatePositionRoadRunner();
            drive.drive(0, 0, .85);
        }
        drive.hardBrakeMotors();

        // Point turn to be oriented correctly

        timer.reset();
        while (opModeIsActive() && timer.seconds()<.3) {
            robot.updatePositionRoadRunner();
            drive.drive(0, 0, .85);
        }
        drive.hardBrakeMotors();


        // Drive back to the quarry
        runPurePursuitPath(
                cp_prepareForSecondSkystone,
                wp_prepareForSecondSkystone,
                4,
                .75,
                .5
        );

        // Goes to position to grab second skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos1,
                    wp_grabSkystone2_pos1,
                    .17,
                    0.01,
                    .1,
                    4,
                    1.5,
                    .3
            );
        }
        else if (skystonePosition == 2) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos2,
                    wp_grabSkystone2_pos2,
                    .17 ,
                    .006,
                    0.05,
                    5,
                    1,
                    .2
            );
        }
        else if (skystonePosition == 3) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos3,
                    wp_grabSkystone2_pos3,
                    .15,
                    .006,
                    .8,
                    4,
                    1.5,
                    .1
            );
        }

        // Move forward to intake the skystone
        timer.reset();
        timer.startTime();
        if (skystonePosition == 1) {
            while(timer.seconds() < .75 && opModeIsActive()) {
                drive.drive(.25,0,0);
                intake.setPower(-.75);
            }
        }
        else {
            while (((skystonePosition == 2 && timer.seconds() < 1.25) ||
                    (skystonePosition == 3 && timer.seconds() < 1.1)) && opModeIsActive()) {
                drive.drive(.25, 0, 0);
                intake.setPower(-1);
            }
        }
        drive.hardBrakeMotors();

        // Intake for a second then grab the block
        waitTime(1);
        intake.setPower(0);
        lift.closeClaw();

        // Run to build zone with the skystone
        runPurePursuitPath(
                cp_depositSkystone_2,
                wp_depositSkystone,
                4,
                .65,
                .5
        );

        // Drop the block out the back
        lift.extendClaw();
        waitTime(1);
        lift.openClaw();

        // Pull lift back in so it doesn't hit anything
        waitTime(.5);
        lift.retractClaw();

        // Park
        if (parkAgainstBridge) {
            runPurePursuitPath(
                    cp_parkBridge,
                    wp_parkBridgeFromLeft,
                    .06,
                    .005,
                    .07,
                    4,
                    1.5,
                    10
            );
        }
        else {
            runPurePursuitPath(
                    cp_parkWall,
                    wp_parkWallFromLeft,
                    .06,
                    .005,
                    .07,
                    4,
                    1.5,
                    10
            );
        }

        displayEndAuto();
    }
}
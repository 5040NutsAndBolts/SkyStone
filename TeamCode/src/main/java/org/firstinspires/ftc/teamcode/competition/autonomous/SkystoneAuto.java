package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(group = "Auto", name = "Skystone Auto")
public class SkystoneAuto extends AutoMethods {

    @Override
    public void runOpMode() {
        initAuto(true, 9, 135, 3 * Math.PI / 2);

        robot.resetOdometry(9, robot.y, robot.theta);

        // Release intake
        robot.intakeBlock.setPosition(.5);
        timer.reset();
        timer.startTime();
        while (timer.seconds() < .5 && opModeIsActive())
            if (timer.seconds() > .3)
                intake.setPower(1);
        intake.setPower(0);

        // Goes to position to grab skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos1,
                    wp_grabSkystone1_pos1,
                    .15,
                    .006,
                    0.1,
                    4,
                    1.25,
                    1
            );
        }
        else if (skystonePosition == 2) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos2,
                    wp_grabSkystone1_pos2,
                    .155,
                    .006,
                    0.05,
                    4,
                    1.25,
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
        while (opModeIsActive() &&
                (skystonePosition != 3 && timer.seconds() < 1) ||
                (skystonePosition == 3 && timer.seconds() < 1.1)) {
            drive.drive(.3, 0, 0);
            intake.setPower(-.9);
        }
        intake.setPower(0);
        drive.hardBrakeMotors();

        // Drive backwards to be out of way with skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_prepareForDepositPos1,
                    wp_prepareForDeposit
            );
        }
        else {
            runPurePursuitPath(
                    cp_prepareForDeposit,
                    wp_prepareForDeposit
            );
        }

        // Run to build zone with the skystone
        runPurePursuitPath(
                cp_depositSkystone,
                wp_depositSkystone,
                4,
                .75,
                .25
        );

        // Spit out the block
        timer.reset();
        timer.startTime();
        while (opModeIsActive() && timer.seconds() < .55)
            intake.setPower(1);
        timer.reset();
        intake.setPower(0);


        // ==========================
        // WORKS PERFECTLY UP TO HERE
        // ==========================


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
                    .15,
                    0,
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
                    4,
                    1,
                    .3
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
                    1,
                    .3
            );
        }

        // Move forward to intake the skystone
        timer.reset();
        timer.startTime();
        // Position 1 and time 1.25 works
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
        intake.setPower(0);
        drive.hardBrakeMotors();

        // Drive backwards to be out of way with skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_prepareForDepositPos1_2,
                    wp_prepareForDeposit
            );
        }
        else {
            runPurePursuitPath(
                    cp_prepareForDeposit_2,
                    wp_prepareForDeposit
            );
        }

        // Run to build zone with the skystone
        runPurePursuitPath(
                cp_depositSkystone_2,
                wp_depositSkystone,
                4,
                .75,
                .5
        );

        // Spit out the block
        timer.reset();
        timer.startTime();
        while (opModeIsActive() && timer.seconds() < .6) {
            intake.setPower(1);
        }
        timer.reset();
        intake.setPower(0);

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
                    .5
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
                    .5
            );
        }

        displayEndAuto();
    }
}
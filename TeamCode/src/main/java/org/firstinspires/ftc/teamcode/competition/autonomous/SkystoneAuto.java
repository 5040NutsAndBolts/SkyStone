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
            while (timer.seconds() < .6 && opModeIsActive())
                drive.drive(0,-.45,0);
            while (opModeIsActive() && !HelperMethods.inThreshold(robot.theta, Math.PI/2, 10)) {
                robot.updatePositionRoadRunner();
                drive.drive(0, 0, .75);
            }
            while (timer.seconds() < .5 && opModeIsActive())
                drive.drive(0,-.5,0);
            drive.hardBrakeMotors();
        }

        // Goes to position to grab skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos1,
                    wp_grabSkystone1_pos1,
                    .9,
                    .009,
                    .3,
                    6,
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
                    .55,
                    .01,
                    0,
                    4,
                    2,
                    .1
            );
        }
        drive.hardBrakeMotors();
        // Move forward to intake the skystone
        timer.reset();
        timer.startTime();
        while (opModeIsActive() && timer.seconds() < .5) {
            intake.setPower(-timer.seconds()*1.5+.25);
        }
        if(skystonePosition!=3) {
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
                drive.drive(-.3, 0, 0);
            }
        }
        drive.hardBrakeMotors();

        // Drive backwards to be out of way with skystone
        runPurePursuitPath(
                cp_prepareForDeposit,
                wp_prepareForDeposit
        );



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
                    1.2,
                    .15
            );
        }
        intake.setPower(0);
        lift.closeClaw();
        waitTime(1);
        // Spit out the block

            // Drop the block out the back
            lift.extendClaw();
            waitTime(1);
            lift.openClaw();

            // Pull lift back in so it doesn't hit anything
            waitTime(.5);
            lift.retractClaw();




        // Drive back to the quarry
        runPurePursuitPath(
                cp_prepareForSecondSkystone,
                wp_prepareForSecondSkystone,
                .25,
                .003,
                .125,
                9,
                .75,
                .5
        );

        // Goes to position to grab second skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos1,
                    wp_grabSkystone2_pos1,
                    .4,
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
                    .8,
                    .07,
                    .05,
                    4,
                    1.5,
                    .1
            );
        }

        // Move forward to intake the skystone
        timer.reset();
        timer.startTime();
        if(skystonePosition!=3) {
            while (opModeIsActive() && timer.seconds() < .5) {
                drive.drive(.3, 0, 0);
                intake.setPower(-1);
            }
        }
        else
        {

            while (opModeIsActive() && timer.seconds() < .9) {
                drive.drive(.3, 0, 0);
                intake.setPower(-1);
            }
        }
        drive.hardBrakeMotors();

        // Intake for a second then grab the block
        waitTime(1);



        // Run to build zone with the skystone
        runPurePursuitPath(
                cp_depositSkystone_2,
                wp_depositSkystone_2,
                4,
                .65,
                .5
        );
        intake.setPower(0);
        lift.closeClaw();
        waitTime(.9);
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
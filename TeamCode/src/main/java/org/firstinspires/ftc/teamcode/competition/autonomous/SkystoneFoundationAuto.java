package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(group = "Auto", name = "Skystone+Foundation Auto")
public class SkystoneFoundationAuto extends AutoMethods {

    /**
     * Auto plan:
     * 1. Detect skystone with vision
     * 2. Intake the proper block and grab with the claw
     * 3. Move to the position to grab the foundation
     * 4. Grab foundation while dropping the block onto the foundation
     * 5. Go and grab second skystone
     * 6. Come back and push the foundation into the proper position
     * 7. Park
     */
    @Override
    public void runOpMode() {
        initAuto(true, 9, 135, 3 * Math.PI / 2);


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
                    0.15,
                    8,
                    1.3,
                    .2
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
                    1,
                    .7
            );
        }

        // Move forward to intake the skystone
        timer.reset();
        while (opModeIsActive() && timer.seconds() < .7) {
            drive.drive(.35, 0, 0);
            intake.setPower(-1);
        }
        drive.hardBrakeMotors();

        // Drive backwards to be out of way with skystone
        runPurePursuitPath(
                cp_prepareForDeposit,
                wp_prepareForDeposit
        );

        intake.setPower(0);
        lift.closeClaw();

        // Run to the foundation with skystone
        runPurePursuitPath(
                cp_skystoneToFoundation,
                wp_skystoneToFoundation,
                4,
                .5,
                .1
        );

        // Drop the block out the back
        lift.extendClaw();
        foundationGrabbers.grab();
        waitTime(1);
        lift.openClaw();

        // Pull lift back in so it doesn't hit anything
        waitTime(.4);
        lift.retractClaw();

        // Pull the foundation back
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.5)
            drive.drive(.5, 0, 0);
        foundationGrabbers.release();
        waitTime(.5);

        // Goes to position to grab second skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos1,
                    wp_grabSkystone2_pos1,
                    .17,
                    0.01,
                    .1,
                    4,
                    .75,
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
                    .75,
                    .2
            );
        }
        else if (skystonePosition == 3) {
            runPurePursuitPath(
                    cp_foundationGrabSkystone2_pos3,
                    wp_foundationGrabSkystone2_pos3,
                    .15,
                    .006,
                    .8,
                    4,
                    .75,
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
                cp_skystoneToFoundation_2,
                wp_skystoneToFoundation_2,
                4,
                .65,
                .5
        );

        // Park
        /*if (parkAgainstBridge) {
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
        */

        displayEndAuto();
    }
}
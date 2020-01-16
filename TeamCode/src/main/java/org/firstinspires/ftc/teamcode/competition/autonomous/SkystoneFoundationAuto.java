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
        robot.resetOdometry(9, robot.y, 3 * Math.PI / 2);

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
        timer.startTime();
        while (opModeIsActive() && timer.seconds() < .7) {
            drive.drive(.35, 0, 0);
            intake.setPower(-1);
        }
        drive.hardBrakeMotors();
        while (opModeIsActive() && timer.seconds() < .2) {
            intake.setPower(-1);
        }

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
                .2
        );



        // Drop the block out the back
        lift.extendClaw();
        foundationGrabbers.grab();
        waitTime(1);
        lift.openClaw();

        // Pull lift back in so it doesn't hit anything
        waitTime(.4);
        lift.retractClaw();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 2)
            drive.drive(.55, 0, 0);


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
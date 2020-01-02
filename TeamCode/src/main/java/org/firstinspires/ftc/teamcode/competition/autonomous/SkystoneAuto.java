package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(group = "Auto", name = "Skystone Auto")
public class SkystoneAuto extends AutoMethods {

    @Override
    public void runOpMode() {
        initAuto(true, 9, 135, 3 * Math.PI / 2);

        // Release intake
        robot.intakeBlock.setPosition(.5);

        /*
        New Auto Path:
        - Robot will start in red depot with phone facing stones
        - Humans will push robot to proper position
        - ** Depending on where Skystone is, move to that position
        - Then move forward and get the block stuck to the intake
        - Move to the exact same position to deposit a stone
        - If there is time, grab another
        - Otherwise, park
         */

        // Goes to position to grab skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos1,
                    wp_grabSkystone1_pos1,
                    .15,
                    .006,
                    0,
                    4,
                    1.25,
                    1
            );
        }
        else if (skystonePosition == 2) {
            runPurePursuitPath(
                    cp_grabSkystone1_pos2,
                    wp_grabSkystone1_pos2,
                    .15,
                    .006,
                    0,
                    4,
                    1.25,
                    1
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
                    1.25,
                    1
            );
        }

        // Go and deposit first skystone
        runWithSkystone();

        // Drive back to the quarry
        runPurePursuitPath(
                cp_prepareForSecondSkystone,
                wp_prepareForSecondSkystone
        );

        // Goes to position to grab skystone
        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos1,
                    wp_grabSkystone2_pos1,
                    .15,
                    .006,
                    0,
                    4,
                    1.25,
                    1
            );
        }
        else if (skystonePosition == 2) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos2,
                    wp_grabSkystone2_pos2,
                    .15,
                    .006,
                    0,
                    4,
                    1.25,
                    1
            );
        }
        else if (skystonePosition == 3) {
            runPurePursuitPath(
                    cp_grabSkystone2_pos3,
                    wp_grabSkystone2_pos3,
                    .15,
                    .006,
                    0,
                    4,
                    1.25,
                    1
            );
        }

        // Go and deposit second skystone
        runWithSkystone();

        // Park
        if (parkAgainstBridge);
        else;

        displayEndAuto();
    }

    /**
     * Pure Pursuit maneuvering with the skystone out of the quarry to deposit in build zone
     */
    private void runWithSkystone() {
        // Move forward to intake the skystone
        timer.reset();
        timer.startTime();
        while ((skystonePosition != 3 && timer.seconds() < 1.25) ||
                (skystonePosition == 3 && timer.seconds() < 1.3) && opModeIsActive()) {
            drive.drive(.25, 0, 0);
            intake.setPower(-1);
        }
        intake.setPower(0);
        drive.hardBrakeMotors();

        // Drive backwards to be out of way with skystone
        if (skystonePosition == 1)
            runPurePursuitPath(
                    cp_prepareForDepositPos1,
                    wp_prepareForDeposit
            );
        else
            runPurePursuitPath(
                    cp_prepareForDeposit,
                    wp_prepareForDeposit
            );

        // Run to build zone with the skystone
        runPurePursuitPath(
                cp_depositSkystone,
                wp_depositSkystone,
                4,
                1,
                .75
        );

        // Spit out the block
        timer.reset();
        timer.startTime();
        while (timer.seconds() < .3) {
            intake.setPower(1);
        }
        timer.reset();
        intake.setPower(0);
    }
}
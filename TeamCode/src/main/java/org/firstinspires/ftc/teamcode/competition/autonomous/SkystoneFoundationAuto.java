package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;


@Disabled
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
        initAuto(true, false, 9, 135, 3 * Math.PI / 2);
        carriage.openClaw();

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
            if(skystonePosition==2)
            {
                drive.drive(.25, 0, 0);
                intake.setPower(-1);
            }
            else{
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

        // Point turn to avoid hitting things
        timer.reset();
        while (opModeIsActive() && timer.seconds()<.3) {
            robot.updatePositionRoadRunner();
            drive.drive(0, 0, .85);
        }
        drive.hardBrakeMotors();
        carriage.closeClaw();

        // Run to the foundation with skystone
        runPurePursuitPath(
                cp_skystoneToFoundation,
                wp_skystoneToFoundation,
                .6,
                .0017555,
                .7,
                4,
                .4,.1

        );

        // Drop the block out the back
        carriage.extend();
        foundationGrabbers.grab();
        waitTime(1);
        carriage.openClaw();


        // Pull the foundation back
        if(!onRed) {
            timer.reset();
            while (opModeIsActive() && timer.seconds() < 2) {
                robot.updatePositionRoadRunner();
                drive.drive(.6, 0, 0);
            }
        }
        else{
            timer.reset();
            while (opModeIsActive() && timer.seconds() < 2) {
                robot.updatePositionRoadRunner();
                drive.drive(.4, 0, 0);
            }
        }
        foundationGrabbers.release();
        waitTime(.45);

        // Run parallel to the wall
        runPurePursuitPath(
                cp_awayFromFoundation,
                wp_awayFromFoundation
                );

        // Pull lift back in so it doesn't hit anything
        carriage.retract();
        waitTime(.25);

        runPurePursuitPath(
                cp_awayFromPartner,
                wp_awayFromPartner
        );

        // Point turn to not hit the partner when moving to
        timer.reset();
        while (opModeIsActive() && !HelperMethods.inThreshold(robot.theta, 3 * Math.PI / 2, 5)) {
            robot.updatePositionRoadRunner();
            drive.drive(0, 0, .5);
        }
        drive.hardBrakeMotors();

        runPurePursuitPath(
                cp_awayFromBridge,
                wp_awayFromBridge
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
            while (((skystonePosition == 2 && timer.seconds() < .8) ||
                    (skystonePosition == 3 && timer.seconds() < .7)) && opModeIsActive()) {
                drive.drive(.25, 0, 0);
                intake.setPower(-1);
            }
        }
        drive.hardBrakeMotors();

        // Intake for a second then grab the block
        waitTime(.1);
        intake.setPower(-.75);

        // Move to be able to sprint to the foundation
        runPurePursuitPath(
                cp_skystoneToFoundation_2,
                wp_skystoneToFoundation_2,
                4,
                .5,
                .5
        );
        carriage.closeClaw();

        // Drive backwards to hit the foundation
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.9) {
            robot.updatePositionRoadRunner();
            drive.drive(-.65, 0, 0);
        }
        waitTime(.05);
        intake.setPower(0);
        drive.hardBrakeMotors();

        // Drop the block out the back
        carriage.extend();
        carriage.openClaw();
        waitTime(.8);
        timer.reset();
        while(timer.seconds()<.4)
            drive.drive(0,-.6,0);
        waitTime(.1);

        foundationGrabbers.release();
        waitTime(.3);
        carriage.retract();
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
                    .7,
                    10
            );
        }

        displayEndAuto();
    }
}
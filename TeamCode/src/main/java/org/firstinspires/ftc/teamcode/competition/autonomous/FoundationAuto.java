package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Auto", name = "Foundation Auto")
public class FoundationAuto extends AutoMethods {

    @Override
    public void runOpMode() {
        initAuto(false);

        foundationGrabbers.release();

        // Run forward to grab foundation
        runPurePursuitPath(
                cp_foundationGrab,
                wp_foundationGrab,
                .4,
                0,
                1.1,
                4,
                1.5,
                1.2
        );

        timer.reset();
        while(timer.seconds()<.2)
            drive.drive(-.25,0,0);

        // Grab foundation
        foundationGrabbers.grab();
        waitTime(1);

        // Pull the foundation backwards for 4 seconds just for redundancy with the odometry
        timer.reset();
        timer.startTime();
        while (opModeIsActive() && timer.seconds() < 3)
            drive.drive(.4, 0, 0);
        drive.hardBrakeMotors();
        robot.resetOdometry(0, robot.y, 0);

        // Release the foundation
        foundationGrabbers.release();
        waitTime(1);

        // Push foundation towards the alliance wall
        runPurePursuitPath(
                cp_foundationPushImproved,
                wp_foundationPushImproved
        );
        timer.reset();
        while(timer.seconds()<1)
            drive.drive(.4,0,0);

        // Move to be in position to park
        runPurePursuitPath(
                cp_foundationParkPrep,
                wp_foundationParkPrep
        );

        // Parking
        if (!parkAgainstBridge) {
            runPurePursuitPath(
                    cp_foundationParkWall,
                    wp_foundationParkWall,
                    4,
                    1.7,
                    1
            );
        } else {
            runPurePursuitPath(
                    cp_foundationParkBridge,
                    wp_foundationParkBridge,
                    4,
                    1.7,
                    1
            );
        }

        displayEndAuto();
    }
}


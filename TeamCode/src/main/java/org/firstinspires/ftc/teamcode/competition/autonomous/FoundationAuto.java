package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Auto", name = "Foundation Auto")
public class FoundationAuto extends AutoMethods {

    @Override
    public void runOpMode() {
        initAuto(false);

        foundationGrabbers.release();

        runPurePursuitPath(
                cp_foundationGrab,
                wp_foundationGrab,
                new double[]{
                        .4,
                        .005,
                        .55,
                        4,
                        2,
                        1.2
                });

        foundationGrabbers.grab();

        waitTime(1.5);

        runPurePursuitPath(
                cp_foundationPull,
                wp_foundationPull,
                new double[]{
                        .05,
                        .05,
                        0,
                        4,
                        3.1,
                        .5
                });

        foundationGrabbers.release();

        waitTime(1.5);

        runPurePursuitPath(
                cp_foundationPush,
                wp_foundationPush,
                new double[]{
                        .5,
                        .1,
                        .01,
                        4,
                        1.7,
                        1
                });

        foundationGrabbers.grab();

        if (!parkAgainstBridge)
            runPurePursuitPath(
                    cp_foundationParkWall,
                    wp_foundationParkWall,
                    new double[]{
                            4,
                            1.7,
                            1
                    });

        displayEndAuto();
    }
}


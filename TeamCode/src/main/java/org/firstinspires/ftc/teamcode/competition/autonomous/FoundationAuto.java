package org.firstinspires.ftc.teamcode.competition.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Auto", name = "Foundation Auto")
public class FoundationAuto extends AutoMethods {

    @Override
    public void runOpMode() {
        initAuto();

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

        robot.foundationGrabber1.setPosition(1);
        robot.foundationGrabber2.setPosition(1);

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

        robot.foundationGrabber1.setPosition(0);
        robot.foundationGrabber2.setPosition(0);

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

        runPurePursuitPath(
                cp_foundationPark,
                wp_foundationPark,
                new double[]{
                        4,
                        1.7,
                        1
                });

        displayEndAuto();
    }
}


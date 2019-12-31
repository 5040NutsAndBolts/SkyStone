package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(group = "Auto", name = "Skystone Auto")
public class SkystoneAuto extends AutoMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(true);

        // Open the intake to grab block
        intake.releaseIntake();

        if (skystonePosition == 1) {
            runPurePursuitPath(
                    cp_grabSkystone_pos1,
                    wp_grabSkystone_pos1,
                    new double[]{
                            4, 2, 1.2
                    });

            runPurePursuitPath(
                    cp_depositSkystone_pos1,
                    wp_depositSkystone_pos1,
                    new double[]{
                            4, 1.5, 1.2
                    });
        }
        if (skystonePosition == 2) {
            runPurePursuitPath(
                    cp_grabSkystone_pos2,
                    wp_grabSkystone_pos2,
                    new double[]{
                            4, 1.5, 1.2
                    });

            runPurePursuitPath(
                    cp_depositSkystone_pos2,
                    wp_depositSkystone_pos2,
                    new double[]{
                            4, 1.5, 1.2
                    });
        }
        if (skystonePosition == 3) {
            runPurePursuitPath(
                    cp_grabSkystone_pos3,
                    wp_grabSkystone_pos3,
                    new double[]{
                            4, 1.5, 1.2
                    });

            runPurePursuitPath(
                    cp_depositSkystone_pos3,
                    wp_depositSkystone_pos3,
                    new double[]{
                            4, 1.5, 1.2
                    });
        }
    }
}
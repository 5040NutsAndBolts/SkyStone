package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(group = "Auto", name = "Test Auto")
public class TestAuto extends AutoMethods {

    @Override
    public void runOpMode() {
        initAuto(true);



        displayEndAuto();
    }
}
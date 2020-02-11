package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Disabled
@Config
@Autonomous(group = "Auto", name = "Test Auto")
public class TestAuto extends AutoMethods {

    public static int
            start_i = 1450,
            end_i = 1050,
            decr_val = 1;
    public static double
            uS_min = 650,
            uS_max = 2502;

    @Override
    public void runOpMode() {
        enableDashboard();

        ServoImplEx capDrop = hardwareMap.get(ServoImplEx.class, "capDrop");
        capDrop.setPwmRange(new PwmControl.PwmRange(uS_min, uS_max));

        waitForStart();

        for(int i=start_i; i>end_i; i-=decr_val) {
            capDrop.setPosition(i/uS_max);
            telemetry.addData("i",i);
            telemetry.update();
        }
    }
}
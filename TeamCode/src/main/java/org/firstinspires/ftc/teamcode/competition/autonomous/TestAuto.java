package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
@Autonomous(group = "Auto", name = "Test Auto")
public class TestAuto extends AutoMethods {

    public static double
            startPower = .5,
            endPower = -.5;
    public static int wait1 = 500, wait2 = 500;

    @Override
    public void runOpMode() {
        enableDashboard();

        CRServo capDrop = hardwareMap.get(CRServo.class, "capstoneDropper");

        waitForStart();

//        capDrop.setPower(start_i);
//        for(double i=end_i; i<start_i; i+=decr_val) {
//            capDrop.setPower(i);
//            telemetry.addData("i",i);
//            telemetry.update();
//        }
//        capDrop.setPower(start_i);
//
//        capDrop.setPower(end_i);
//        for(double i=start_i; i>end_i; i-=decr_val) {
//            capDrop.setPower(i);
//            telemetry.addData("i",i);
//            telemetry.update();
//        }
//        capDrop.setPower(end_i);
        capDrop.setPower(startPower);
        sleep(wait1);
        capDrop.setPower(endPower);
        sleep(wait2);
    }
}